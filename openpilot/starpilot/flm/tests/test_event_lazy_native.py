"""Exact selected-controller/CAN replay units, never native/profile authority."""
from dataclasses import replace
from types import SimpleNamespace
from typing import Protocol, cast
from unittest import mock

import pytest

from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.hyundai.values import CAR as HYUNDAI
from opendbc.car.toyota.values import CAR as TOYOTA
from openpilot.starpilot.flm.controller_replay import (
  ControllerReplay, ReplayFrame, SelectedControllerFactory, _run, capture_snapshot, final_command, frame_input,
)
from openpilot.starpilot.flm.event_controller_replay import EventControllerReplay, _LazyControllerRequests
from openpilot.starpilot.flm.event_objective import EffectObjective
from openpilot.starpilot.flm.pid_controller_replay import PidControllerFactory
from openpilot.starpilot.flm.torque_adapter import Objective
from openpilot.starpilot.flm.torque_emission_replay import Emission
from openpilot.starpilot.flm.tests.test_full_surface_profile import base
from openpilot.starpilot.flm.tests.test_steering_projection import make_interface, install_observer, settle, control, apply, BASE_NS


class ReplayWindowFixture(Protocol):
  """Mutable timestamp-pair windows consumed by the production replay engine."""
  replay_windows: tuple[tuple[int, int], ...]


class EagerRequests(_LazyControllerRequests):
  """Test-only original ordering: ALL controller work before native rollout."""
  def __init__(self, values, frames):
    super().__init__(values, frames)
    self.finish()


def replay_fixture(domain):
  platform = {'torque': TOYOTA.TOYOTA_COROLLA_TSS2,
              'starpilot': HYUNDAI.HYUNDAI_IONIQ_6, 'pid': HONDA.HONDA_CIVIC}[domain]
  ci = make_interface(platform)
  assert install_observer(ci)
  settle(ci, control(0.))
  factory = PidControllerFactory(ci.CP, 'native') if domain == 'pid' else SelectedControllerFactory(
    ci.CP, 'starpilot' if domain == 'starpilot' else 'standard')
  objective = Objective('feedforward', 'high', 'steady')
  original = ControllerReplay(factory, objective, identity='f' * 64, baseline_revision='a' * 64)
  actual, vm = factory.create()
  original.raw_mode = True
  first = BASE_NS + ci.CC.frame * 10_000_000
  original.checkpoint(first, capture_snapshot(actual, vm))
  rows = []
  for tick in range(44):
    stamp = first + tick * 10_000_000
    inputs = frame_input(ci.CS.out, vm, SimpleNamespace(angleOffsetDeg=0., roll=0.), active=True,
      safety_limited=False, desired_curvature=.00003, curvature_limited=False, lat_delay=.2)
    inputs += (('control_frame', float(tick + 1)), ('raw_model_curvature', .00003))
    value, _ = _run(actual, vm, inputs, objective, (0., 0.))
    rows.append(ReplayFrame(stamp, inputs, final_command(value), (0., 0.), frame_sequence=tick + 1,
      actual_lac_command=value, controller_domain='torque_pid' if domain == 'pid' else 'torque'))
    if tick >= 40:
      continue  # Controller-only tail MUST still run after native success.
    _, packets, witness = apply(ci, control(value))
    if witness is not None:
      assert (witness.can_address, witness.can_data, witness.can_bus) in packets
      clipped = witness.unlimited_raw is None or witness.unlimited_raw != witness.final_raw
      event = Emission(stamp, stamp, stamp + 1_000_000, witness, 'f' * 64, 'e' * 64, 'a' * 64,
        True, True, False, False, clipped, False, False, 0, 0, '')
      # As in registered codec units, native flags grant NO current authority.
      # We exercise original decode/EncoderBranch with actual captured bytes.
      original.emissions.events[event.apply_ns] = event
      original.emissions.decode_queue.append(event.apply_ns)
      original.emissions.by_control[event.control_ns] = event
  original.records.extend(rows)
  cast(ReplayWindowFixture, original).replay_windows = ((rows[0].control_ns, rows[-1].control_ns),)
  envelope = base()
  selected = EffectObjective('ff.left', envelope.trial_names, envelope.trial_anchor, envelope.context_key)
  return EventControllerReplay(original, selected, envelope.replay_anchor())


def run_observed(engine, eager=False, coordinates=(0.,)):
  trace = []
  engine.emissions.decoder = None  # Both paths independently decode real bytes.
  engine.emissions.decode_queue.clear()
  engine.emissions.decode_queue.extend(engine.emissions.events)
  engine.emissions.decode_errors.clear()
  def update(*args, **kwargs):
    result = _run(*args, **kwargs)
    trace.append((args[2], result[0], result[1].to_dict(), capture_snapshot(args[0], args[1])))
    return result
  with mock.patch('openpilot.starpilot.flm.event_controller_replay._run', side_effect=update), \
       mock.patch('openpilot.starpilot.flm.event_controller_replay._LazyControllerRequests',
         EagerRequests if eager else _LazyControllerRequests):
    try:
      return ('success', engine._rollout_uncached(coordinates)), trace
    except ValueError as error:
      return ('rejected', str(error)), trace


@pytest.mark.parametrize('domain', ('torque', 'starpilot', 'pid'))
@pytest.mark.parametrize('coordinates', ((0.,), (.00001,)))
def test_actual_selected_and_native_success_outputs_and_full_controller_trace_equal(domain, coordinates):
  engine = replay_fixture(domain)
  eager, before = run_observed(engine, eager=True, coordinates=coordinates)
  lazy, after = run_observed(engine, coordinates=coordinates)
  assert lazy == eager
  if lazy[0] == 'success':
    assert after == before and len(after) == len(engine.records)
    assert set(lazy[1][0]) == {row.control_ns for row in engine.records}
  if coordinates == (0.,):
    assert lazy[0] == 'success'


@pytest.mark.parametrize('domain', ('torque', 'starpilot', 'pid'))
def test_real_first_native_byte_failure_rejects_without_unneeded_controller_tail(domain):
  engine = replay_fixture(domain)
  stamp, event = next(iter(engine.emissions.events.items()))
  engine.emissions.events[stamp] = replace(event, witness=event.witness._replace(final_raw=event.witness.final_raw + 1))
  eager, before = run_observed(engine, eager=True)
  lazy, after = run_observed(engine)
  assert eager == lazy and lazy[0] == 'rejected' and 'actual steering CAN bytes mismatch' in lazy[1]
  assert 0 < len(after) < len(before) == len(engine.records)


@pytest.mark.parametrize('defect', ('late_controller', 'missing_last_tick', 'missing_consumed_request'))
def test_native_success_never_skips_late_controller_or_exact_endpoint_guards(defect):
  engine = replay_fixture('torque')
  if defect == 'late_controller':
    engine.records[-1] = replace(engine.records[-1], healthy=False)
  elif defect == 'missing_last_tick':
    first, last = engine.replay_windows[0]
    engine.engine.replay_windows = ((first, last + 10_000_000),)
  else:
    stamp, event = next(iter(engine.emissions.events.items()))
    engine.emissions.events[stamp] = replace(event, control_ns=event.control_ns + 5_000_000)
  eager, _ = run_observed(engine, eager=True)
  lazy, _ = run_observed(engine)
  assert lazy == eager and lazy[0] == 'rejected'


def test_lazy_mapping_repeated_older_keys_missing_key_and_explicit_finish():
  values, generated = {}, []
  def frames():
    for stamp in (10, 20, 30):
      generated.append(stamp)
      values[stamp] = stamp / 100
      yield stamp
  lazy = _LazyControllerRequests(values, frames())
  assert 20 in lazy and generated == [10, 20]
  assert lazy[10] == .1 and lazy[20] == .2 and generated == [10, 20]
  lazy.finish()
  assert generated == [10, 20, 30] and dict(lazy) == values
  assert 99 not in lazy
  with pytest.raises(KeyError):
    lazy[99]


@pytest.mark.parametrize('defect', ('success', 'native_failure', 'tail_failure'))
def test_controller_iterator_is_closed_on_every_exit_without_bypassing_finish(defect):
  engine = replay_fixture('torque')
  if defect == 'native_failure':
    stamp, event = next(iter(engine.emissions.events.items()))
    engine.emissions.events[stamp] = replace(event, witness=event.witness._replace(final_raw=event.witness.final_raw + 1))
  elif defect == 'tail_failure':
    engine.records[-1] = replace(engine.records[-1], healthy=False)
  closed = []
  original_close = _LazyControllerRequests.close
  def close(value):
    original_close(value)
    closed.append(value.frames.gi_frame is None)
  with mock.patch.object(_LazyControllerRequests, 'close', close):
    result, trace = run_observed(engine)
  assert closed == [True]
  assert result[0] == ('success' if defect == 'success' else 'rejected')
  if defect == 'success':
    assert len(trace) == len(engine.records)


def test_unknown_legacy_receiver_keeps_plain_dictionary_and_eager_order():
  from openpilot.starpilot.flm.tests.test_declared_restore import replay_fixture as legacy_fixture
  engine, restored, frames, emissions = legacy_fixture()
  coordinates = tuple(dict(restored.coordinates)[name] for name in engine.objective.names)
  seen = []
  def update(*args, **kwargs):
    seen.append(args[2])
    return 0., SimpleNamespace(saturated=False)
  def native(requests, *args, **kwargs):
    assert type(requests) is dict and len(requests) == len(frames) and len(seen) == len(frames)
    return {}
  emissions.rollout.side_effect = native
  with mock.patch('openpilot.starpilot.flm.event_controller_replay._run', side_effect=update), \
       mock.patch('openpilot.starpilot.flm.event_controller_replay.restore_snapshot'):
    engine._rollout_uncached(coordinates, restore_target=True)


@pytest.mark.parametrize('domain', ('torque', 'starpilot', 'pid'))
def test_eager_and_lazy_reset_reparse_every_original_can_emission(domain):
  from openpilot.starpilot.flm.torque_emission_replay import EmissionDecoder
  engine = replay_fixture(domain)
  expected = list(engine.emissions.events)
  parsed = []
  original_check = EmissionDecoder.check

  def check(decoder, event):
    parsed.append(event.apply_ns)
    return original_check(decoder, event)

  with mock.patch.object(EmissionDecoder, 'check', check):
    eager, _ = run_observed(engine, eager=True)
    first = len(parsed)
    lazy, _ = run_observed(engine)
  assert eager[0] == lazy[0] == 'success'
  assert parsed[:first] == expected
  assert parsed[first:] == expected
