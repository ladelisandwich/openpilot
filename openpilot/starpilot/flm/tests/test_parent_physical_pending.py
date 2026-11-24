"""Real optional parent + observer + typed collector, not a fitted-profile proof.

The SM/native world is declared synthetic. PoseCalibrator, observer, wire readers,
control/output joins and the actual parent loop/collector are not replaced.
"""
from openpilot.starpilot.flm.tests._drain_fixture import separate_drain_fixture
from dataclasses import replace
from types import SimpleNamespace
from unittest import mock

import pytest

from openpilot.starpilot.flm import steerlearnerd
from openpilot.starpilot.flm.fullrate_collector import FullrateCollector
from openpilot.starpilot.flm.physical_observation import PendingPhysical
from openpilot.starpilot.flm.runtime_adapter_registry import SourceState, physical_observer
from openpilot.starpilot.flm.tests.test_fullrate_pipeline import SyntheticAdapter, snapshots, serialized_control, serialized_output
from openpilot.starpilot.flm.tests.test_physical_pose_queue import sources

BASE = 20_000_000_000
KEY = ('feedforward', 'low', 'steady')


def exercise(*, failure=None, polls=86, pending=True):
  sm = sources(BASE)
  sm['lateralDelay'].lateralDelay = 0.
  cp = SimpleNamespace(steerRatio=14., wheelbase=2.9, centerToFront=1.2, mass=2000.)
  clock = [BASE + 2_000_000]
  native = [True]
  loop = [-1]
  collectors, observations, histories = [], [], []
  control_socket, output_socket = object(), object()

  class Adapter(SyntheticAdapter):
    event_collection = raw_storage = False
    def select_objective(self, key):
      pass
    def prepare_control(self, value):
      return value

  def source_reader(now):
    return SourceState(True, False, '0', '2' * 64, now - 100_000_000, now + 400_000_000,
      'b' * 64, (0., 0., 0.), calibration_generation='c' * 32)

  def update(timeout=None):
    sm.updated = dict.fromkeys(sm.messages, False)
    if timeout == 0:
      return
    loop[0] += 1
    if loop[0] >= polls:
      raise StopIteration
    pair, right = divmod(loop[0], 2)
    pose = BASE + pair * 50_000_000
    state = pose + (10_000_000 if right else -10_000_000)
    clock[0] = pose + (12_000_000 if right else 2_000_000)
    source_reader_override[0] = False
    native[0] = True
    sm['carState'].canValid = True
    sm['deviceMotion'].inputsOK = True
    sm['deviceMotion'].timestamp = pose
    sm.logMonoTime['deviceMotion'] = pose
    sm.updated['deviceMotion'] = not right
    sm.updated['carState'] = sm.updated['lateralDelay'] = True
    sm.recv_time['deviceMotion'] = (pose + 2_000_000) / 1e9
    sm.logMonoTime['carState'] = state
    sm.logMonoTime['lateralDelay'] = clock[0]
    if failure is not None and loop[0] == 1:
      if failure == 'fault':
        sm['carState'].canValid = False
      elif failure == 'native':
        native[0] = False
      elif failure == 'invalid_pose':
        sm['deviceMotion'].inputsOK = False
      elif failure == 'expired_pose':
        sm['deviceMotion'].timestamp = clock[0] - 100_000_001
      elif failure == 'state':
        sm.logMonoTime['carState'] = clock[0] - 100_000_001
      elif failure == 'source':
        source_reader_override[0] = True
      elif failure == 'context':
        collectors[0].split('synthetic_external_context_boundary')

  source_reader_override = [False]
  def read_source(now):
    source = source_reader(now)
    return replace(source, source_epoch='d' * 64) if source_reader_override[0] else source

  sm.update = update
  def collector_factory(adapter):
    collector = FullrateCollector(adapter)
    collectors.append(collector)
    return collector

  def row():
    state = sm.logMonoTime['carState']
    control, output, _ = snapshots(state + 1_000_000)
    ratio = 16. if failure == 'vm' and loop[0] == 1 else 14.
    values = (('vm.sR', ratio), ('vm.cF', 100000.), ('vm.cR', 100000.), ('vEgo', 20.), ('steeringAngleDeg', 0.))
    control = replace(control, state_ns=state, source_receipt_ns=clock[0] - 100_000_000,
      replay_inputs=values)
    output = replace(output, state_ns=state)
    return serialized_control(control), serialized_output(output)

  def drain(socket, callback, receive, sink, captured):
    control, output = row()
    callback(control if socket is control_socket else output, sink, captured)

  def observe(sm, cp, collector, now):
    result = physical_observer(sm, cp, collector, now, distinguish_pending=pending)
    observations.append(result)
    # Record only existing state; no evidence, health or clocks are rewritten.
    histories.append((collector.active_since, collector.model_block_sequence, collector.last_response))
    return result

  def boot_clock(*args):
    return clock[0] + (2_000_000 if failure == 'clock' and loop[0] == 1 else 0)

  fitter = mock.Mock(child=object())
  fitter.poll.return_value = None
  persistence = mock.Mock()
  with mock.patch.object(steerlearnerd, 'FullrateCollector', side_effect=collector_factory), \
       mock.patch.object(steerlearnerd, 'drain_pair', side_effect=separate_drain_fixture(drain)), \
       mock.patch('openpilot.starpilot.flm.native_health.current_native_health', side_effect=lambda *args: native[0]), \
       mock.patch('openpilot.starpilot.flm.runtime_adapter_registry.time.monotonic_ns', side_effect=lambda: clock[0]), \
       mock.patch('openpilot.starpilot.flm.runtime_adapter_registry.time.clock_gettime_ns', side_effect=boot_clock):
    with pytest.raises(StopIteration):
      steerlearnerd.run(cp, sm, control_socket, output_socket, mock.Mock(),
        lambda *args: Adapter(), observe, read_source, lambda _: (KEY,), lambda _: KEY,
        mock.Mock(), persistence, fitter, mock.Mock(), clock=lambda: clock[0])
  return collectors, observations, histories


def test_actual_parent_keeps_healthy_pending_pose_until_original_right_bracket():
  collectors, observations, histories = exercise()
  assert len(collectors) == 1
  collector = collectors[0]
  assert type(observations[0]) is PendingPhysical
  recovered = observations[1]
  assert recovered.mono_ns == BASE and recovered.received_ns == BASE + 2_000_000
  assert histories[1][0] == histories[0][0] > 0
  assert histories[1][1] == histories[0][1]
  assert collector.active_since == histories[0][0]
  assert collector.last_response > BASE + 2_000_000_000
  assert collector.points  # Existing2s warmup completed through the REAL observer.
  assert not collector.training_interruptions['physical_or_native_witness_unavailable']
  assert collector.fit_result is None and collector.result is None  # No manufactured fit/admission.


def test_original_undifferentiated_parent_path_loses_original_pose_and_warmup():
  collectors, observations, histories = exercise(pending=False, polls=2)
  assert observations[0] is None and observations[1] is None
  assert collectors[0].training_interruptions['physical_or_native_witness_unavailable'] == 2
  assert collectors[0].active_since == 0 and collectors[0].last_response == 0
  assert histories[1][1] > histories[0][1]


@pytest.mark.parametrize('failure', ['fault', 'native', 'invalid_pose', 'expired_pose'])
def test_real_parent_hard_invalidity_still_splits_and_never_becomes_pending(failure):
  collectors, observations, _ = exercise(failure=failure, polls=2)
  collector = collectors[0]
  assert collector.active_since == collector.last_response == 0
  assert collector.training_interruptions['physical_or_native_witness_unavailable']
  assert collector.fit_result is None and collector.result is None
  assert observations[-1] is None


def test_source_transition_clears_collector_and_queued_original_pose():
  collectors, observations, _ = exercise(failure='source', polls=2)
  assert type(observations[0]) is PendingPhysical
  assert len(observations) == 1
  assert not collectors[0].ready and collectors[0].active_since == 0
  assert collectors[0].fit_result is None and collectors[0].result is None


@pytest.mark.parametrize('mono,received,now', [
  (BASE, BASE, BASE + 100_000_001), (BASE, BASE + 1, BASE),
  (BASE + 1, BASE, BASE + 1), (True, BASE, BASE), (BASE, BASE, float(BASE)),
])
def test_pending_type_never_grants_future_or_stale_wait(mono, received, now):
  assert not PendingPhysical(mono, received).fresh(now)


@pytest.mark.parametrize('failure', ['fault', 'native', 'invalid_pose', 'expired_pose', 'clock', 'context', 'state', 'vm', 'source'])
def test_single_hard_boundary_recovers_only_with_new_original_pose_and_full_warmup(failure):
  collectors, observations, histories = exercise(failure=failure, polls=90)
  assert len(collectors) == (2 if failure == 'source' else 1)
  collector = collectors[-1]
  if failure == 'source':
    assert not collectors[0].ready and collectors[0].last_qualified_response == 0
  assert collector.active_since > histories[0][0]  # Boundary really restarted warmup.
  assert collector.last_response > collector.active_since + 2_000_000_000
  assert collector.points
  assert collector.fit_result is None and collector.result is None
  # The queued pose from before the boundary is never recovered afterward.
  returned = [value for value in observations[1:] if value is not None and type(value) is not PendingPhysical]
  assert returned and all(value.mono_ns > BASE for value in returned)
  assert collector.model_block_sequence < 8  # No self-perpetuating pending/reset loop.
