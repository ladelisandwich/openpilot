"""Controller-only preactivation reuse: actual selected controllers and native CAN."""
from dataclasses import replace
from types import ModuleType, SimpleNamespace
from typing import cast
from unittest import mock
import pickle
import sys

import pytest

from openpilot.starpilot.flm.controller_replay import (
  ControllerReplay, ReplayFrame, SelectedControllerFactory, _run, capture_snapshot, final_command, frame_input,
)
from openpilot.starpilot.flm.effect_trace import FullEffectTrace
from openpilot.starpilot.flm.event_activation import EventActivationPlan, ZERO_REFERENCE_ACTIVATION_VERSION
from openpilot.starpilot.flm.event_controller_replay import EventControllerReplay
from openpilot.starpilot.flm.event_objective import EffectObjective
from openpilot.starpilot.flm.pid_controller_replay import PidControllerFactory
from openpilot.starpilot.flm.selected_effects import EffectReplayRecordedSource
from openpilot.starpilot.flm.torque_adapter import Objective
from openpilot.starpilot.flm.torque_emission_replay import Emission
from openpilot.starpilot.flm.tests.test_full_surface_profile import base
from openpilot.starpilot.flm.tests.test_event_lazy_native import (
  TOYOTA, HYUNDAI, HONDA, make_interface, install_observer, settle, control, apply, BASE_NS, ReplayWindowFixture,
)


def fixture(domain='torque', nonzero=False, two_windows=False):
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
  envelope = base()
  if nonzero:
    envelope = base(prior=base().with_proposal({'ff_gain_left': .002}))
  rows = []
  for tick in range(60):
    stamp = first + tick * 10_000_000
    if two_windows and tick == 30:
      original.checkpoint(stamp, capture_snapshot(actual, vm))
    trace = FullEffectTrace(1, envelope, stamp, stamp - 10_000_000, True,
      envelope.coordinates, envelope.coordinates, envelope.coordinates, envelope.full_profile_sha256).validate()
    inputs = frame_input(ci.CS.out, vm, SimpleNamespace(angleOffsetDeg=.01, roll=.002), active=True,
      safety_limited=False, desired_curvature=.00003, curvature_limited=False, lat_delay=.2)
    inputs += (('control_frame', float(tick + 1)), ('raw_model_curvature', .00003))
    value, _ = _run(actual, vm, inputs, objective, (0., 0.), effect_trial=EffectReplayRecordedSource(trace))
    rows.append(ReplayFrame(stamp, inputs, final_command(value), (0., 0.), frame_sequence=tick + 1,
      actual_lac_command=value, controller_domain='torque_pid' if domain == 'pid' else 'torque',
      effect_trace=trace, effect_prepare_ns=stamp))
    _, packets, witness = apply(ci, control(value))
    if witness is not None:
      assert (witness.can_address, witness.can_data, witness.can_bus) in packets
      clipped = witness.unlimited_raw is None or witness.unlimited_raw != witness.final_raw
      event = Emission(stamp, stamp, stamp + 1_000_000, witness, 'f' * 64, 'e' * 64, 'a' * 64,
        True, True, False, False, clipped, False, False, 0, 0, '')
      original.emissions.events[event.apply_ns] = event
      original.emissions.decode_queue.append(event.apply_ns)
      original.emissions.by_control[event.control_ns] = event
  original.records.extend(rows)
  cast(ReplayWindowFixture, original).replay_windows = (((rows[0].control_ns, rows[29].control_ns), (rows[30].control_ns, rows[-1].control_ns))
    if two_windows else ((rows[0].control_ns, rows[-1].control_ns),))
  selected = EffectObjective('ff.left', envelope.trial_names, envelope.trial_anchor, envelope.context_key)
  activation = rows[45 if two_windows else 30].control_ns
  windows = (((rows[0].control_ns, rows[29].control_ns, 0, 0),
              (rows[30].control_ns, rows[-1].control_ns, activation, activation)) if two_windows else
             ((first, rows[-1].control_ns, activation, activation),))
  plan = EventActivationPlan(ZERO_REFERENCE_ACTIVATION_VERSION, original.identity, original.baseline_revision,
    selected.context_key, 'b' * 64, tuple('train-' + str(i) for i in range(4)),
    tuple('held-' + str(i) for i in range(4)), activation - 10_000_000,
    tuple('stage-' + str(i) for i in range(9)), windows).validate()
  return EventControllerReplay(original, selected, envelope.replay_anchor(), activation_plan=plan)


def state(value, seen=None):
  """Independent vars/containers audit, not capture_snapshot against itself."""
  seen = set() if seen is None else seen
  if value is None or type(value) in (bool, int, float, str, bytes):
    return value
  label = type(value).__module__ + '.' + type(value).__qualname__
  # Constructor bindings are identities, not mutable numerical state objects.
  # In particular, memoryview.tolist on the CLASS is an unbound descriptor.
  if isinstance(value, ModuleType):
    return ('module', value.__name__, getattr(value, '__file__', None))
  if isinstance(value, type):
    return ('class', value.__module__, value.__qualname__, getattr(sys.modules.get(value.__module__), '__file__', None))
  if callable(value):
    return ('callable', getattr(value, '__module__', None), getattr(value, '__qualname__', None), label,
            getattr(sys.modules.get(getattr(value, '__module__', None)), '__file__', None))
  if id(value) in seen:
    return ('reference', label)
  seen.add(id(value))
  if isinstance(value, dict):
    return label, tuple((state(k, seen), state(v, seen)) for k, v in value.items())
  if isinstance(value, (tuple, list)) or label == 'collections.deque':
    return label, tuple(state(v, seen) for v in value), getattr(value, 'maxlen', None)
  if hasattr(value, 'to_dict'):
    return label, value.to_dict()
  if hasattr(value, 'tolist'):
    return label, value.tolist()
  if hasattr(value, '__dict__'):
    return label, tuple((k, state(v, seen)) for k, v in vars(value).items())
  return label, str(value)


def observed(engine, coordinates, shadow=True):
  calls, native = [], []
  original_native = engine.emissions.rollout
  def update(*args, **kwargs):
    result = _run(*args, **kwargs)
    calls.append((args[2], result[0], result[1].to_dict(), state(args[0]), state(args[1])))
    return result
  def emit(requests, *args, **kwargs):
    result = original_native(requests, *args, **kwargs)
    native.append((dict(requests), result, kwargs))
    return result
  token = {'_identification_token': engine._EventControllerReplay__identification_token} if shadow else {}
  with mock.patch('openpilot.starpilot.flm.event_controller_replay._run', side_effect=update), \
       mock.patch.object(engine.emissions, 'rollout', side_effect=emit):
    result = engine._rollout(coordinates, **token)
  return result, calls, native


@pytest.mark.parametrize('domain', ('torque', 'starpilot', 'pid'))
@pytest.mark.parametrize('nonzero', (False, True))
@pytest.mark.parametrize('shadow, delta', ((False, .000000001), (True, .000000001), (True, .01), (True, -.01)))
def test_actual_prefix_reuse_full_tail_state_logs_requests_and_native_maps_exact(domain, nonzero, shadow, delta):
  cached = fixture(domain, nonzero)
  # Original full strict baseline must succeed before a certificate can exist.
  baseline = cached._rollout(cached.reference)
  certificate = cached._EventControllerReplay__controller_prefix
  assert certificate is not None and len(certificate[1]) == 30
  # The original guard checks raw Float64, NOT the rounded native request map.
  assert all(pickle.dumps(certificate[1][row.control_ns], protocol=5) ==
             pickle.dumps(row.actual_lac_command, protocol=5) for row in list(cached.records)[:30])
  with pytest.raises(TypeError):
    certificate[1][0] = 0.
  eager = EventControllerReplay(cached.engine, cached.objective, cached.original_anchor, activation_plan=cached.activation_plan)
  coordinates = (cached.reference[0] + delta,)
  expected, before, native_before = observed(eager, coordinates, shadow=shadow)
  actual, after, native_after = observed(cached, coordinates, shadow=shadow)
  assert pickle.dumps(actual, protocol=5) == pickle.dumps(expected, protocol=5)
  assert len(before) == 60 and len(after) == 30
  assert after == before[30:]  # Every actual treated log + independent vars(controller/VM).
  assert native_before == native_after and len(native_after[0][0]) == 60
  assert set(actual[0]) == set(baseline[0])


@pytest.mark.parametrize('change', ('input', 'checkpoint', 'plan', 'prior_bits', 'native', 'context', 'late_fault'))
def test_mutated_owned_inputs_never_use_stale_prefix(change):
  engine = fixture()
  engine._rollout(engine.reference)
  before = engine._EventControllerReplay__controller_prefix
  if change == 'input':
    row = engine.records[0]
    engine.records[0] = replace(row, inputs=row.inputs + (('extra', 1.),))
  elif change == 'checkpoint':
    stamp, snapshot = engine.checkpoints[0]
    engine.checkpoints[0] = (stamp, replace(snapshot, values=snapshot.values + (('unknown', 1.),)))
  elif change == 'plan':
    engine.activation_plan = replace(engine.activation_plan, physical_context='c' * 64)
  elif change == 'prior_bits':
    engine.prior_full['ff_gain_right'] = -0.
  elif change == 'native':
    stamp, event = next(iter(engine.emissions.events.items()))
    engine.emissions.events[stamp] = replace(event, witness=event.witness._replace(final_raw=event.witness.final_raw + 1))
  elif change == 'context':
    engine.factory.manual_source_epoch = 'changed'
  else:
    engine.records[-1] = replace(engine.records[-1], healthy=False)
  assert engine._causal_input_key(.01) != before[0]
  with mock.patch('openpilot.starpilot.flm.event_controller_replay._run', wraps=_run) as run:
    try:
      engine._rollout((.00001,), _identification_token=engine._EventControllerReplay__identification_token)
    except (ValueError, KeyError, TypeError):
      pass
    assert run.call_count > 0 or change in ('checkpoint', 'context')


def test_failure_unknown_operation_and_bad_shadow_token_never_create_certificate():
  engine = fixture()
  stamp, event = next(iter(engine.emissions.events.items()))
  engine.emissions.events[stamp] = replace(event, witness=event.witness._replace(final_raw=event.witness.final_raw + 1))
  with pytest.raises(ValueError):
    engine._rollout(engine.reference)
  assert engine._EventControllerReplay__controller_prefix is None
  engine = fixture()
  with pytest.raises(ValueError, match='identification scope'):
    engine._rollout((.00001,), _identification_token=object())
  assert engine._EventControllerReplay__controller_prefix is None
  # Old/default operation has no new certificate even after a complete baseline.
  engine.activation_plan = None
  engine._rollout(engine.reference)
  assert engine._EventControllerReplay__controller_prefix is None


def test_every_original_prefix_context_guard_and_late_tail_guard_still_runs():
  engine = fixture()
  engine._rollout(engine.reference)
  calls = []
  original = engine.engine._frame_context
  def context(frame):
    calls.append(frame.control_ns)
    return original(frame)
  with mock.patch.object(engine.engine, '_frame_context', side_effect=context):
    engine._rollout((.00001,), _identification_token=engine._EventControllerReplay__identification_token)
  assert calls == [frame.control_ns for frame in engine.records]
  engine.records[-1] = replace(engine.records[-1], healthy=False)
  with pytest.raises(ValueError, match='source/history'):
    engine._rollout((.00001,), _identification_token=engine._EventControllerReplay__identification_token)


def test_baseline_source_changed_during_success_never_publishes_pending_certificate():
  engine = fixture()
  with mock.patch.object(engine, '_causal_input_key', side_effect=(b'first', b'changed')):
    with pytest.raises(ValueError, match='changed owned input'):
      engine._rollout(engine.reference)
  assert engine._EventControllerReplay__controller_prefix is None
  assert engine._EventControllerReplay__pending_controller_prefix is None


def test_shadow_prefix_source_changed_during_success_rejects_and_discards_certificate():
  engine = fixture()
  engine._rollout(engine.reference)
  key = engine._EventControllerReplay__controller_prefix[0]
  with mock.patch.object(engine, '_causal_input_key', side_effect=(key, b'changed')):
    with pytest.raises(ValueError, match='changed owned input'):
      engine._rollout((.000000001,), _identification_token=engine._EventControllerReplay__identification_token)
  assert engine._EventControllerReplay__controller_prefix is None


def test_restore_endpoint_never_uses_trial_prefix_certificate():
  engine = fixture()
  engine._rollout(engine.reference)
  with pytest.raises(ValueError, match='restore endpoint'):
    engine._rollout(engine.reference, restore_target=True)


def test_unavailable_activation_snapshot_keeps_original_baseline_and_creates_no_certificate():
  engine = fixture()
  with mock.patch('openpilot.starpilot.flm.event_controller_replay.capture_snapshot',
                  side_effect=ValueError('unsupported transient state')):
    result = engine._rollout(engine.reference)
  assert len(result[0]) == 60
  assert engine._EventControllerReplay__controller_prefix is None


def test_unsupported_exact_controller_policy_keeps_original_replay_without_certificate():
  engine = fixture('starpilot')
  original_create = engine.factory.create
  def create():
    controller, vm = original_create()
    # Inherit all actual methods/state but leave the audited exact-class set.
    original = type(controller.starpilot_extension)
    controller.starpilot_extension.__class__ = type('UnknownExtension', (original,), {})
    return controller, vm
  with mock.patch.object(engine.factory, 'create', side_effect=create):
    result = engine._rollout(engine.reference)
  assert len(result[0]) == 60
  assert engine._EventControllerReplay__controller_prefix is None


def test_actual_raw_double_prefix_guard_preserves_non_float32_exact_command():
  engine = fixture()
  assert any(row.actual_lac_command != final_command(row.actual_lac_command) for row in list(engine.records)[:30])
  engine._rollout(engine.reference)
  certificate = engine._EventControllerReplay__controller_prefix
  assert any(value != final_command(value) for value in certificate[1].values())
  result, tail, _ = observed(engine, (.000000001,))
  assert len(result[0]) == 60 and len(tail) == 30


def test_legacy_shell_without_new_private_fields_keeps_original_argument_api():
  from openpilot.starpilot.flm.tests.test_event_strict_rollout_cache import engine, numeric
  value = engine()
  with mock.patch.object(value, '_rollout_uncached', side_effect=numeric) as original:
    value._rollout(value.reference)
    assert original.call_args.kwargs == {}
    value.activation_plan = None
    value._rollout((.01,))
    assert original.call_args.kwargs == {'restore_target': False, 'restore_omit_group': None, '_identification_token': None}


@pytest.mark.parametrize('domain', ('torque', 'starpilot', 'pid'))
def test_actual_second_window_checkpoint_reset_and_native_partition_remain_exact(domain):
  cached = fixture(domain, nonzero=True, two_windows=True)
  cached._rollout(cached.reference)
  certificate = cached._EventControllerReplay__controller_prefix
  assert certificate is not None and len(certificate[1]) == 15 and len(certificate[3]) == 1
  eager = EventControllerReplay(cached.engine, cached.objective, cached.original_anchor, activation_plan=cached.activation_plan)
  expected, before, native_before = observed(eager, (cached.reference[0] + .01,))
  actual, after, native_after = observed(cached, (cached.reference[0] + .01,))
  assert pickle.dumps(actual, protocol=5) == pickle.dumps(expected, protocol=5)
  assert len(before) == 60 and len(after) == 45
  assert after == before[:30] + before[45:]
  assert native_after == native_before


def test_state_observer_handles_module_class_and_memoryview_instance_without_dropping_values():
  import math
  assert state(memoryview) == ('class', 'builtins', 'memoryview', None)
  assert state(math) == ('module', 'math', getattr(math, '__file__', None))
  assert state(memoryview(b'ab')) == ('builtins.memoryview', [97, 98])
  assert state(SimpleNamespace(number=3., mutable=[1., 2.], constructor=memoryview)) != \
         state(SimpleNamespace(number=3., mutable=[1., 4.], constructor=memoryview))
