"""Per-job deterministic reuse only; no manufactured physical/native acceptance."""
from dataclasses import replace
from types import SimpleNamespace
from unittest import mock

import pytest
from opendbc.car.car_helpers import interfaces
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.toyota.values import CAR as TOYOTA
from openpilot.starpilot.flm.canonical_surface_adapter import parameters_for, NAMES
from openpilot.starpilot.flm.controller_replay import SelectedControllerFactory
from openpilot.starpilot.flm.pid_controller_replay import PidControllerFactory
from openpilot.starpilot.flm.event_controller_replay import EventControllerReplay
from openpilot.starpilot.flm.event_objective import EffectObjective
from openpilot.starpilot.flm.selected_effects import EffectTrial, EffectReplayTransition, CanonicalSurfaceEffects
from openpilot.starpilot.flm.tests.test_full_surface_profile import base


def engine_for(domain):
  if domain == 'pid':
    cp = interfaces[HONDA.HONDA_CIVIC].get_non_essential_params(HONDA.HONDA_CIVIC)
    factory = PidControllerFactory(cp, 'native')
  else:
    cp = interfaces[TOYOTA.TOYOTA_RAV4_TSS2].get_non_essential_params(TOYOTA.TOYOTA_RAV4_TSS2)
    factory = SelectedControllerFactory(cp, 'standard')
  frame = SimpleNamespace(control_ns=1, inputs=(('safety_limited', False),))
  actual = SimpleNamespace(factory=factory, identity='a' * 64, baseline_revision='b' * 64,
    model_revision='c' * 64, failed=False, raw_mode=True, objective=('original',), records=[frame],
    checkpoints=((1, ('exact-original-state',)),), replay_windows=((1, 2),),
    emissions=SimpleNamespace(cp=factory.cp, identity='a' * 64, baseline_revision='b' * 64,
      events={1: ('original-native-event',)}, carriers={1: (1, 1)}))
  engine = EventControllerReplay.__new__(EventControllerReplay)
  engine.engine = actual
  engine.objective = SimpleNamespace(names=('ff_gain_left',), parameters=parameters_for(('ff_gain_left',)))
  engine.reference, engine.prior_full, engine.original_anchor = (0.,), {'ff_gain_left': 0.}, {'full30': (0.,)}
  engine.activation_plan = engine.restore_envelope = None
  engine.projected = {}
  engine._EventControllerReplay__causal_features = None
  return engine


def rollout(coordinates):
  # Symbolic deterministic native trace for cache behavior, NEVER a validation.
  return {1: coordinates[0]}, {1: coordinates[0]}, {1: False}


@pytest.mark.parametrize('domain', ('torque', 'pid'))
def test_real_registered_factory_first_builds_then_reuses_exact_readonly_derived_features(domain):
  engine = engine_for(domain)
  assert type(engine._causal_input_key(.01)) is bytes
  with mock.patch.object(engine, '_rollout', side_effect=rollout) as native:
    assert engine.ensure_causal_features() == 1 and native.call_count == 3
    original = dict(engine.projected)
    engine.projected[1] = ('untrusted-public-edit',)
    assert engine.ensure_causal_features() == 1 and engine.projected == original
    assert native.call_count == 3
    _, cached = engine._EventControllerReplay__causal_features
    with pytest.raises(TypeError):
      cached[1] = (0., (9.,))
    assert engine.ensure_causal_features(epsilon=.02) == 1 and native.call_count == 6


@pytest.mark.parametrize('field', ('frame', 'native', 'checkpoint', 'anchor', 'objective', 'plan', 'source', 'factory'))
def test_changed_exact_owned_input_does_not_reuse_previous_feature_result(field):
  engine = engine_for('torque')
  with mock.patch.object(engine, '_rollout', side_effect=rollout) as native:
    engine.ensure_causal_features()
    if field == 'frame':
      engine.records[0].inputs = (('safety_limited', True),)
    elif field == 'native':
      engine.emissions.events[1] = ('changed-native-event',)
    elif field == 'checkpoint':
      engine.engine.checkpoints = ((1, ('changed-checkpoint',)),)
    elif field == 'anchor':
      engine.original_anchor['full30'] = (-0.,)  # Bit-sensitive, unlike dataclass equality.
    elif field == 'objective':
      engine.objective.parameters = tuple(replace(p, excitation_floor=.02) for p in engine.objective.parameters)
    elif field == 'plan':
      engine.activation_plan = SimpleNamespace(identification_policy='strict-final-quality-v1')
    elif field == 'source':
      engine.engine.baseline_revision = 'd' * 64
    else:
      engine.factory.manual_source_epoch = 'changed-source'
    engine.ensure_causal_features()
    assert native.call_count == 6


def test_boolean_integer_type_and_missing_factory_cache_keys_never_alias_readiness():
  engine = engine_for('pid')
  before = engine._causal_input_key(.01)
  engine.records[0].inputs = (('safety_limited', 0),)
  assert engine._causal_input_key(.01) != before
  engine.engine.factory = SimpleNamespace()
  assert engine._causal_input_key(.01) is None
  with mock.patch.object(engine, '_rollout', side_effect=rollout) as native:
    engine.ensure_causal_features()
    engine.ensure_causal_features()
    assert native.call_count == 6


def test_failed_full_build_cannot_create_ready_result_and_restore_still_rejects():
  engine = engine_for('pid')
  with mock.patch.object(engine, '_rollout', side_effect=ValueError('native/source guard')):
    with pytest.raises(ValueError):
      engine.ensure_causal_features()
  assert engine._EventControllerReplay__causal_features is None
  engine.restore_envelope = object()
  with pytest.raises(ValueError, match='restoration requires'):
    engine.ensure_causal_features()


def transition(coordinate=.005):
  envelope = base(('ff_gain_left',))
  objective = EffectObjective('ff.left', envelope.trial_names, envelope.trial_anchor, envelope.context_key)
  return EffectReplayTransition(EffectTrial(objective, (coordinate,), envelope.selected_baseline_revision, envelope.replay_anchor()))


def test_transition_reuses_only_exact_plateau_geometry_but_resets_each_real_frame_counter():
  owner = transition()
  with mock.patch('openpilot.starpilot.flm.selected_effects.CanonicalSurfaceEffects', wraps=CanonicalSurfaceEffects) as constructor:
    last = None
    for tick in range(110):
      current = owner.source_for_frame(1_000_000_000 + tick * 10_000_000)
      current.begin_frame(1_000_000_000 + tick * 10_000_000)
      current.selected_friction(.1, original_anchor=current.original_effect_anchor)
      assert current.friction_calls == 1
      with pytest.raises(ValueError):
        current.selected_friction(.1, original_anchor=current.original_effect_anchor)
      if tick >= 102:
        assert current.source is last.source
      last = current
    assert constructor.call_count <= 101  # Ramp configs change; plateau configs do not.
    previous = last.source
    owner.reset()
    fresh = owner.source_for_frame(3_000_000_000)
    assert fresh.source is not previous


def test_cached_and_fresh_geometry_fields_are_bit_exact_and_candidates_do_not_share_state():
  owner = transition(0.)
  first = owner.source_for_frame(1_000_000_000)
  first.begin_frame(1_000_000_000)
  second = owner.source_for_frame(1_010_000_000)
  second.begin_frame(1_010_000_000)
  assert second.source is first.source
  values = dict(owner.applied)
  fresh = CanonicalSurfaceEffects({'ff_gain_left': values['ff_gain_left']}, owner.trial.selected_baseline_revision, accumulated=values)
  for name in ('values', 'names', 'applied_coordinates', 'original_effect_anchor', 'full_original_anchor', 'full_profile_sha256'):
    assert getattr(second.source, name) == getattr(fresh, name)
  assert second.source.surface.knobs == fresh.surface.knobs
  other = transition(0.).source_for_frame(1_010_000_000)
  assert other.source is not second.source
  with pytest.raises(ValueError, match='nonmonotonic'):
    second.begin_frame(1_010_000_000)
  assert tuple(values) == NAMES


def test_plateau_cache_keeps_per_frame_trace_validation_and_exact_stage_outputs():
  from openpilot.starpilot.flm.tests.test_canonical_surface_adapter import CanonicalSurfaceTest
  owner = transition(.005)
  for tick in range(105):
    source = owner.source_for_frame(1_000_000_000 + tick * 10_000_000)
    source.begin_frame(1_000_000_000 + tick * 10_000_000)
  values = dict(owner.applied)
  fresh = CanonicalSurfaceEffects({'ff_gain_left': values['ff_gain_left']}, owner.trial.selected_baseline_revision, accumulated=values)
  fresh.begin_frame(2_040_000_000)
  for speed, setpoint in ((8., .2), (30., -.2)):
    frame = CanonicalSurfaceTest().frame(speed=speed, setpoint=setpoint)
    assert source.selected_stage(frame, original_anchor=source.original_effect_anchor) == fresh.selected_stage(
      frame, original_anchor=fresh.original_effect_anchor)
    source.begin_frame(source.source.frame_ns + 1)
    fresh.begin_frame(fresh.frame_ns + 1)
  invalid = SimpleNamespace(validate=mock.Mock(side_effect=ValueError('invalid current trace')))
  with pytest.raises(ValueError, match='invalid current trace'):
    owner.source_for_frame(2_100_000_000, invalid)
  invalid.validate.assert_called_once()


def test_uninitialized_legacy_shell_never_recurses_or_mints_cache_readiness():
  from openpilot.starpilot.flm.tests.test_event_fd_boundary import engine as legacy_engine
  engine = legacy_engine()
  assert engine._causal_input_key(.01) is None
  with mock.patch.object(engine, '_rollout', side_effect=rollout) as native:
    assert engine.ensure_causal_features() == 1
    assert engine.ensure_causal_features() == 1
    assert native.call_count == 6
  assert engine._EventControllerReplay__causal_features is None
