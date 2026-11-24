"""Bounded deterministic memo tests, not controller/native/physical qualification."""
from dataclasses import replace
from types import SimpleNamespace
from unittest import mock

import pytest

from openpilot.starpilot.flm.event_activation import ZERO_REFERENCE_ACTIVATION_VERSION, EXACT_EMISSION_ACTIVATION_VERSION
from openpilot.starpilot.flm.event_controller_replay import POLICY
from openpilot.starpilot.flm.tests.test_event_activation import fitted_model, primitive_plan
from openpilot.starpilot.flm.tests.test_event_replay_cost import engine_for, rollout


def numeric(coordinates, **_kwargs):
  return rollout(coordinates)


def engine(domain='torque'):
  result = engine_for(domain)
  result.activation_plan = replace(primitive_plan(fitted_model()), version=ZERO_REFERENCE_ACTIVATION_VERSION).validate()
  result._EventControllerReplay__strict_rollouts = None
  return result


@pytest.mark.parametrize('domain', ('torque', 'pid'))
def test_exact_owned_registered_factory_baseline_and_last_candidate_only_readonly_copyout(domain):
  value = engine(domain)
  with mock.patch.object(value, '_rollout_uncached', side_effect=numeric) as original:
    before = value._rollout(value.reference)
    candidate = value._rollout((.01,))
    assert original.call_count == 2
    before[0][1] = 99.
    candidate[1][1] = 99.
    assert value._rollout(value.reference) == rollout(value.reference)
    assert value._rollout((.01,)) == rollout((.01,))
    assert original.call_count == 2
    _, slots = value._EventControllerReplay__strict_rollouts
    assert set(slots) == {'baseline', 'candidate'}
    with pytest.raises(TypeError):
      slots['baseline'][1][0][1] = 1.
    value._rollout((.02,))
    value._rollout((.01,))  # Only the last successful candidate is retained.
    assert original.call_count == 4 and len(slots) == 2


@pytest.mark.parametrize('change', ('frame', 'native', 'checkpoint', 'anchor', 'objective', 'plan', 'failed', 'factory', 'reference'))
def test_every_changed_owned_input_misses_and_replaces_all_slots(change):
  value = engine()
  with mock.patch.object(value, '_rollout_uncached', side_effect=numeric) as original:
    value._rollout(value.reference)
    value._rollout((.01,))
    if change == 'frame':
      value.records[0].inputs = (('safety_limited', 0),)  # False and0 must not alias.
    elif change == 'native':
      value.emissions.events[1] = ('different-native-source',)
    elif change == 'checkpoint':
      value.engine.checkpoints = ((1, ('different-checkpoint',)),)
    elif change == 'anchor':
      value.original_anchor['full30'] = (-0.,)
    elif change == 'objective':
      value.objective.names = ('ff_gain_right',)
    elif change == 'plan':
      value.activation_plan = replace(value.activation_plan, stage_blocks=tuple(reversed(value.activation_plan.stage_blocks)))
    elif change == 'failed':
      value.engine.failed = True
    elif change == 'factory':
      value.factory.manual_source_epoch = 'different-source'
    else:
      value.reference = (-0.,)
    value._rollout((.01,))
    assert original.call_count == 3
    assert set(value._EventControllerReplay__strict_rollouts[1]) == {'candidate'}


def test_exact_coordinate_types_signed_zero_unknown_key_and_errors_never_create_alias():
  value = engine()
  with mock.patch.object(value, '_rollout_uncached', side_effect=numeric) as original:
    for coordinates in ((0.,), (0,), (-0.,)):
      value._rollout(coordinates)
    assert original.call_count == 3
    with mock.patch.object(value, '_causal_input_key', return_value=None):
      value._rollout((.01,))
      value._rollout((.01,))
    assert original.call_count == 5 and value._EventControllerReplay__strict_rollouts is None
  with mock.patch.object(value, '_rollout_uncached', side_effect=ValueError('strict native failure')) as failure:
    for _ in range(2):
      with pytest.raises(ValueError, match='strict native failure'):
        value._rollout((.01,))
    assert failure.call_count == 2
    assert not value._EventControllerReplay__strict_rollouts[1]


def test_shadow_restore_ablation_old_plan_and_unsupported_types_always_execute_original_path():
  for kwargs in ({'_identification_token': object()}, {'restore_target': True}, {'restore_omit_group': ('ff_gain_left',)}):
    value = engine()
    with mock.patch.object(value, '_rollout_uncached', side_effect=numeric) as original:
      value._rollout(value.reference)
      slots = dict(value._EventControllerReplay__strict_rollouts[1])
      value._rollout((.01,), **kwargs)
      value._rollout((.01,), **kwargs)
      assert original.call_count == 3 and value._EventControllerReplay__strict_rollouts[1] == slots
      assert original.call_args.kwargs == {'restore_target': kwargs.get('restore_target', False),
        'restore_omit_group': kwargs.get('restore_omit_group'), '_identification_token': kwargs.get('_identification_token')}
  value = engine()
  value.activation_plan = replace(value.activation_plan, version=EXACT_EMISSION_ACTIVATION_VERSION)
  with mock.patch.object(value, '_rollout_uncached', side_effect=numeric) as original:
    value._rollout((.01,))
    value._rollout((.01,))
    assert original.call_count == 2 and value._EventControllerReplay__strict_rollouts is None
  value = engine()
  with mock.patch.object(value, '_rollout_uncached', side_effect=TypeError('original type rejection')) as original:
    for coordinates in ((False,), ['x'], object()):
      with pytest.raises(TypeError, match='original type rejection'):
        value._rollout(coordinates)
    assert original.call_count == 3 and value._EventControllerReplay__strict_rollouts is None


def test_midexecution_input_mutation_cannot_be_stored_as_success():
  value = engine()
  def changing(coordinates):
    value.records[0].inputs = (('changed-during-controller-run', True),)
    return rollout(coordinates)
  with mock.patch.object(value, '_rollout_uncached', side_effect=changing), pytest.raises(ValueError, match='changed owned input'):
    value._rollout((.01,))
  assert value._EventControllerReplay__strict_rollouts is None


def test_native_feasibility_and_final_current_guard_are_not_cached_admission():
  value = engine()
  with mock.patch.object(value, '_rollout_uncached', side_effect=numeric) as original, \
       mock.patch.object(value, '_slew_ok', return_value=True):
    assert value.native_feasible((.01,))
    assert value.native_feasible((.01,))
    assert original.call_count == 2
    # This negative supplies no ReplayValidation: a cached numeric trace cannot
    # override the ordinary current-output limiter veto in final validate.
    profile = SimpleNamespace(policy=POLICY, identity=value.identity, parameters=value.objective.parameters, coefficients=(.01,))
    model = SimpleNamespace(identity=value.identity, domain='normalized_torque', context='unused')
    control = SimpleNamespace(identity=value.identity, baseline_revision=value.baseline_revision)
    with mock.patch('openpilot.starpilot.flm.event_controller_replay.canonical_physical_context', return_value='unused'):
      assert value.validate(profile, control, SimpleNamespace(limited=True), (), (), model) is None
    assert original.call_count == 2


@pytest.mark.parametrize('domain', ('torque', 'pid'))
def test_failed_candidate_skips_baseline_lookup_but_retains_exact_owned_key(domain):
  value = engine(domain)
  with mock.patch.object(value, '_rollout_uncached', side_effect=numeric):
    value._rollout(value.reference)
  slots = value._EventControllerReplay__strict_rollouts[1]
  baseline = slots['baseline']
  with mock.patch.object(value, '_causal_input_key', wraps=value._causal_input_key) as source_key, \
       mock.patch.object(value, '_rollout_uncached', side_effect=ValueError('strict candidate clipping')) as original, \
       mock.patch.object(value, '_slew_ok') as slew:
    assert not value.native_feasible((.01,))
    assert original.call_args_list == [mock.call((.01,))]
    assert source_key.call_args_list == [mock.call(.01)]
    slew.assert_not_called()
  assert slots == {'baseline': baseline}  # No failed candidate was stored.


@pytest.mark.parametrize('domain', ('torque', 'pid'))
def test_successful_candidate_still_requires_original_baseline_and_all_source_keys(domain):
  value = engine(domain)
  with mock.patch.object(value, '_causal_input_key', wraps=value._causal_input_key) as source_key, \
       mock.patch.object(value, '_rollout_uncached',
         side_effect=[rollout((.01,)), ValueError('invalid baseline')]) as original, \
       mock.patch.object(value, '_slew_ok') as slew:
    assert not value.native_feasible((.01,))
    assert original.call_args_list == [mock.call((.01,)), mock.call(value.reference)]
    assert source_key.call_args_list == [mock.call(.01)] * 3
    slew.assert_not_called()
