"""Deterministic native selection, no heldout search or authority fabrication."""
from dataclasses import asdict, replace
from unittest import mock

import pytest

from openpilot.starpilot.flm.adaptive_profile import Block, Sample, loss
from openpilot.starpilot.flm.canonical_surface_adapter import parameters_for
from openpilot.starpilot.flm.event_evaluate import evaluate_event, NATIVE_FEASIBILITY_SCALES
from openpilot.starpilot.flm.event_controller_replay import EventControllerReplay
from openpilot.starpilot.flm.event_activation import (
  ZERO_REFERENCE_ACTIVATION_VERSION, EXACT_EMISSION_ACTIVATION_VERSION, activation_from_dict,
  activation_model_from_dict, activation_matches_model, bind_activation_model,
)
from openpilot.starpilot.flm.tests.test_full_surface_profile import base
from openpilot.starpilot.flm.tests.test_event_activation import primitive_plan, fitted_model
from openpilot.starpilot.flm.tests.test_declared_restore import envelope, plan_for


def problem():
  envelope = base()
  identity, cohort, response = 'b' * 64, 'cohort', 'measured-response'
  blocks, episodes = [], []
  for episode in range(9):
    samples = tuple(Sample(1_000_000 * (episode * 40 + row + 1) + 2,
      1_000_000 * (episode * 40 + row + 1), 1_000_000 * (episode * 40 + row + 1) + 1,
      (1.,), .02, 1.) for row in range(40))
    block = Block(identity, str(episode), cohort, 'torque-selected-effect-v2', response, samples)
    blocks.append(block)
    episodes.append({'id': f'{episode:064x}', 'kind': 'subtle', 'start_ns': samples[0].control_ns,
      'end_ns': samples[-1].mono_ns, 'block_ids': (block.block_id,)})
  kwargs = {'episodes': tuple(episodes), 'now_ns': 1_000_000_000, 'required_cohorts': (cohort,), 'response_revision': response,
    'prior': (0.,), 'effect_envelope': envelope}
  return (identity, parameters_for(envelope.trial_names), tuple(blocks)), kwargs


def test_first_native_feasible_scale_selected_before_tiny_heldout_losses_exact_metrics():
  args, kwargs = problem()
  calls = []
  def native(coordinates):
    calls.append(coordinates)
    return len(calls) == 3
  def exact_loss(samples, coordinates):
    assert len(calls) == 3  # All event tiny-heldout losses are AFTER frozen selection.
    return loss(samples, coordinates)
  with mock.patch('openpilot.starpilot.flm.event_evaluate.loss', side_effect=exact_loss):
    result = evaluate_event(*args, **kwargs, native_feasible=native)
  assert NATIVE_FEASIBILITY_SCALES == (1., .5, .25, .125, .0625, .03125, .015625, .0078125)
  assert calls == [(.005,), (.0025,), (.00125,)]
  profile = result.profile
  assert profile is not None and profile.coefficients == calls[-1]
  rows = tuple(row for block in args[2][4:] for row in block.samples)
  assert profile.baseline_loss == loss(rows, (0.,))
  assert profile.candidate_loss == loss(rows, calls[-1])
  assert profile.candidate_loss != loss(rows, calls[0]) * .25  # Never scale fake losses.
  assert dict(profile.effect_envelope.coordinates)['ff_gain_left'] == calls[-1][0]
  assert profile.train_ids == ('0', '1', '2', '3') and profile.heldout_ids == ('4', '5', '6', '7', '8')


def test_all_fixed_native_scales_fail_without_losses_neutral_or_profile():
  args, kwargs = problem()
  native = mock.Mock(return_value=False)
  with mock.patch('openpilot.starpilot.flm.event_evaluate.loss') as tiny_loss:
    result = evaluate_event(*args, **kwargs, native_feasible=native)
  assert result.profile is None and result.reason == 'event_no_native_feasible_scale'
  assert [call.args[0] for call in native.call_args_list] == [(.005 * scale,) for scale in NATIVE_FEASIBILITY_SCALES]
  tiny_loss.assert_not_called()


def test_selected_tiny_heldout_failure_never_falls_back_or_retries():
  args, kwargs = problem()
  native = mock.Mock(side_effect=(False, True, AssertionError('must not retry heldout rejection')))
  with mock.patch('openpilot.starpilot.flm.event_evaluate._lower', return_value=-1.):
    result = evaluate_event(*args, **kwargs, native_feasible=native)
  assert result.profile is None and result.reason == 'event_tiny_trial_heldout_regression_or_noise'
  assert native.call_count == 2


def test_preliminary_gate_failure_cannot_select_direction_and_legacy_has_no_callback():
  args, kwargs = problem()
  native = mock.Mock(return_value=True)
  # Real contradictory heldout data fails the existing preliminary ridge gate.
  blocks = tuple(replace(block, samples=tuple(replace(row, residual_target=-.02) for row in block.samples))
    if index >= 4 else block for index, block in enumerate(args[2]))
  result = evaluate_event(args[0], args[1], blocks, **kwargs, native_feasible=native)
  assert result.profile is None and 'heldout' in result.reason
  native.assert_not_called()
  ordinary = evaluate_event(*args, **kwargs)
  assert ordinary.profile is not None and ordinary.profile.coefficients == (.005,)
  assert evaluate_event(*args, **kwargs, native_feasible=lambda _: 1).reason == 'event_native_feasibility_contract'


def replay():
  plan = replace(primitive_plan(fitted_model()), version=ZERO_REFERENCE_ACTIVATION_VERSION).validate()
  engine = EventControllerReplay.__new__(EventControllerReplay)
  engine.activation_plan, engine.restore_envelope, engine.reference = plan, None, (0.,)
  engine._EventControllerReplay__strict_rollouts = None
  return engine


def test_native_selector_has_no_loss_inputs_and_unknown_source_runs_baseline_each_time():
  engine = replay()
  native = ({1: 0., 2: 0.}, {1: 0., 2: 0.}, {1: False, 2: False})
  seen = []
  def rollout(coordinates):
    seen.append(coordinates)
    return native
  with mock.patch.object(engine, '_rollout', side_effect=rollout), \
       mock.patch.object(engine, '_slew_ok', return_value=True) as slew:
    assert engine.native_feasible((.005,)) and engine.native_feasible((.0025,))
    assert seen == [(.005,), (0.,), (.0025,), (0.,)]
    assert slew.call_count == 4
  # This symbolic shell has no owned native history and cannot gain cache
  # readiness. Actual Torque/PID cache keys and immutable maps are tested in
  # test_event_strict_rollout_cache, using their registered factories.
  assert engine._EventControllerReplay__strict_rollouts is None


@pytest.mark.parametrize('defect', ('clipping', 'saturation', 'request_slew', 'encoded_slew', 'missing'))
def test_native_selector_preserves_every_strict_feasibility_failure(defect):
  engine = replay()
  baseline = ({1: 0.}, {1: 0.}, {1: False})
  candidate = ({1: .01}, {1: .01}, {1: defect == 'saturation'})
  if defect == 'missing':
    candidate = ({1: .01}, {}, {1: False})
  sides = [ValueError('original unclip candidate clipping') if defect == 'clipping' else candidate, baseline]
  with mock.patch.object(engine, '_rollout', side_effect=sides), \
       mock.patch.object(engine, '_slew_ok', side_effect=lambda *_, encoded: not (
         defect == 'request_slew' and not encoded or defect == 'encoded_slew' and encoded)):
    assert not engine.native_feasible((.005,))


def test_new_unpublished_plan_rejects_abandoned_string_restore_and_legacy_no_backtrack():
  engine = replay()
  model = bind_activation_model(fitted_model(), engine.activation_plan)
  assert activation_matches_model(engine.activation_plan, activation_model_from_dict(asdict(model)))
  abandoned = {**asdict(engine.activation_plan), 'version': 'fixed-eight-shadow-zero-reference-v5'}
  with pytest.raises(ValueError):
    activation_from_dict(abandoned)
  for plan in (replace(engine.activation_plan, version=EXACT_EMISSION_ACTIVATION_VERSION), plan_for(envelope())):
    engine.activation_plan = plan
    with pytest.raises(ValueError, match='declared trial plan'):
      engine.native_feasible((.005,))
