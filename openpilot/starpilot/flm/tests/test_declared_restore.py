"""Strict restore declaration/dispatch units; no fabricated replay qualification."""
from dataclasses import asdict, replace
from types import SimpleNamespace
from unittest import mock
import base64

import pytest

from openpilot.starpilot.flm.canonical_surface_adapter import GROUPS
from openpilot.starpilot.flm.clipping_quality import (
  FIXED_MAP_IDENTIFICATION, NATIVE_SHADOW_IDENTIFICATION, QUALIFIED_EVENT_CLIPPING_QUALITY,
  RESTORE_ACTIVATION_VERSION, event_clipping_quality,
)
from openpilot.starpilot.flm.controller_replay import pack_primitive_snapshot
from openpilot.starpilot.flm.event_activation import activation_from_dict, activation_matches_model, activation_model_from_dict, bind_activation_model
from openpilot.starpilot.flm.event_controller_replay import EventControllerReplay
from openpilot.starpilot.flm.event_fit_factory import fit_event_snapshot
from openpilot.starpilot.flm.event_objective import EffectObjective
from openpilot.starpilot.flm.event_restore_fit import _fixed_restore_rollout
from openpilot.starpilot.flm.learner_factory import fit_snapshot
from openpilot.starpilot.flm.restore_activation import (
  WholeMapRestorePlan, restore_envelope_digest, restore_plan_from_dict, restore_plan_matches,
)
from openpilot.starpilot.flm.selected_effects import EffectRestoreTrial, EffectRestoreAblationTrial
from openpilot.starpilot.flm.tests.test_event_activation import fitted_model, primitive_plan
from openpilot.starpilot.flm.tests.test_history_parent import parent, saved_prior
from openpilot.starpilot.flm.torque_adapter import Objective


def envelope():
  owner = parent()
  owner.install_full_prior(saved_prior(owner.envelope))
  return owner.restore_envelope


def payload():
  restored = envelope()
  objective = EffectObjective('ff.left', restored.trial_names, restored.trial_anchor, restored.context_key)
  return {'version': 1, 'family': 'torque', 'event_version': 4, 'event_operation': 'restore',
    'event_restore_policy': RESTORE_ACTIVATION_VERSION, 'event_identification_policy': FIXED_MAP_IDENTIFICATION,
    'event_clipping_quality': QUALIFIED_EVENT_CLIPPING_QUALITY, 'effect_envelope': asdict(restored),
    'event_objective': asdict(objective), 'objective': asdict(Objective('feedforward', 'high', 'steady'))}


def plan_for(restored):
  model = fitted_model()
  base = primitive_plan(model)
  return WholeMapRestorePlan(
    version=RESTORE_ACTIVATION_VERSION, identity=base.identity,
    baseline_revision=restored.selected_baseline_revision, source_epoch=restored.source_epoch, physical_context=base.physical_context,
    calibration_train_blocks=base.calibration_train_blocks, calibration_heldout_blocks=base.calibration_heldout_blocks,
    cutoff_ns=2_005_000_000, stage_blocks=base.stage_blocks,
    windows=((2_000_000_000, 2_020_000_000, 2_010_000_000, 2_010_000_000),),
    restore_envelope_sha256=restore_envelope_digest(restored)).validate()


def test_restore_only_payload_has_no_shadow_or_operation_fallback():
  original = payload()
  assert event_clipping_quality(original) == QUALIFIED_EVENT_CLIPPING_QUALITY
  for change in ({'event_operation': 'trial'}, {'event_version': 3}, {'event_restore_policy': 'unknown'},
                 {'event_identification_policy': NATIVE_SHADOW_IDENTIFICATION}, {'event_clipping_quality': 'recorded-debt-v1'}):
    with pytest.raises(ValueError):
      event_clipping_quality({**original, **change})
  for key in ('event_restore_policy', 'event_identification_policy', 'event_clipping_quality'):
    with pytest.raises(ValueError):
      event_clipping_quality({name: value for name, value in original.items() if name != key})


def test_restore_plan_model_full_envelope_and_strict_decode_binding():
  restored = envelope()
  plan = plan_for(restored)
  model = bind_activation_model(fitted_model(), plan)
  assert plan.identification_policy == FIXED_MAP_IDENTIFICATION
  assert restore_plan_matches(plan, restored)
  assert restore_plan_from_dict(asdict(plan)) == plan
  assert activation_matches_model(plan, activation_model_from_dict(asdict(model)))
  with pytest.raises(ValueError):
    activation_from_dict(asdict(plan))
  for change in ({'version': 'unknown'}, {'restore_envelope_sha256': ''}, {'extra': 1}):
    with pytest.raises((ValueError, TypeError)):
      restore_plan_from_dict({**asdict(plan), **change})
  assert not activation_matches_model(primitive_plan(fitted_model()), model)
  with pytest.raises(ValueError):
    restore_envelope_digest(replace(restored, restore_transaction_sha256='0' * 64))
  assert not restore_plan_matches(replace(plan, restore_envelope_sha256='1' * 64), restored)


def test_actual_child_and_outer_factory_dispatch_restore_v4_without_fd():
  row = payload()
  restored = envelope()
  replay = {'baseline_revision': restored.selected_baseline_revision}
  expected = {'status': 'collecting', 'reason': 'fixed-eight-not-supported'}
  factory = SimpleNamespace(cp=SimpleNamespace(lateralTuning=SimpleNamespace(which=lambda: 'torque')))
  row['replay'] = pack_primitive_snapshot(replay)
  wrapped = {'version': 1, 'family': 'torque', 'snapshot_b64': base64.b64encode(pack_primitive_snapshot(row)).decode()}
  with mock.patch('openpilot.starpilot.flm.learner_factory.factory_from_snapshot', return_value=(factory, None)), \
       mock.patch('openpilot.starpilot.flm.evidence_noise.reject_noisy_job', return_value=None), \
       mock.patch('openpilot.starpilot.flm.event_fit_factory.ControllerReplay.from_snapshot'), \
       mock.patch('openpilot.starpilot.flm.event_fit_factory.EventControllerReplay') as engine, \
       mock.patch('openpilot.starpilot.flm.event_restore_fit.prepare_fixed_restore_calibration', return_value=expected) as prepare, \
       mock.patch('openpilot.starpilot.flm.event_restore_fit.fit_event_restore') as fit:
    assert fit_snapshot(wrapped) == expected
    assert prepare.call_count == 1 and fit.call_count == 0
    engine.return_value.rebuild_causal_features.assert_not_called()
  with pytest.raises(ValueError):
    fit_event_snapshot({**row, 'event_operation': 'trial'}, factory, replay)


def replay_fixture():
  restored = envelope()
  objective = EffectObjective('ff.left', restored.trial_names, restored.trial_anchor, restored.context_key)
  plan = plan_for(restored)
  frames = tuple(SimpleNamespace(control_ns=2_000_000_000 + index * 10_000_000, frame_sequence=index + 1,
    effect_prepare_ns=2_000_000_000 + index * 10_000_000, effect_trace=None,
    applied_coordinates=(0.,), centre_values=(0., 0.), actual_stage_residual=None,
    selected_command=0., actual_lac_command=0., inputs=(), torque_values=()) for index in range(3))
  emissions = SimpleNamespace(rollout=mock.Mock(return_value=dict.fromkeys(range(3), 0.)))
  original = SimpleNamespace(failed=False, raw_mode=True, identity=plan.identity,
    baseline_revision=plan.baseline_revision, replay_windows=((2_000_000_000, 2_020_000_000),), records=frames,
    factory=SimpleNamespace(create=lambda: (object(), object())), checkpoints=((2_000_000_000, object()),),
    objective=Objective('feedforward', 'high', 'steady'), emissions=emissions, _frame_context=lambda frame: True)
  engine = EventControllerReplay(original, objective, restored.replay_anchor(), restore_envelope=restored, activation_plan=plan)
  return engine, restored, frames, emissions


def test_fullmap_and_groupoff_share_exact_neutral_prefix_and_strict_policy():
  engine, restored, frames, emissions = replay_fixture()
  coordinates = tuple(dict(restored.coordinates)[name] for name in engine.objective.names)
  for omitted in (None, GROUPS['ff.left']):
    seen = []
    def run(*args, effect_trial=None, seen_sources=seen, **kwargs):
      seen_sources.append(effect_trial)
      return 0., SimpleNamespace(saturated=False)
    with mock.patch('openpilot.starpilot.flm.event_controller_replay._run', side_effect=run), \
         mock.patch('openpilot.starpilot.flm.event_controller_replay.restore_snapshot'):
      engine._rollout(coordinates, restore_target=True, restore_omit_group=omitted)
    assert seen[0] is None
    expected = EffectRestoreTrial if omitted is None else EffectRestoreAblationTrial
    assert all(type(source) is expected for source in seen[1:])
    assert emissions.rollout.call_args.kwargs == {'quality_contract': QUALIFIED_EVENT_CLIPPING_QUALITY}
  with pytest.raises(ValueError, match='restoration requires'):
    engine.rebuild_causal_features()
  with pytest.raises(ValueError, match='scope/policy'):
    engine._rollout(coordinates, _identification_token=engine._EventControllerReplay__identification_token)
  with pytest.raises(ValueError, match='active exact canonical group'):
    engine._rollout(coordinates, restore_target=True, restore_omit_group=GROUPS['center'])
  with pytest.raises(ValueError, match='operation/transaction'):
    EventControllerReplay(engine.engine, engine.objective, restored.replay_anchor(),
      restore_envelope=restored, activation_plan=primitive_plan(fitted_model()))
  frames[1].effect_prepare_ns -= 1
  with mock.patch('openpilot.starpilot.flm.event_controller_replay._run', return_value=(0., SimpleNamespace(saturated=False))), \
       mock.patch('openpilot.starpilot.flm.event_controller_replay.restore_snapshot'), \
       pytest.raises(ValueError, match='prepare clock changed'):
    engine._rollout(coordinates, restore_target=True)


def test_strict_native_refusal_is_truthful_v4_report_and_v2_still_raises():
  engine, _, _, _ = replay_fixture()
  with mock.patch.object(engine, '_rollout', side_effect=ValueError('candidate encoder clipping regression')):
    report = _fixed_restore_rollout(payload(), engine, (0.,), restore_target=True)
    assert report['profile'] is report['validation'] is None
    assert report['replay_phase'] == 'full-map' and 'clipping regression' in report['replay_reason']
    with pytest.raises(ValueError, match='clipping regression'):
      _fixed_restore_rollout({'event_version': 2}, engine, (0.,), restore_target=True)


def test_plan_binds_nonmarker_fullmap_source_reset_cp_and_signed_zero():
  from openpilot.starpilot.flm.canonical_surface_adapter import full_profile_sha256
  from openpilot.starpilot.flm.full_surface_profile import restore_transaction
  restored = envelope()
  plan = plan_for(restored)
  changes = ({'cp_sha256': '8' * 64}, {'reset_generation': '1'},
             {'source_epoch': '7' * 64, 'context_key': '7' * 64})
  for change in changes:
    changed = replace(restored, **change)
    changed = replace(changed, restore_transaction_sha256=restore_transaction(changed)).validate()
    assert not restore_plan_matches(plan, changed)
  for value in (.005, -0.):
    coordinates = tuple((name, value if name == 'ff_gain_right' else old) for name, old in restored.coordinates)
    digest = full_profile_sha256(restored.selected_baseline_revision, coordinates)
    changed = replace(restored, coordinates=coordinates, full_profile_sha256=digest, history_full_profile_sha256=digest)
    changed = replace(changed, restore_transaction_sha256=restore_transaction(changed)).validate()
    assert restore_envelope_digest(changed) != restore_envelope_digest(restored)
    assert not restore_plan_matches(plan, changed)


@pytest.mark.parametrize('configured,reported', ((4, 2), (2, 4)))
def test_actual_parent_rejects_restore_report_from_other_configured_version(configured, reported):
  from openpilot.starpilot.flm.full_surface_profile import RESTORE_POLICY
  owner = parent()
  owner.install_full_prior(saved_prior(owner.envelope))
  restored = owner.restore_envelope
  objective = EffectObjective('ff.left', restored.trial_names, restored.trial_anchor, restored.context_key)
  # Negative contract test only: no ReplayValidation or native permission flags.
  profile = SimpleNamespace(effect_envelope=restored, policy=RESTORE_POLICY,
    parameters=objective.parameters, coefficients=tuple(dict(restored.coordinates)[name] for name in objective.names))
  owner.key = ('event', 'ff.left')
  owner.replay_version, owner.restore_version = 3, configured
  report = {'event_version': reported, 'model': asdict(fitted_model()),
    'restoration_qualified_groups': restored.restoration_groups}
  assert not owner.install_fit(report, SimpleNamespace(profile=profile), SimpleNamespace(profile=profile), physical_context=())


def test_fixed_first_eight_physical_failure_cannot_search_a_later_cutoff():
  from openpilot.starpilot.flm.event_restore_fit import prepare_fixed_restore_calibration
  restored = envelope()
  events = tuple({'block_id': str(index), 'samples': ({'control_ns': index * 1_000_000_000,
    'apply_ns': index * 1_000_000_000 + 1, 'mono_ns': index * 1_000_000_000 + 2,
    'output': {'stamp': index * 1_000_000_000 + 1}, 'physical': {'lateral_accel': .02}},)} for index in range(1, 17))
  engine = SimpleNamespace(identity='a' * 64, records=tuple(SimpleNamespace(control_ns=index) for index in range(1, 17)))
  row = {**payload(), 'physical_context': ('cal', 'vm', 'lag', 0)}
  with mock.patch('openpilot.starpilot.flm.event_restore_fit._restore_original_evidence',
                  return_value=(restored, 'b' * 64, (), events)), \
       mock.patch('openpilot.starpilot.flm.event_restore_fit.output_from_dict',
                  side_effect=lambda values: SimpleNamespace(apply_ns=values['stamp'])), \
       mock.patch('openpilot.starpilot.flm.event_restore_fit.delivered', return_value=.01), \
       mock.patch('openpilot.starpilot.flm.event_restore_fit.fit_event_physical', return_value=None) as measured:
    report = prepare_fixed_restore_calibration(row, engine, None)
  assert report['reason'] == 'restore_fixed_eight_independent_physical_model'
  assert report['profile'] is report['validation'] is None
  measured.assert_called_once()
  assert tuple(point.block_id for point in measured.call_args.args[2]) == tuple(str(index) for index in range(1, 9))
  assert measured.call_args.kwargs['now_ns'] == 8_000_000_002
