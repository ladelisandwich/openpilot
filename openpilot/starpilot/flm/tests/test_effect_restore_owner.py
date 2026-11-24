from dataclasses import replace

from openpilot.starpilot.flm.adaptive_profile import ReplayValidation
from openpilot.starpilot.flm.canonical_surface_adapter import NAMES
from openpilot.starpilot.flm.event_objective import EffectObjective
from openpilot.starpilot.flm.full_surface_store import remember_full, load_prior
from openpilot.starpilot.flm.selected_effects import EffectRestoreTrial, EffectOwner, RESTORE_POLICY
from openpilot.starpilot.flm.tests.test_history_restore import history, restore_profile
from openpilot.starpilot.flm.tests.test_full_surface_profile import base, profile
from openpilot.starpilot.flm.torque_adapter import Admission


def full_restore():
  raw, last = history()
  prior = load_prior(raw, binding=last.source_binding())
  # Another genuinely persisted group, beyond the marker group's dimensions.
  updated = base(('ff_gain_right',), prior.envelope).with_proposal({'ff_gain_right': 0.005})
  raw = remember_full(raw, profile(updated), validation_sha256='4' * 64)
  current = replace(base(), source_epoch='8' * 64, context_key='8' * 64).validate()
  history_prior = load_prior(raw, binding=current.source_binding())
  envelope = current.restore_from_history(history_prior)
  p = restore_profile(envelope)
  objective = EffectObjective('ff.left', envelope.trial_names, envelope.trial_anchor, envelope.context_key)
  return envelope, p, objective


def test_explicit_restore_source_runs_full_map_not_only_marker_parameters():
  env, p, objective = full_restore()
  assert len(p.parameters) == 1 and env.trial_names == ('ff_gain_left',)
  trial = EffectRestoreTrial(objective, p.coefficients, env.selected_baseline_revision, env.replay_anchor(), env)
  source = trial.source()
  assert source.values['ff_gain_left'] == 0.020
  assert source.values['ff_gain_right'] == 0.005
  assert tuple((n, source.values[n]) for n in NAMES) == env.coordinates


def test_restore_owner_slews_all_groups_and_exact_renewal_keeps_continuity():
  env, p, objective = full_restore()
  owner = EffectOwner(
    p.identity,
    objective,
    selected_baseline_revision=env.selected_baseline_revision,
    physical_model_revision='physical',
    original_anchor=env.replay_anchor(),
    restore_envelope=env,
  )
  context = {
    'cp_sha256': env.cp_sha256,
    'controller_revision': env.controller_revision,
    'source_epoch': env.source_epoch,
    'reset_generation': env.reset_generation,
    'selected_baseline_revision': env.selected_baseline_revision,
    'context_key': env.context_key,
    'persistent_source_epoch': env.persistent_source_epoch,
    'current_full_profile_sha256': env.prior_full_profile_sha256,
  }
  v = ReplayValidation(p, env.selected_baseline_revision, 'physical', True, True, 0.1)
  owner.prepare(Admission(p, 10, True, False, True, v), active=True, current_context=context)
  assert owner.profile_policy == RESTORE_POLICY and owner.source is not None
  assert owner.applied_full['ff_gain_left'] == owner.applied_full['ff_gain_right'] == 0.005 * 0.01
  first = dict(owner.applied_full)
  owner.prepare(Admission(p, 10_000_010, True, False, True, v), active=True, current_context=context)
  assert all(abs(owner.applied_full[n] - first[n]) <= 0.005 * 0.01 + 1e-12 for n in NAMES)
  assert owner.applied_full['ff_gain_right'] > first['ff_gain_right']
  owner.prepare(Admission(p, 20_000_010, True, True, True, v), active=True, current_context=context)
  assert owner.source is None and not any(owner.applied_full.values())


def test_child_group_off_analysis_preserves_historical_envelope_and_cannot_admit():
  import pytest
  from openpilot.starpilot.flm.canonical_surface_adapter import GROUPS
  from openpilot.starpilot.flm.selected_effects import EffectRestoreAblationTrial, EffectReplayTransition

  env, p, objective = full_restore()
  before = env.coordinates
  analysis = EffectRestoreAblationTrial(objective, p.coefficients, env.selected_baseline_revision, env.replay_anchor(), env, GROUPS['ff.right'])
  source = analysis.source()
  assert source.values['ff_gain_left'] == 0.020
  assert source.values['ff_gain_right'] == 0
  assert env.coordinates == before
  assert dict(env.coordinates)['ff_gain_right'] == 0.005
  with pytest.raises(ValueError):
    EffectReplayTransition(analysis)
