"""Actual effect owner lifecycle; profile fixture is not physical fit proof."""

from dataclasses import replace

import pytest

from openpilot.starpilot.flm.adaptive_profile import Profile, ReplayValidation
from openpilot.starpilot.flm.canonical_surface_adapter import (
  MANIFEST_SHA256,
  NAMES,
  full_original_anchor,
  full_profile_sha256,
  original_anchor,
  parameters_for,
)
from openpilot.starpilot.flm.event_objective import EffectObjective
from openpilot.starpilot.flm.full_surface_profile import FullSurfaceEnvelope
from openpilot.starpilot.flm.selected_effects import EffectOwner, EffectTrial
from openpilot.starpilot.flm.torque_adapter import Admission

NOW = 1_000_000_000
BASE = 'b' * 64
SOURCE = 'e' * 64


def fixture(*, accumulated=False):
  names = ('ff_gain_left',)
  prior = dict.fromkeys(NAMES, 0.0)
  if accumulated:
    prior['center_taper_max'] = 0.1
  proposed = dict(prior, ff_gain_left=0.004)
  envelope = FullSurfaceEnvelope(
    1,
    MANIFEST_SHA256,
    full_original_anchor(BASE),
    original_anchor(BASE, names),
    SOURCE,
    'c' * 64,
    'selected-controller',
    SOURCE,
    '0',
    BASE,
    full_profile_sha256(BASE, prior),
    full_profile_sha256(BASE, proposed),
    tuple(prior.items()),
    tuple(proposed.items()),
    names,
    'f' * 64,
  )
  profile = Profile(
    'a' * 64,
    1,
    NOW,
    NOW + 500_000_000,
    parameters_for(names),
    (0.004,),
    tuple(f'train{i}' for i in range(4)),
    tuple(f'held{i}' for i in range(4)),
    1.0,
    0.8,
    (('held', 0.1),),
    policy='event_effect_coordinate_v2',
    effect_envelope=envelope,
  )
  validation = ReplayValidation(profile, BASE, 'physical-model', True, True, 0.1)
  objective = EffectObjective('ff.left', names, envelope.trial_anchor, SOURCE)
  context = {
    'cp_sha256': envelope.cp_sha256,
    'controller_revision': envelope.controller_revision,
    'source_epoch': SOURCE,
    'reset_generation': '0',
    'selected_baseline_revision': BASE,
    'context_key': SOURCE,
    'persistent_source_epoch': 'f' * 64,
    'current_full_profile_sha256': envelope.prior_full_profile_sha256,
  }
  owner = EffectOwner(
    profile.identity, objective, selected_baseline_revision=BASE, physical_model_revision='physical-model', original_anchor=envelope.replay_anchor()
  )
  return owner, profile, validation, context


def test_expired_proof_withdraws_all30_not_just_fitted_group():
  owner, p, v, context = fixture(accumulated=True)
  owner.prepare(Admission(p, NOW, True, False, True, v), active=True, current_context=context)
  assert owner.source is not None
  assert 0 < owner.applied_full['center_taper_max'] <= 0.005 * 0.01
  owner.prepare(Admission(p, NOW + 500_000_001, True, False, True, v), active=True, current_context=context)
  assert owner.source is None
  assert all(value == 0 for value in owner.applied_full.values())


@pytest.mark.parametrize('reason', ['manual', 'off', 'inactive', 'unhealthy'])
def test_withdrawal_and_recovery_slew_all30(reason):
  owner, p, v, context = fixture(accumulated=True)
  owner.prepare(Admission(p, NOW, True, False, True, v), active=True, current_context=context)
  owner.prepare(
    Admission(p, NOW + 10_000_000, reason != 'off', reason == 'manual', reason != 'unhealthy', v), active=reason != 'inactive', current_context=context
  )
  assert owner.source is None and not any(owner.applied_full.values())
  owner.prepare(Admission(p, NOW + 20_000_000, True, False, True, v), active=True, current_context=context)
  assert owner.source is not None
  assert max(abs(value) for value in owner.applied_full.values()) <= 0.005 * 0.01 + 1e-12


def test_changed_current_context_never_leaves_previous_source_attached():
  owner, p, v, context = fixture()
  owner.prepare(Admission(p, NOW, True, False, True, v), active=True, current_context=context)
  with pytest.raises(ValueError):
    owner.prepare(Admission(p, NOW + 10_000_000, True, False, True, v), active=True, current_context=dict(context, source_epoch='d' * 64))
  assert owner.source is None


def test_replay_trial_preserves_nonzero_group_and_outside_group_original_prior():
  owner, p, _, _ = fixture(accumulated=True)
  prior = dict(p.effect_envelope.prior_coordinates)
  prior['ff_gain_left'] = 0.1
  envelope = replace(
    p.effect_envelope,
    prior_coordinates=tuple(prior.items()),
    coordinates=tuple(prior.items()),
    prior_full_profile_sha256=full_profile_sha256(BASE, prior),
    full_profile_sha256=full_profile_sha256(BASE, prior),
  )
  trial = EffectTrial(owner.objective, (0.104,), BASE, envelope.replay_anchor())
  source = trial.source()
  assert source.values['ff_gain_left'] == 0.104
  assert source.values['center_taper_max'] == 0.1
  assert source.values['ff_gain_right'] == 0


def test_trial_group_switch_preserves_full30_state_and_cas_without_step_jump():
  owner, p, v, context = fixture()
  owner.prepare(Admission(p, NOW, True, False, True, v), active=True, current_context=context)
  before = dict(owner.applied_full)
  old = p.effect_envelope
  names = ('turn_in_threshold_reduction_left',)
  prior = dict(old.coordinates)
  proposed = dict(prior, turn_in_threshold_reduction_left=0.004)
  envelope = replace(
    old,
    trial_names=names,
    trial_anchor=original_anchor(BASE, names),
    prior_coordinates=tuple(prior.items()),
    coordinates=tuple(proposed.items()),
    prior_full_profile_sha256=old.full_profile_sha256,
    full_profile_sha256=full_profile_sha256(BASE, proposed),
  )
  p2 = replace(p, sequence=2, parameters=parameters_for(names), coefficients=(0.004,), effect_envelope=envelope)
  v2 = replace(v, profile=p2)
  objective = EffectObjective('friction.left', names, envelope.trial_anchor, SOURCE)
  next_owner = EffectOwner(
    p.identity,
    objective,
    selected_baseline_revision=BASE,
    physical_model_revision='physical-model',
    original_anchor=envelope.replay_anchor(),
    applied_full=before,
  )
  context2 = dict(context, current_full_profile_sha256=old.full_profile_sha256)
  next_owner.prepare(Admission(p2, NOW + 10_000_000, True, False, True, v2), active=True, current_context=context2)
  assert next_owner.trace.before == tuple(before.items())
  assert next_owner.applied_full['ff_gain_left'] >= before['ff_gain_left']
  assert 0 < next_owner.applied_full['turn_in_threshold_reduction_left'] <= 0.005 * 0.01
  assert all(abs(next_owner.applied_full[n] - before[n]) <= 0.005 * 0.01 + 1e-12 for n in NAMES)
  stale = EffectOwner(
    p.identity, objective, selected_baseline_revision=BASE, physical_model_revision='physical-model', original_anchor=envelope.replay_anchor()
  )
  with pytest.raises(ValueError):
    stale.prepare(Admission(p2, NOW + 10_000_000, True, False, True, v2), active=True, current_context=context)
  assert stale.source is None
