"""Historical values require a fresh explicit full-map restoration transaction."""

from dataclasses import replace
from types import SimpleNamespace
import pytest
from openpilot.starpilot.flm.full_surface_profile import FullSurfaceEnvelope, RESTORE_POLICY, neutral_coordinates
from openpilot.starpilot.flm.full_surface_store import remember_full, load_prior
from openpilot.starpilot.flm.tests.test_full_surface_profile import base, profile, wire_profile


def history():
  raw = None
  proposed = None
  for value in (0.005, 0.010, 0.015, 0.020):
    current = base() if raw is None else load_prior(raw, binding=proposed.source_binding()).envelope
    proposed = current.for_trial(('ff_gain_left',)).with_proposal({'ff_gain_left': value})
    raw = remember_full(raw, profile(proposed), validation_sha256='3' * 64)
  return raw, proposed


def restore_transaction():
  raw, stored = history()
  current = replace(base(), source_epoch='8' * 64, context_key='8' * 64).validate()
  prior = load_prior(raw, binding=current.source_binding())
  assert prior.historical_only and dict(prior.envelope.coordinates)['ff_gain_left'] == 0.020
  restore = current.restore_from_history(prior)
  return raw, current, prior, restore


def restore_profile(envelope):
  p, _ = wire_profile(envelope)
  return replace(p, policy=RESTORE_POLICY, effect_envelope=envelope)


def test_history_is_not_current_prior_and_restore_is_explicit_full_map_operation():
  raw, current, prior, restore = restore_transaction()
  with pytest.raises(ValueError):
    FullSurfaceEnvelope.initial(
      cp_sha256=current.cp_sha256,
      controller_revision=current.controller_revision,
      source_epoch=current.source_epoch,
      reset_generation=current.reset_generation,
      selected_baseline_revision=current.selected_baseline_revision,
      trial_names=current.trial_names,
      persistent_source_epoch=current.persistent_source_epoch,
      prior=prior,
    )
  assert restore.version == 2 and restore.operation == 'restore'
  assert restore.prior_coordinates == neutral_coordinates()
  assert restore.coordinates == prior.envelope.coordinates
  assert restore.history_full_profile_sha256 == prior.envelope.full_profile_sha256
  assert restore.restoration_groups
  assert restore.replay_anchor()['restore_target'] == restore.coordinates
  assert restore.replay_anchor()['version'] == 2
  assert current.replay_anchor()['version'] == 1 and 'operation' not in current.replay_anchor()
  assert not hasattr(prior, 'profile') and not hasattr(prior, 'expires_ns')


def test_restore_policy_cannot_masquerade_as_tiny_trial():
  _, current, prior, restore = restore_transaction()
  assert dict(restore.coordinates)['ff_gain_left'] == 0.020  # More than .005 trialbudget.
  with pytest.raises(ValueError):
    restore.validate(profile(restore))
  with pytest.raises(ValueError):
    replace(restore, operation='trial', version=1, history_full_profile_sha256='', restore_transaction_sha256='').validate()
  with pytest.raises(ValueError):
    restore.with_proposal({'ff_gain_left': 0.005})
  restore.validate(restore_profile(restore))


@pytest.mark.parametrize(
  'field,value',
  [
    ('history_full_profile_sha256', '7' * 64),
    ('restore_transaction_sha256', '7' * 64),
    ('source_epoch', '7' * 64),
    ('persistent_source_epoch', '7' * 64),
    ('selected_baseline_revision', '7' * 64),
  ],
)
def test_restore_identity_or_target_tampering_fails(field, value):
  _, _, _, restore = restore_transaction()
  with pytest.raises(ValueError):
    replace(restore, **{field: value}).validate()


def test_restoration_persist_transaction_cas_matches_stable_accepted_history_only():
  raw, current, prior, restore = restore_transaction()
  p = restore_profile(restore)
  renewed = remember_full(raw, p, validation_sha256='4' * 64)
  loaded = load_prior(renewed, binding=current.source_binding())
  assert loaded.historical_only and loaded.envelope.coordinates == restore.coordinates
  assert loaded.envelope.operation == 'trial'  # Disk never restoresliveoperation.
  # Empty/different history cannot accept a forged restoration claim.
  with pytest.raises(ValueError):
    remember_full(None, p, validation_sha256='4' * 64)


def test_restore_current_probe_context_still_requires_neutral_cas_and_fresh_source():
  _, current, _, restore = restore_transaction()
  args = {
    'cp_sha256': current.cp_sha256,
    'controller_revision': current.controller_revision,
    'source_epoch': current.source_epoch,
    'reset_generation': current.reset_generation,
    'selected_baseline_revision': current.selected_baseline_revision,
    'context_key': current.context_key,
    'current_full_profile_sha256': current.prior_full_profile_sha256,
    'persistent_source_epoch': current.persistent_source_epoch,
  }
  assert restore.admit_context(**args) == restore
  with pytest.raises(ValueError):
    restore.admit_context(**(args | {'current_full_profile_sha256': '7' * 64}))
  with pytest.raises(ValueError):
    restore.admit_context(**(args | {'source_epoch': '7' * 64}))


def test_restore_wire_explicit_operation_policy_and_transaction_roundtrip():
  from openpilot.starpilot.flm.adaptive_profile import Result, ReplayValidation
  from openpilot.starpilot.flm.learning_wire import result_message, read_runtime_admission

  _, _, _, restore = restore_transaction()
  p = restore_profile(restore)
  validation = ReplayValidation(p, 'a' * 64, 'b' * 64, True, True, 0.1)
  event = result_message(Result('qualified', 'fresh_full_history_replay', p), 'new-drive', 1, 10, replay_validation=validation)
  assert event.autoSteeringLearning.learning.runtimeProfile.fullSurface.version == 2
  assert event.autoSteeringLearning.learning.runtimeProfile.fullSurface.operation == 'restore'
  parsed = read_runtime_admission(event)
  assert parsed is not None and parsed[0].effect_envelope == restore
  event.autoSteeringLearning.learning.runtimeProfile.fullSurface.restoreTransactionSha256 = '7' * 64
  assert read_runtime_admission(event) is None


def test_disk_restore_requires_current_replay_validation_before_params_save():
  from openpilot.starpilot.flm.learned_lifecycle import StorageOwner
  from unittest.mock import patch

  _, _, _, restore = restore_transaction()
  p = restore_profile(restore)
  with patch('openpilot.starpilot.saved_document.commit_exact') as commit:
    assert not StorageOwner(SimpleNamespace()).save(
      p, reset=restore.reset_generation, replay_validation=None, authorized=lambda: True, full_binding=restore.source_binding()
    )
  commit.assert_not_called()
