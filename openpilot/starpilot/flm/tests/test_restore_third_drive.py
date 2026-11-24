"""Current production restoration persists and reloads only historical values."""

from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import patch
import pytest
from openpilot.starpilot.flm.full_surface_store import load_prior, remember_full, rollback_full
from openpilot.starpilot.flm.learned_lifecycle import StorageOwner
from openpilot.starpilot.flm.full_surface_profile import neutral_coordinates
from openpilot.starpilot.flm.tests.test_full_surface_profile import base, profile
from openpilot.starpilot.flm.tests.test_history_restore import restore_transaction, restore_profile


def test_restore_save_then_third_drive_load_remains_historical_only():
  raw, _, prior2, restore2 = restore_transaction()
  saved = remember_full(raw, restore_profile(restore2), validation_sha256='4' * 64)
  drive3 = replace(base(), source_epoch='7' * 64, context_key='7' * 64).validate()
  prior3 = load_prior(saved, binding=drive3.source_binding())
  assert prior3 is not None and prior3.historical_only
  assert prior3.envelope.operation == 'trial'
  assert prior3.envelope.source_epoch == drive3.source_epoch
  assert prior3.envelope.coordinates == prior2.envelope.coordinates
  restore3 = drive3.restore_from_history(prior3)
  assert restore3.operation == 'restore' and restore3.prior_coordinates == neutral_coordinates()
  assert restore3.coordinates == restore2.coordinates
  assert restore3.restore_transaction_sha256 != restore2.restore_transaction_sha256
  assert not hasattr(prior3, 'profile') and not hasattr(prior3, 'expires_ns')


def test_first_new_trial_after_actual_restoration_uses_stored_target_cas():
  raw, current, _, restore = restore_transaction()
  saved = remember_full(raw, restore_profile(restore), validation_sha256='4' * 64)
  # This transition happens ONLY after the actual controller accepted/restored
  # target is observed in its live trace; disk loading alone cannot create it.
  actual = replace(restore, prior_coordinates=restore.coordinates, prior_full_profile_sha256=restore.full_profile_sha256).for_trial(('ff_gain_left',))
  proposal = actual.with_proposal({'ff_gain_left': 0.025})
  changed = remember_full(saved, profile(proposal), validation_sha256='5' * 64)
  assert dict(load_prior(changed, binding=current.source_binding()).envelope.coordinates)['ff_gain_left'] == 0.025
  rolled = rollback_full(changed, binding=current.source_binding(), expected_full_hash=proposal.full_profile_sha256)
  assert load_prior(rolled, binding=current.source_binding()).envelope.coordinates == restore.coordinates


@pytest.mark.parametrize(
  'change', [{'persistent_source_epoch': '6' * 64}, {'reset_generation': '1'}, {'cp_sha256': '6' * 64}, {'controller_revision': 'other-controller'}]
)
def test_third_drive_wrong_vehicle_or_config_or_reset_never_loads_history(change):
  raw, current, _, restore = restore_transaction()
  saved = remember_full(raw, restore_profile(restore), validation_sha256='4' * 64)
  third = replace(base(), source_epoch='7' * 64, context_key='7' * 64, **change)
  assert load_prior(saved, binding=third.source_binding()) is None


def test_disk_identity_does_not_bypass_fresh_live_replay_or_params_source_cas():
  _, _, _, restore = restore_transaction()
  p = restore_profile(restore)
  with patch('openpilot.starpilot.saved_document.commit_exact') as commit:
    assert not StorageOwner(SimpleNamespace()).save(
      p, reset=restore.reset_generation, replay_validation=None, authorized=lambda: True, full_binding=restore.source_binding()
    )
  commit.assert_not_called()


def test_actual_storage_owner_commit_and_third_drive_prior_without_live_permission():
  from openpilot.common.params import Params
  from openpilot.common.prefix import OpenpilotPrefix
  from openpilot.starpilot.flm.learned_lifecycle import KEY
  from openpilot.starpilot.flm.adaptive_profile import ReplayValidation

  with OpenpilotPrefix():
    params = Params()
    raw, current, _, restore = restore_transaction()
    params.put(KEY, raw, block=True)
    p = restore_profile(restore)
    validation = ReplayValidation(p, restore.selected_baseline_revision, 'b' * 64, True, True, 0.1)
    authorized = []

    def current_authority():
      authorized.append(True)
      return True

    assert StorageOwner(params).save(
      p, reset=restore.reset_generation, replay_validation=validation, authorized=current_authority, full_binding=restore.source_binding()
    )
    assert len(authorized) >= 2  # Writer checks authority again under Params lock.
    third = replace(base(), source_epoch='7' * 64, context_key='7' * 64).validate()
    loaded = load_prior(params.get(KEY), binding=third.source_binding())
    assert loaded.historical_only and loaded.envelope.coordinates == restore.coordinates
    assert loaded.envelope.source_epoch == third.source_epoch
    assert loaded.envelope.operation == 'trial' and not hasattr(loaded, 'profile')
    old = params.get(KEY)
    assert not StorageOwner(params).save(
      p, reset=restore.reset_generation, replay_validation=validation, authorized=lambda: False, full_binding=restore.source_binding()
    )
    assert params.get(KEY) == old
