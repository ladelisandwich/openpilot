from dataclasses import replace
from types import SimpleNamespace
import json
import pytest
from openpilot.starpilot.flm.canonical_surface_adapter import NAMES, MANIFEST_SHA256, original_anchor, full_original_anchor, full_profile_sha256, parameters_for
from openpilot.starpilot.flm.full_surface_profile import FullSurfaceEnvelope, neutral_coordinates, POLICY
from openpilot.starpilot.flm.full_surface_store import remember_full, load_prior, rollback_full
from openpilot.starpilot.flm.learned_lifecycle import remember, decode


def base(names=('ff_gain_left',), prior=None):
  rows = neutral_coordinates() if prior is None else prior.coordinates
  digest = full_profile_sha256('a' * 64, rows)
  return FullSurfaceEnvelope(
    1,
    MANIFEST_SHA256,
    full_original_anchor('a' * 64),
    original_anchor('a' * 64, names),
    'e' * 64,
    'c' * 64,
    'selected-controller-v2:' + ('d' * 64),
    'e' * 64,
    '0',
    'a' * 64,
    digest,
    digest,
    rows,
    rows,
    names,
    '9' * 64,
  ).validate()


def profile(envelope):
  return SimpleNamespace(
    identity='f' * 64,
    policy=POLICY,
    parameters=parameters_for(envelope.trial_names),
    coefficients=tuple(dict(envelope.coordinates)[n] for n in envelope.trial_names),
    effect_envelope=envelope,
  )


def test_legacy_document_unchanged_when_full_namespace_added():
  legacy = SimpleNamespace(
    identity='1' * 64, policy='passive_stage_ridge_v1', parameters=(SimpleNamespace(name='gain', stage_revision='stage'),), coefficients=(0.01,)
  )
  raw = remember(None, legacy, reset='0', validation_sha256='2' * 64)
  before = decode(raw)['profiles']
  proposal = base().with_proposal({'ff_gain_left': 0.005})
  upgraded = remember_full(raw, profile(proposal), validation_sha256='3' * 64)
  assert decode(upgraded)['version'] == 2
  assert decode(upgraded)['profiles'] == before
  restored = load_prior(upgraded, binding=proposal.source_binding())
  assert restored.historical_only and restored.envelope.coordinates == proposal.coordinates
  assert not hasattr(restored, 'expires_ns') and not hasattr(restored, 'profile')


def test_different_group_updates_merge_all30_and_retain_rollback():
  first = base().with_proposal({'ff_gain_left': 0.005})
  raw = remember_full(None, profile(first), validation_sha256='3' * 64)
  second = base(('center_taper_max',), first).with_proposal({'center_taper_max': 0.005})
  updated = remember_full(raw, profile(second), validation_sha256='4' * 64)
  prior = load_prior(updated, binding=second.source_binding())
  assert dict(prior.envelope.coordinates)['ff_gain_left'] == 0.005
  assert dict(prior.envelope.coordinates)['center_taper_max'] == 0.005
  assert len(prior.envelope.coordinates) == 30
  rolled = rollback_full(updated, binding=second.source_binding(), expected_full_hash=second.full_profile_sha256)
  assert load_prior(rolled, binding=first.source_binding()).envelope.coordinates == first.coordinates


def test_stale_prior_hash_and_outside_group_changes_rejected():
  first = base().with_proposal({'ff_gain_left': 0.005})
  raw = remember_full(None, profile(first), validation_sha256='3' * 64)
  with pytest.raises(ValueError):
    remember_full(raw, profile(base().with_proposal({'ff_gain_left': 0.004})), validation_sha256='4' * 64)
  changed = dict(first.coordinates)
  changed['center_taper_max'] = 0.005
  with pytest.raises(ValueError):
    replace(first, coordinates=tuple((n, changed[n]) for n in NAMES), full_profile_sha256=full_profile_sha256('a' * 64, changed)).validate()


def test_identified_subset_gets_own_trial_anchor_and_preserves_nontrial_prior():
  full = base(('ff_gain_left', 'turn_in_boost_left')).for_trial(('ff_gain_left',)).with_proposal({'ff_gain_left': 0.005})
  assert full.trial_names == ('ff_gain_left',)
  assert full.trial_anchor == original_anchor('a' * 64, ('ff_gain_left',))
  assert full.full_original_anchor == full_original_anchor('a' * 64)
  assert dict(full.coordinates)['turn_in_boost_left'] == 0.0


@pytest.mark.parametrize(
  'field,value',
  [('cp_sha256', '0' * 64), ('source_epoch', '0' * 64), ('reset_generation', '1'), ('selected_baseline_revision', '0' * 64), ('context_key', '0' * 64)],
)
def test_live_readmission_requires_exact_current_source_and_prior(field, value):
  current = base()
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
  assert current.admit_context(**args) == current
  args[field] = value
  with pytest.raises(ValueError):
    current.admit_context(**args)


def test_original_bounds_step_and_full_content_hash_are_immutable():
  initial = base()
  with pytest.raises(ValueError):
    initial.with_proposal({'ff_gain_left': 0.006})
  with pytest.raises(ValueError):
    initial.with_proposal({'ff_gain_left': 0.3})
  with pytest.raises(ValueError):
    replace(initial, full_original_anchor='0' * 64).validate()
  with pytest.raises(ValueError):
    replace(initial, full_profile_sha256='0' * 64).validate()


def test_duplicate_json_members_and_oversized_namespace_rejected():
  with pytest.raises(ValueError):
    decode(b'{"version":2,"profiles":{},"fullSurfaces":{},"fullSurfaces":{}}')
  with pytest.raises(ValueError):
    decode(json.dumps({'version': 2, 'profiles': {}, 'fullSurfaces': dict.fromkeys([str(i) * 64 for i in range(5)], {})}).encode())


def wire_profile(full):
  from openpilot.starpilot.flm.adaptive_profile import Profile, ReplayValidation

  p = Profile(
    'f' * 64,
    1,
    10,
    500_000_010,
    parameters_for(full.trial_names),
    tuple(dict(full.coordinates)[name] for name in full.trial_names),
    tuple('train' + str(i) for i in range(4)),
    tuple('heldout' + str(i) for i in range(4)),
    1.0,
    0.5,
    (('actual-event', 0.1),),
    POLICY,
    full,
  )
  return p, ReplayValidation(p, 'a' * 64, 'b' * 64, True, True, 0.1)


def test_explicit_version2_wire_roundtrip_preserves_full30_and_original_leases():
  from openpilot.starpilot.flm.adaptive_profile import Result
  from openpilot.starpilot.flm.learning_wire import result_message, read_runtime_admission

  full = base().with_proposal({'ff_gain_left': 0.005})
  p, v = wire_profile(full)
  event = result_message(Result('qualified', 'actual_controller_replay', p), 'session', 1, 10, replay_validation=v)
  assert event.autoSteeringLearning.learning.runtimeProfile.version == 2
  assert event.autoSteeringLearning.learning.runtimeProfile.fullSurface.version == 1
  parsed = read_runtime_admission(event)
  assert parsed is not None and parsed[0] == p and parsed[1] == v
  assert parsed[0].observed_ns == 10 and parsed[0].expires_ns == 500_000_010


def test_wire_tampered_full_map_is_rejected():
  from openpilot.starpilot.flm.adaptive_profile import Result
  from openpilot.starpilot.flm.learning_wire import result_message, read_profile

  full = base().with_proposal({'ff_gain_left': 0.005})
  p, v = wire_profile(full)
  event = result_message(Result('qualified', 'actual_controller_replay', p), 'session', 1, 10, replay_validation=v)
  event.autoSteeringLearning.learning.runtimeProfile.fullSurface.coordinates[0].value = 0.004
  assert read_profile(event) is None


def test_new_policy_cannot_hide_envelope_in_legacy_wire_version():
  from openpilot.starpilot.flm.adaptive_profile import Result
  from openpilot.starpilot.flm.learning_wire import result_message, read_profile

  full = base().with_proposal({'ff_gain_left': 0.005})
  p, v = wire_profile(full)
  event = result_message(Result('qualified', 'actual_controller_replay', p), 'session', 1, 10, replay_validation=v)
  event.autoSteeringLearning.learning.runtimeProfile.version = 1
  assert read_profile(event) is None


def test_new_drive_generation_loads_values_as_prior_and_never_a_live_fit():
  first = base().with_proposal({'ff_gain_left': 0.005})
  raw = remember_full(None, profile(first), validation_sha256='3' * 64)
  current = replace(base(), source_epoch='8' * 64, context_key='8' * 64).validate()
  assert current.source_binding() != first.source_binding()
  assert current.persistent_binding() == first.persistent_binding()
  prior = load_prior(raw, binding=current.source_binding())
  assert prior.historical_only and prior.envelope.coordinates == first.coordinates
  assert prior.envelope.source_epoch == current.source_epoch
  assert prior.envelope.context_key == current.context_key
  assert prior.envelope.prior_coordinates == prior.envelope.coordinates
  assert not hasattr(prior, 'profile') and not hasattr(prior, 'expires_ns')
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
  restore = current.restore_from_history(prior)
  assert restore.coordinates == first.coordinates and restore.prior_coordinates == neutral_coordinates()
  # Old live envelope never acquires the fresh drive's permission.
  with pytest.raises(ValueError):
    first.admit_context(
      cp_sha256=current.cp_sha256,
      controller_revision=current.controller_revision,
      source_epoch=current.source_epoch,
      reset_generation=current.reset_generation,
      selected_baseline_revision=current.selected_baseline_revision,
      context_key=current.context_key,
      current_full_profile_sha256=first.prior_full_profile_sha256,
      persistent_source_epoch=current.persistent_source_epoch,
    )


@pytest.mark.parametrize(
  'change',
  [
    {'cp_sha256': '7' * 64},
    {'controller_revision': 'different-controller'},
    {'persistent_source_epoch': '7' * 64},
    {'reset_generation': '1'},
    {'selected_baseline_revision': '7' * 64},
  ],
)
def test_foreign_static_identity_or_reset_cannot_load_history(change):
  first = base().with_proposal({'ff_gain_left': 0.005})
  raw = remember_full(None, profile(first), validation_sha256='3' * 64)
  binding = list(first.source_binding())
  index = {'cp_sha256': 0, 'controller_revision': 1, 'source_epoch': 2, 'reset_generation': 3, 'selected_baseline_revision': 4, 'persistent_source_epoch': 7}
  for field, value in change.items():
    binding[index[field]] = value
  assert load_prior(raw, binding=tuple(binding)) is None


def test_unknown_vin_scope_cannot_be_persisted():
  transient = replace(base(), persistent_source_epoch='').validate()
  assert load_prior(None, binding=transient.source_binding()) is None
  with pytest.raises(ValueError):
    remember_full(None, profile(transient), validation_sha256='3' * 64)


def test_distinct_saved_configurations_survive_different_reset_generations():
  first = base().with_proposal({'ff_gain_left': 0.005})
  raw = remember_full(None, profile(first), validation_sha256='3' * 64)
  other = replace(base(), persistent_source_epoch='8' * 64, reset_generation='1').with_proposal({'ff_gain_left': 0.005})
  updated = remember_full(raw, profile(other), validation_sha256='4' * 64)
  assert load_prior(updated, binding=first.source_binding()) is not None
  assert load_prior(updated, binding=other.source_binding()) is not None
  assert len(decode(updated)['fullSurfaces']) == 2


def test_full_store_refuses_new_scope_without_evicting_existing_configs():
  raw = None
  accepted = []
  for generation in range(4):
    current = replace(base(), reset_generation=str(generation), persistent_source_epoch=hex(generation + 1)[2:] * 64).validate()
    proposal = current.with_proposal({'ff_gain_left': 0.005})
    raw = remember_full(raw, profile(proposal), validation_sha256='3' * 64)
    accepted.append(proposal)
  before = raw
  fifth = replace(base(), reset_generation='4', persistent_source_epoch='5' * 64).with_proposal({'ff_gain_left': 0.005})
  with pytest.raises(ValueError):
    remember_full(raw, profile(fifth), validation_sha256='3' * 64)
  assert raw == before
  assert all(load_prior(raw, binding=p.source_binding()) is not None for p in accepted)


def test_actual_read_source_stable_history_key_excludes_two_drive_uuids():
  from openpilot.starpilot.flm.tests.test_fixed_manual_baseline import fixed_controls, BASE
  from openpilot.starpilot.flm.runtime_adapter_registry import read_source
  from openpilot.starpilot.flm.learning_wire import write_source_receipt, read_source_receipt
  from openpilot.cereal import messaging

  with fixed_controls() as (controls, _):
    cp = controls.CP.as_builder()
    cp.carVin = '5YJ3E1EA7JF000001'
    kwargs = {'now_ns': BASE, 'calibration': (0.0, 0.0, 0.0), 'vehicle_session': 'drive-one'}
    first = read_source(cp, controls.params, calibration_generation='1' * 32, **kwargs)
    second = read_source(cp, controls.params, calibration_generation='2' * 32, **kwargs)
    assert first.enabled and second.enabled
    assert first.source_epoch != second.source_epoch
    assert first.persistent_source_epoch == second.persistent_source_epoch and first.persistent_source_epoch
    event = messaging.new_message('autoSteeringLearning', valid=True)
    write_source_receipt(event, second)
    assert event.autoSteeringLearning.learning.sourceReceipt.version == 3
    assert read_source_receipt(event, now_ns=BASE, cp_sha256=second.cp_sha256) == second
    cp.carVin = '5YJ3E1EA7JF000002'
    foreign = read_source(cp, controls.params, calibration_generation='2' * 32, **kwargs)
    assert foreign.persistent_source_epoch != second.persistent_source_epoch
    cp.carVin = '5YJ3E1EA7JF000001'
    cp.mass += 1.0
    config = read_source(cp, controls.params, calibration_generation='2' * 32, **kwargs)
    assert config.persistent_source_epoch != second.persistent_source_epoch
    cp.mass -= 1.0
    controls.params.put('AutoSteeringLearningReset', '1', block=True)
    reset = read_source(cp, controls.params, calibration_generation='2' * 32, **kwargs)
    assert reset.persistent_source_epoch != second.persistent_source_epoch


@pytest.mark.parametrize(
  'change',
  [
    {'cp_sha256': '7' * 64},
    {'controller_revision': 'wrong-controller'},
    {'source_epoch': '7' * 64, 'context_key': '7' * 64},
    {'selected_baseline_revision': '7' * 64},
    {'persistent_source_epoch': '7' * 64},
  ],
)
def test_persistence_rejects_foreign_envelope_even_with_matching_profile_identity_and_coeffs(change):
  from openpilot.starpilot.flm.learned_lifecycle import ProducerPersistence
  from unittest.mock import Mock

  full = base().with_proposal({'ff_gain_left': 0.005})
  p, v = wire_profile(full)
  foreign = replace(full, **change)
  changed = replace(p, effect_envelope=foreign)
  validation = replace(v, profile=changed)
  owner = ProducerPersistence(SimpleNamespace())
  owner.storage.save = Mock(return_value=True)
  assert not owner.update(
    changed,
    validation,
    now_ns=1_000_000_000,
    reset='0',
    current_sources=lambda: (p.identity, True, False, '0'),
    current_full_binding=lambda: full.source_binding(),
  )
  owner.storage.save.assert_not_called()


def test_persistence_rejects_replay_baseline_not_matching_original_envelope():
  from openpilot.starpilot.flm.learned_lifecycle import StorageOwner
  from unittest.mock import patch

  full = base().with_proposal({'ff_gain_left': 0.005})
  p, v = wire_profile(full)
  with patch('openpilot.starpilot.saved_source.read_saved', return_value=(None, True)), patch('openpilot.starpilot.saved_document.commit_exact') as commit:
    assert not StorageOwner(SimpleNamespace()).save(
      p, reset='0', replay_validation=replace(v, baseline_revision='7' * 64), authorized=lambda: True, full_binding=full.source_binding()
    )
  commit.assert_not_called()


def test_parent_only_manual_tuple_does_not_alias_wire_persistent_hash():
  from openpilot.starpilot.flm.tests.test_fixed_manual_baseline import fixed_controls, BASE
  from openpilot.starpilot.flm.runtime_adapter_registry import read_source
  from openpilot.starpilot.flm.learning_wire import write_source_receipt, read_source_receipt
  from openpilot.cereal import messaging

  with fixed_controls() as (controls, _):
    cp = controls.CP.as_builder()
    cp.carVin = '5YJ3E1EA7JF000001'
    source = read_source(cp, controls.params, now_ns=BASE, calibration=(0.0, 0.0, 0.0), vehicle_session='drive-one', calibration_generation='1' * 32)
    assert source.enabled and len(source.manual_baseline) == 3
    assert isinstance(source.persistent_source_epoch, str) and len(source.persistent_source_epoch) == 64
    event = messaging.new_message('autoSteeringLearning', valid=True)
    write_source_receipt(event, source)
    decoded = read_source_receipt(event, now_ns=BASE, cp_sha256=source.cp_sha256)
    assert decoded is not None and decoded.persistent_source_epoch == source.persistent_source_epoch
    assert decoded.manual_baseline == ()  # Parent-only fixed tuple never copied over IPC.
    assert decoded == source  # compare=False keeps wire-visible equality stable.
