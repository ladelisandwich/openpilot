"""Synthetic numerical/protocol tests; never actual native or road proof."""
from typing import Any


from dataclasses import replace
from openpilot.starpilot.flm.runtime_adapter_registry import digest
import pytest
import json

from openpilot.starpilot.flm.live_trial_contract import LiveTrialBinding
from openpilot.starpilot.flm.live_tracking_response import (
  ResponseCertificate,
  VERSION,
  _revision,
  validate_certificate,
  physical_calibration_generation,
)
from openpilot.starpilot.flm.live_tracking_requalification import (
  StableResponseBasis,
  RetainedResponseRequalifier,
  _primitive_key,
  _rebound,
  compatible_basis,
  retained_to_dict,
  retained_from_dict,
)


def certificate():
  binding = LiveTrialBinding(
    'a' * 64,
    'b' * 64,
    'actual-controller-registry-v2:' + 'c' * 64,
    'd' * 64,
    'e' * 64,
    'drive1-reset',
    'f' * 32,
    digest((15.0, 100000.0, 110000.0)),
    '',
    '0' * 64,
    'b' * 64,
    False,
  )
  raw = ResponseCertificate(
    VERSION,
    binding,
    '0' * 64,
    170_000_000,
    3.0,
    0.0,
    -0.3,
    0.3,
    0.005,
    ('train0', 'train1', 'train2', 'train3'),
    ('hold0', 'hold1', 'hold2', 'hold3'),
    'c' * 64,
    'd' * 64,
    1,
  )
  revision = _revision(raw)
  return validate_certificate(replace(raw, model_revision=revision, binding=replace(binding, physical_model_revision=revision)))


def basis(cert):
  b = cert.binding
  return StableResponseBasis(
    b.identity,
    b.cp_sha256,
    b.controller_revision,
    b.baseline_revision,
    (0.0, 0.01, -0.01),
    (15.0, 100000.0, 110000.0, 15.0, 2.8, 1.1, 1700.0),
    ('ordinary-hardware-v1', ('measured-hardware-type',), 0, (('hyundai', 0),), ((0, 'hyundai', 0, 0),)),
    False,
  )


def new_drive(cert):
  return replace(
    cert.binding, identity='6' * 64, source_epoch='2' * 64, reset_generation='new-UUID', calibration_generation='3' * 32, persistent_source_epoch='5' * 64
  )


def test_session_and_generation_rotation_does_not_require_eight_new_identification_slots():
  old = certificate()
  owner = RetainedResponseRequalifier(old, basis(old), new_drive(old))
  assert owner.original is old
  assert owner._candidate.binding.source_epoch == '2' * 64
  assert owner.qualified(10_000_000_000) is None
  assert owner.reason == 'fresh_independent_requalification_required'
  assert not hasattr(owner, 'current_permission')
  assert not hasattr(owner, 'profile')


@pytest.mark.parametrize(
  'name,value',
  [
    ('cp_sha256', '1' * 64),
    ('controller_revision', 'other-controller'),
    ('baseline_revision', '1' * 64),
    ('native_required', True),
  ],
)
def test_actual_car_tune_controller_or_native_domain_change_cannot_rebind(name, value):
  old = certificate()
  changed = replace(new_drive(old), **{name: value})
  if name == 'native_required':
    changed = replace(changed, native_identity='1' * 64)
  with pytest.raises(ValueError, match='car/controller/tune/native domain'):
    RetainedResponseRequalifier(old, basis(old), changed)


@pytest.mark.parametrize(
  'name,value',
  [
    ('identity', '1' * 64),
    ('cp_sha256', '1' * 64),
    ('controller_revision', 'other-controller'),
    ('baseline_revision', '1' * 64),
    ('native_required', True),
  ],
)
def test_historical_basis_cannot_be_attached_to_a_different_model(name, value):
  old = certificate()
  with pytest.raises(ValueError, match='retained certificate/basis'):
    RetainedResponseRequalifier(old, replace(basis(old), **{name: value}), new_drive(old))


@pytest.mark.parametrize(
  'name,value',
  [
    ('calibration_rpy', (0.0, 0.02, -0.01)),
    ('vehicle_model', (15.0, 100000.0, 110000.0, 16.0, 2.8, 1.1, 1700.0)),
    ('hardware', ('different-measured-inventory',)),
  ],
)
def test_measured_calibration_geometry_or_hardware_is_not_an_ephemeral_generation(name, value):
  old = certificate()
  original = basis(old)
  assert original.key() != replace(original, **{name: value}).key()


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), True, object(), [1]])
def test_basis_rejects_nonfinite_custom_or_mutable_physical_leaves(bad):
  old = certificate()
  with pytest.raises(ValueError):
    replace(basis(old), calibration_rpy=(bad, 0.0, 0.0)).key()


def test_typed_context_never_coerces_signed_zero_or_bool_int():
  assert _primitive_key((True,)) != _primitive_key((1,))
  assert _primitive_key((0.0,)) != _primitive_key((-0.0,))


def test_rebound_keeps_historical_identification_and_marks_only_numerical_provenance():
  old = certificate()
  rebound = _rebound(old, new_drive(old), '6' * 64)
  assert rebound.provenance == 'retained-current-residual-v1'
  assert rebound.parent_model_revision == old.model_revision
  assert rebound.requalification_sha256 == '6' * 64
  assert rebound.model_revision != old.model_revision
  assert rebound.identified_ns == old.identified_ns
  assert rebound.train_block_ids == old.train_block_ids
  assert rebound.heldout_block_ids == old.heldout_block_ids
  assert rebound.identification_sha256 == old.identification_sha256
  assert rebound.source_inventory_sha256 == old.source_inventory_sha256
  assert (rebound.slope, rebound.intercept, rebound.lag_ns) == (old.slope, old.intercept, old.lag_ns)


def test_missing_actual_pose_native_or_hardware_cannot_qualify_retained_model():
  old = certificate()
  owner = RetainedResponseRequalifier(old, basis(old), new_drive(old))
  assert not owner.observe(object(), object(), object(), now_ns=1, current_native=False)
  assert owner.qualified(1) is None
  assert not owner._rows
  assert not owner._monitor.rows


def test_vm_uses_existing_measured_two_percent_regime_not_a_session_generation():
  original = basis(certificate())
  nearby = replace(original, vehicle_model=(15.1, 100500.0, 110100.0, *original.vehicle_model[3:]))
  outside = replace(original, vehicle_model=(16.0, *original.vehicle_model[1:]))
  assert compatible_basis(original, nearby)
  assert not compatible_basis(original, outside)
  assert not compatible_basis(original, replace(nearby, calibration_rpy=(0.0, 0.02, -0.01)))


def test_historical_model_json_roundtrip_never_contains_a_pending_trial_or_permission():
  original = certificate()
  raw = json.loads(json.dumps(retained_to_dict(original, basis(original))))
  restored, measured = retained_from_dict(raw)
  assert restored == original
  assert measured.key() == basis(original).key()
  assert raw["current_permission"] is False
  assert raw["pending_trial"] is None
  assert RetainedResponseRequalifier(restored, measured, new_drive(restored)).qualified(1) is None


@pytest.mark.parametrize("changes", [{"current_permission": True}, {"pending_trial": {"target": 1}}, {"unknown": 1}])
def test_history_never_restores_pending_target_or_permission(changes):
  original = certificate()
  raw = retained_to_dict(original, basis(original))
  raw.update(changes)
  with pytest.raises(ValueError, match="historical numerical model"):
    retained_from_dict(raw)


def test_frozen_bypass_basis_mutation_cannot_renew_cached_compatibility():
  original = certificate()
  measured = basis(original)
  owner = RetainedResponseRequalifier(original, measured, new_drive(original))
  object.__setattr__(measured, "hardware", ("swapped-hardware",))
  assert not owner.observe(object(), object(), object(), now_ns=1, current_native=True)
  assert owner.reason == "retained stable basis mutated"
  with pytest.raises(ValueError, match="retained stable basis mutated"):
    owner.qualified(1)


def synthetic_pair(owner, index, *, command=None):
  # Genuine evidence classes/validators, but explicitly synthetic clock/native
  # fixtures. This checks the protocol path, never road or native acceptance.
  from openpilot.starpilot.flm.fullrate_collector import ControlEvidence, AppliedEvidence
  from openpilot.starpilot.flm.original_source_capture import OriginalSourcePose
  from opendbc.car.steering_projection import TorqueProjection

  b = owner._binding
  mono = 10_000_000_000 + index * 50_000_000
  apply = mono - owner.original.lag_ns
  value = (0.1 if index % 2 else -0.1) if command is None else command
  raw = int(-value * 1000)
  limits = (
    ("STEER_DELTA_UP", 10.0),
    ("STEER_DELTA_DOWN", 10.0),
    ("STEER_DRIVER_ALLOWANCE", 50.0),
    ("STEER_DRIVER_MULTIPLIER", 2.0),
    ("STEER_DRIVER_FACTOR", 1.0),
  )
  projection = TorqueProjection(
    1,
    "hyundai",
    "synthetic",
    0,
    "hyundai.LKAS11",
    "round",
    index,
    -value,
    -value,
    raw,
    raw,
    1000.0,
    (),
    (),
    "driver",
    -value,
    0.0,
    limits,
    (),
    True,
    832,
    0,
    b"12345678",
    -value,
  )
  control = ControlEvidence(
    b.identity,
    1,
    apply,
    apply - 1_000_000,
    "normalized_torque",
    "stage",
    "response",
    -value,
    (),
    0.0,
    True,
    False,
    True,
    True,
    False,
    False,
    False,
    False,
    False,
    False,
    source_epoch=b.source_epoch,
    baseline_revision=b.baseline_revision,
    controller_revision=b.controller_revision,
    native_required=False,
    native_identity="",
    vehicle_model_anchor=(15.0, 100000.0, 110000.0),
    replay_inputs=(("vm.sR", 15.0), ("vm.cF", 100000.0), ("vm.cR", 110000.0)),
  )
  output = AppliedEvidence(apply, apply - 1_000_000, apply, "normalized_torque", -value, True, True, False, False, False, True, projection=projection)
  config = (0, (("hyundai", 0),), ((0, "hyundai", 0, 0, True, False, False, ()),))
  context = (
    "new-session",
    b.identity,
    1,
    b.source_epoch,
    b.baseline_revision,
    b.controller_revision,
    physical_calibration_generation(b),
    ("clock-generation", 2),
    ("synthetic",),
    "",
    (15.0, 100000.0, 110000.0),
    15.0,
    2.8,
    1.1,
    1700.0,
  )
  pose = OriginalSourcePose(
    mono,
    mono,
    0,
    0,
    0,
    physical_calibration_generation(b),
    "7" * 64,
    context,
    3 * value / 20.0,
    0.0,
    "",
    True,
    True,
    b.source_epoch,
    b.identity,
    b.baseline_revision,
    b.controller_revision,
    mono - 1_000_000,
    mono - 1_000_000,
    observed_ns=mono,
    completed_ns=mono,
    native_required=False,
    ordinary_safety=("ordinary-panda-current-v1", b.cp_sha256, mono, mono, config),
    source_lower_ns=mono,
    source_upper_ns=mono,
    source_timestamp_ns=mono,
    source_clock_domain="monotonic",
    source_clock_contract=("monotonic", "8" * 64, "9" * 64, "synthetic-fixture"),
    calibration_rpy=(0.0, 0.01, -0.01),
    angular_velocity_device=(0.0, 0.0, 3 * value / 20.0),
    orientation_ned=(0.0, 0.0, 0.0),
    speed_mps=20.0,
    left_state_ns=mono,
    right_state_ns=mono,
    left_state=(20.0, 0.0),
    right_state=(20.0, 0.0),
  )
  return control, output, pose


def feed(owner, index, *, command=None, current_native=True):
  control, output, pose = synthetic_pair(owner, index, command=command)
  return owner.observe(control, output, pose, now_ns=pose.mono_ns, current_native=current_native, hardware_inventory=("measured-hardware-type",))


def test_actual_evidence_validators_accept_fresh_synthetic_pairs_without_relabelling_eight_blocks():
  old = certificate()
  owner = RetainedResponseRequalifier(old, basis(old), new_drive(old))
  for index in range(7):
    assert feed(owner, index)
  assert owner.qualified(10_300_000_000) is None
  assert feed(owner, 7)
  cert, witness = owner.qualified(10_350_000_000)
  assert cert.provenance == "retained-current-residual-v1"
  assert cert.train_block_ids == old.train_block_ids
  assert cert.heldout_block_ids == old.heldout_block_ids
  assert cert.identified_ns == old.identified_ns
  assert witness.independent_poses == 8
  assert witness.binding.source_epoch == "2" * 64
  assert owner.qualified(10_450_000_001) is None


@pytest.mark.parametrize("command", [0.0, 0.001, 0.1])
def test_constant_encoded_command_cannot_define_a_local_range(command):
  old = certificate()
  owner = RetainedResponseRequalifier(old, basis(old), new_drive(old))
  for index in range(8):
    assert feed(owner, index, command=command)
  assert owner.qualified(10_350_000_000) is None
  assert owner.reason == 'fresh_local_range_required'


def test_small_one_sided_highway_tracking_requalifies_only_its_fresh_encoded_local_scope():
  old = certificate()
  owner = RetainedResponseRequalifier(old, basis(old), new_drive(old))
  for index in range(8):
    assert feed(owner, index, command=0.010 if index % 2 else 0.011)
  cert, witness = owner.qualified(10_350_000_000)
  assert (cert.command_low, cert.command_high) == (0.010, 0.011)
  assert cert.command_low > old.command_low
  assert cert.command_high < old.command_high
  assert cert.slope == old.slope
  assert cert.parent_model_revision == old.model_revision
  assert witness.independent_poses == 8
  # Publication and a new passing window cannot silently expand this scope.
  assert feed(owner, 8, command=0.010)
  same, _ = owner.qualified(10_400_000_000)
  assert same is cert
  assert not feed(owner, 9, command=0.012)
  assert owner.qualified(10_450_000_000) is None
  assert owner._frozen_certificate is cert


def test_local_scope_cannot_expand_old_independently_identified_support():
  old = certificate()
  for proposed in ((old.command_low - 0.001, 0.1), (-0.1, old.command_high + 0.001), (0.1, 0.1)):
    with pytest.raises(ValueError, match='cannot extrapolate'):
      _rebound(old, new_drive(old), '6' * 64, command_range=proposed)


def test_current_native_loss_erases_fresh_rows_without_erasing_historical_model():
  old = certificate()
  owner = RetainedResponseRequalifier(old, basis(old), new_drive(old))
  for index in range(8):
    assert feed(owner, index)
  assert owner.qualified(10_350_000_000) is not None
  assert not feed(owner, 8, current_native=False)
  assert owner.qualified(10_400_000_000) is None
  assert owner.original is old


def test_capture_epoch_rotation_alone_can_requalify_with_fresh_pairs():
  old = certificate()
  rotated = replace(old.binding, source_epoch="2" * 64)
  owner = RetainedResponseRequalifier(old, basis(old), rotated)
  for index in range(8):
    assert feed(owner, index)
  cert, witness = owner.qualified(10_350_000_000)
  assert cert.binding.reset_generation == old.binding.reset_generation
  assert witness.binding.source_epoch != old.binding.source_epoch


@pytest.mark.parametrize("bad", ["residual", "manual", "limited", "calibration", "controller", "lag"])
def test_fresh_requalification_rejects_actual_evidence_guard_failures(bad):
  old = certificate()
  owner = RetainedResponseRequalifier(old, basis(old), new_drive(old))
  control, output, pose = synthetic_pair(owner, 0)
  if bad == "residual":
    pose = replace(pose, yaw_rate=pose.yaw_rate + 0.1)
    # A single outlier may be collected; it is never a physical witness.
    # Current policy tests the complete independent rolling RMS window.
    assert owner.observe(control, output, pose, now_ns=pose.mono_ns, current_native=True, hardware_inventory=("measured-hardware-type",))
    assert owner.qualified(pose.mono_ns) is None
    for index in range(1, 8):
      assert feed(owner, index)
      assert owner.qualified(10_000_000_000 + index * 50_000_000) is None
    assert owner._monitor.witness(10_350_000_000) is None
    assert owner._frozen_certificate is None
    return
  elif bad == "manual":
    control = replace(control, manual=True)
  elif bad == "limited":
    output = replace(output, limited=True)
  elif bad == "calibration":
    pose = replace(pose, calibration_rpy=(0.0, 0.02, -0.01))
  elif bad == "controller":
    control = replace(control, controller_revision="wrong-controller")
  else:
    pose = replace(pose, source_lower_ns=pose.mono_ns - 40_000_000)
  assert not owner.observe(control, output, pose, now_ns=pose.mono_ns, current_native=True, hardware_inventory=("measured-hardware-type",))
  assert owner.qualified(pose.mono_ns) is None


def test_axis_session_can_change_only_with_fresh_native_inventory_and_residuals():
  from openpilot.starpilot.flm.runtime_adapter_registry import native_identity_digest

  key = ("old-axis-session", "actual-serial", 1, 0, ((0, "actual-serial", 1, 0, 0),))
  old = certificate()
  raw = replace(
    old,
    model_revision="0" * 64,
    binding=replace(old.binding, native_required=True, native_identity=native_identity_digest(key), physical_model_revision="0" * 64),
  )
  revision = _revision(raw)
  old = validate_certificate(replace(raw, model_revision=revision, binding=replace(raw.binding, physical_model_revision=revision)))
  measured = replace(basis(old), native_required=True, hardware=("axis-hardware-v1", *key[1:]))
  new_key = ("new-axis-session", *key[1:])
  binding = replace(new_drive(old), native_identity=native_identity_digest(new_key))
  owner = RetainedResponseRequalifier(old, measured, binding)
  assert owner.qualified(1) is None
  for index in range(8):
    control, output, pose = synthetic_pair(owner, index)
    control = replace(
      control,
      native_required=True,
      native_identity=binding.native_identity,
      native_allowed=True,
      native_observed_ns=control.control_ns - 1_000_000,
      native_expires_ns=control.control_ns + 100_000_000,
    )
    pose = replace(
      pose,
      native_required=True,
      native_identity=binding.native_identity,
      native_control_ns=pose.mono_ns - 1_000_000,
      native_observed_ns=pose.mono_ns - 2_000_000,
      native_expires_ns=pose.mono_ns + 100_000_000,
      ordinary_safety=(),
    )
    assert owner.observe(control, output, pose, now_ns=pose.mono_ns, current_native=True, native_key=new_key)
  rebound, witness = owner.qualified(10_350_000_000)
  assert rebound.binding.native_identity == native_identity_digest(new_key)
  assert rebound.parent_model_revision == old.model_revision
  assert witness.independent_poses == 8
  changed_hardware = (new_key[0], "different-serial", *new_key[2:])
  assert not owner.observe(control, output, pose, now_ns=pose.mono_ns, current_native=True, native_key=changed_hardware)
  assert owner.qualified(pose.mono_ns) is None


def test_raw_live_binding_calibration_and_physical_pose_hash_are_distinct_actual_representations():
  original = certificate()
  owner = RetainedResponseRequalifier(original, basis(original), new_drive(original))
  _, _, pose = synthetic_pair(owner, 0)
  assert len(owner._binding.calibration_generation) == 32
  assert len(pose.calibration) == 64
  assert pose.calibration != owner._binding.calibration_generation
  assert pose.calibration == physical_calibration_generation(owner._binding)


def test_selected_identity_is_ephemeral_but_historical_identity_provenance_remains_exact():
  original = certificate()
  current = new_drive(original)
  assert current.identity != original.binding.identity
  owner = RetainedResponseRequalifier(original, basis(original), current)
  for index in range(8):
    assert feed(owner, index)
  cert, witness = owner.qualified(10_350_000_000)
  assert cert.binding.identity == current.identity
  assert original.binding.identity == basis(original).identity
  assert witness.binding.identity == current.identity


def test_monotone_startup_does_not_freeze_a_range_before_original_input_revisits_it():
  old = certificate()
  owner = RetainedResponseRequalifier(old, basis(old), new_drive(old))
  for index in range(8):
    assert feed(owner, index, command=0.010 + 0.001 * index)
  assert owner.qualified(10_350_000_000) is None
  assert owner.reason == 'fresh_local_range_revisit_required'
  assert owner._frozen_certificate is None
  assert feed(owner, 8, command=0.016)
  assert owner.qualified(10_400_000_000) is None
  assert owner._frozen_certificate is None
  assert feed(owner, 9, command=0.017)
  cert, witness = owner.qualified(10_450_000_000)
  assert (cert.command_low, cert.command_high) == (0.010, 0.017)
  assert cert.command_low >= old.command_low and cert.command_high <= old.command_high
  assert witness.independent_poses == 10
  assert cert.slope == old.slope and cert.lag_ns == old.lag_ns
  assert not feed(owner, 10, command=0.018)
  assert owner.qualified(10_500_000_000) is None
  assert owner._frozen_certificate is cert


def test_unfinished_restore_retries_only_input_scope_with_no_monitor_or_permission_reuse():
  from types import SimpleNamespace
  from openpilot.starpilot.flm.steerlearnerd import _LiveHistoryRestore

  old = certificate()
  current = new_drive(old)
  owner = RetainedResponseRequalifier(old, basis(old), current)
  for index in range(8):
    assert feed(owner, index, command=0.010 if index % 2 else 0.011)
  local, _ = owner.qualified(10_350_000_000)
  assert not feed(owner, 8, command=0.020)
  assert owner.reason == 'response_command_outside_identified_range'
  h: Any = _LiveHistoryRestore(())
  h.prior = SimpleNamespace(certificate=old, response_basis=basis(old))
  h.requalifier = owner
  h.offer = object()
  calls = []
  ingress = SimpleNamespace(optimizer=SimpleNamespace(offer_state=None), monitor=owner._monitor)
  actor = SimpleNamespace(
    seed=current,
    restore_pending=True,
    ingress=ingress,
    certificate=local,
    application=object(),
    initial_basis=object(),
    interrupt=lambda *args, **kw: calls.append((args, kw)),
    hold_restore=lambda value: calls.append(value),
  )
  collector = SimpleNamespace(live_tracking=ingress, original_source_capture=SimpleNamespace(rows=[SimpleNamespace(mono_ns=10_400_000_000)]))
  assert h._retry_local_scope(actor, collector, current)
  assert actor.ingress is actor.certificate is actor.application is actor.initial_basis is None
  assert collector.live_tracking is h.offer is None
  assert h.last_pose_ns == 10_400_000_000
  assert h.requalifier is not owner and h.requalifier.original is old
  assert not h.requalifier._rows and h.requalifier.qualified(10_400_000_000) is None
  assert h.blocked_seed is None and not h.restored
  assert calls[0] == (('retained_local_input_scope_left',), {'context_changed': True})
  assert calls[-1] is True
  # The old issued certificate NEVER grows, including after the new owner
  # measures a distinct local range under fresh physical/source validation.
  for index in range(9, 17):
    assert feed(h.requalifier, index, command=0.020 if index % 2 else 0.021)
  fresh, _ = h.requalifier.qualified(10_800_000_000)
  assert (fresh.command_low, fresh.command_high) == (0.020, 0.021)
  assert (local.command_low, local.command_high) == (0.010, 0.011)
  assert fresh.model_revision != local.model_revision


@pytest.mark.parametrize(
  'reason', ('physical_response_drift', 'response_current_source_or_native_unavailable', 'response_original_source_pair_unqualified', 'response_source_gap')
)
def test_restore_scope_retry_never_reclassifies_faults_as_input_departure(reason):
  from types import SimpleNamespace
  from openpilot.starpilot.flm.steerlearnerd import _LiveHistoryRestore

  h: Any = _LiveHistoryRestore(())
  h.prior = object()
  h.requalifier = object()
  actor = SimpleNamespace(
    restore_pending=True,
    ingress=SimpleNamespace(optimizer=SimpleNamespace(offer_state=None), monitor=SimpleNamespace(reason=reason)),
    certificate=object(),
    seed='unchanged',
  )
  assert not h._retry_local_scope(actor, object(), 'unchanged')
  assert h.prior is not None and h.requalifier is not None
