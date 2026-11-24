"""Exact native zero-reference boundary; no fit, native authority or plant result."""
from dataclasses import asdict, replace
from unittest import mock

import pytest
from opendbc.car import DT_CTRL, rate_limit
from opendbc.car.gm.values import CAR as GM
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.toyota.values import CAR as TOYOTA
from opendbc.car.steering_projection import normalized_applied_command, _shaped_input, float32
from openpilot.starpilot.flm.clipping_quality import (
  ZERO_REFERENCE_QUALITY, EXACT_EMISSION_QUALITY, QUALIFIED_EVENT_CLIPPING_QUALITY,
  NATIVE_SHADOW_IDENTIFICATION, FIXED_MAP_IDENTIFICATION, RESTORE_ZERO_REFERENCE_VERSION,
  RESTORE_ACTIVATION_VERSION, RESTORE_AGGREGATE_ACTIVATION_VERSION,
  zero_reference_relief_is_admitted, native_relief_is_admitted, event_clipping_quality, restore_policy, restore_quality,
)
from openpilot.starpilot.flm.event_activation import (
  ZERO_REFERENCE_ACTIVATION_VERSION, bind_activation_model, activation_matches_model, activation_model_from_dict, model_clipping_quality,
)
from openpilot.starpilot.flm.restore_activation import restore_plan_from_dict
from openpilot.starpilot.flm.torque_emission_replay import EncoderBranch
from openpilot.starpilot.flm.tests.test_event_activation import fitted_model, primitive_plan
from openpilot.starpilot.flm.tests.test_state_aligned_encoder import captured_event
from openpilot.starpilot.flm.tests.test_steering_projection import make_interface, control, apply, settle, install_observer
from openpilot.starpilot.flm.tests.test_declared_restore import envelope, plan_for, payload


def test_exact_actual_v9_integer_witness_and_all_sign_mirrors():
  values = (-86, -86, -85, -87, -86, -86, 1, 0, 1)
  for sign in (-1, 1):
    mirrored = tuple(sign * value for value in values)
    assert not native_relief_is_admitted(*mirrored)
    assert zero_reference_relief_is_admitted(*mirrored)
  # This admits no nonzero reference debt and no generalized <= comparator.
  changes = ((6, 0), (7, 1), (7, -1), (8, 0), (2, -86), (2, -87),
             (2, 0), (2, 1), (4, -85), (5, -85), (5, -87), (3, -85),
             (4, 0), (6, -1), (8, -1), (0, True), (2, float('nan')), (5, float('inf')))
  for index, value in changes:
    changed: list[int | float] = list(values)
    changed[index] = value
    assert not zero_reference_relief_is_admitted(*changed), (index, value)


def test_partial_order_boundary_property_has_no_tolerance_or_zero_generalization():
  for original in range(3, 15):
    for reduction in range(1, original):
      for sign in (-1, 1):
        args = tuple(sign * v for v in (original, original, original - reduction,
          original + 2, original, original, -2, 0, -reduction))
        assert zero_reference_relief_is_admitted(*args)
        for index in range(9):
          changed: list[int | float] = list(args)
          if index == 7:
            changed[index] = sign * 1e-12
          else:
            changed[index] = 0
          assert not zero_reference_relief_is_admitted(*changed)


def unwind(platform):
  ci = make_interface(platform)
  assert install_observer(ci)
  settle(ci, control(.1))
  for _ in range(4):
    _, _, witness = apply(ci, control(.01))
    if witness is not None:
      break
  assert witness is not None and 1 < witness.unlimited_raw < witness.final_raw
  return replace(captured_event(platform), witness=witness, limited=True)


@pytest.mark.parametrize('platform', (TOYOTA.TOYOTA_COROLLA_TSS2, GM.GMC_ACADIA))
def test_registered_native_boundary_old_rejects_new_exact_reference_admits(platform):
  event = unwind(platform)
  witness = event.witness
  candidate = (witness.unlimited_raw - 1) / witness.effective_max
  previous = witness.unlimited_raw + dict(witness.limits)['STEER_DELTA_DOWN']
  old = EncoderBranch(quality_contract=EXACT_EMISSION_QUALITY)
  old.previous = previous
  with pytest.raises(ValueError, match='clipping regression'):
    old.step(event, candidate)
  branch = EncoderBranch(quality_contract=ZERO_REFERENCE_QUALITY)
  branch.previous = previous
  delivered = branch.step(event, candidate)
  assert branch.previous == witness.unlimited_raw
  assert delivered == normalized_applied_command(witness._replace(final_raw=witness.unlimited_raw))
  assert 0 < branch.previous < witness.final_raw
  # Future frame retains strict original-unclipped and own-state native checks.
  future = replace(event, control_ns=event.control_ns + 100_000_000,
    state_ns=event.state_ns + 100_000_000, apply_ns=event.apply_ns + 100_000_000, limited=False)
  with pytest.raises(ValueError):
    branch.step(future, 1.)
  for changed in (replace(event, limited=False), replace(event, witness=witness._replace(steer_requested=False)),
                  replace(event, witness=witness._replace(codec_id='volkswagen.HCA_01'))):
    denied = EncoderBranch(quality_contract=ZERO_REFERENCE_QUALITY)
    denied.previous = previous
    with pytest.raises(ValueError):
      denied.step(changed, candidate)


@pytest.mark.parametrize('modified', (False, True))
def test_honda_zero_reference_remains_in_shaped_normalized_and_decoded_domain(modified):
  platform = HONDA.HONDA_CIVIC_BOSCH if modified else HONDA.HONDA_CIVIC
  # Original production witness is a real normalized-rate unwind, inverse CAN sign.
  from opendbc.car import structs
  firmware = (structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.eps, address=0x18DA30F1, subAddress=0,
    fwVersion=b'39990-TGG,A020\x00\x00', brand='honda'),) if modified else ()
  ci = make_interface(platform, firmware)
  assert install_observer(ci)
  settle(ci, control(.8))
  for _ in range(4):
    _, _, witness = apply(ci, control(.01))
    if witness is not None:
      break
  event = replace(captured_event(platform, modified=modified), witness=witness, limited=True)
  shape = tuple(witness.shape) if witness.shape else ()
  reference = _shaped_input(witness, float32(witness.requested_torque), shape)
  down = dict(witness.limits)['STEER_DELTA_DOWN'] * DT_CTRL
  previous = reference + down
  assert rate_limit(reference, previous, -down, dict(witness.limits)['STEER_DELTA_UP'] * DT_CTRL) == reference
  # Candidate shaping and the full actual native function remain unstubbed.
  old = EncoderBranch(quality_contract=EXACT_EMISSION_QUALITY)
  old.previous = previous
  with pytest.raises(ValueError, match='clipping regression'):
    old.step(event, .005)
  branch = EncoderBranch(quality_contract=ZERO_REFERENCE_QUALITY)
  branch.previous = previous
  delivered = branch.step(event, .005)
  assert 0 < delivered <= normalized_applied_command(witness)
  assert branch.previous == reference and witness.final_raw < 0
  if modified:
    assert branch.civic is not None and branch.civic != tuple(witness.shape[:4])
  # Both original-paired and candidate override decisions still run before quality.
  if modified:
    pressed = replace(event, witness=witness._replace(shape=tuple(witness.shape[:4]) + (18., True, -1.)))
    veto = EncoderBranch(quality_contract=ZERO_REFERENCE_QUALITY)
    veto.previous = previous
    veto.civic = (1., -.2, .095, False)
    with pytest.raises(ValueError, match='driver override'):
      veto.step(pressed, .005)


def test_new_policy_trial_restore_model_proof_and_old_restore_mappings_are_distinct():
  assert QUALIFIED_EVENT_CLIPPING_QUALITY == EXACT_EMISSION_QUALITY
  assert tuple(restore_policy(v) for v in (4, 5, 6)) == (
    RESTORE_ACTIVATION_VERSION, RESTORE_AGGREGATE_ACTIVATION_VERSION, RESTORE_ZERO_REFERENCE_VERSION)
  assert restore_quality(4) == restore_quality(5) == EXACT_EMISSION_QUALITY
  assert restore_quality(6) == ZERO_REFERENCE_QUALITY
  row = {'event_version': 3, 'event_operation': 'trial', 'event_clipping_quality': ZERO_REFERENCE_QUALITY,
    'event_identification_policy': NATIVE_SHADOW_IDENTIFICATION}
  assert event_clipping_quality(row) == ZERO_REFERENCE_QUALITY
  trial = replace(primitive_plan(fitted_model()), version=ZERO_REFERENCE_ACTIVATION_VERSION).validate()
  assert trial.clipping_quality == ZERO_REFERENCE_QUALITY
  bound = bind_activation_model(fitted_model(), trial)
  assert model_clipping_quality(bound) == ZERO_REFERENCE_QUALITY
  assert activation_matches_model(trial, activation_model_from_dict(asdict(bound)))
  restore = replace(plan_for(envelope()), version=RESTORE_ZERO_REFERENCE_VERSION).validate()
  assert restore.identification_policy == FIXED_MAP_IDENTIFICATION and restore.clipping_quality == ZERO_REFERENCE_QUALITY
  model = bind_activation_model(fitted_model(), restore)
  assert model_clipping_quality(model) == ZERO_REFERENCE_QUALITY
  assert activation_matches_model(restore, activation_model_from_dict(asdict(model)))
  assert not activation_matches_model(replace(restore, version=RESTORE_AGGREGATE_ACTIVATION_VERSION), model)
  assert restore_plan_from_dict(asdict(restore)) == restore
  row = {**payload(), 'event_version': 6, 'event_restore_policy': RESTORE_ZERO_REFERENCE_VERSION,
    'event_clipping_quality': ZERO_REFERENCE_QUALITY}
  assert event_clipping_quality(row) == ZERO_REFERENCE_QUALITY
  for change in ({'event_version': 5}, {'event_restore_policy': RESTORE_AGGREGATE_ACTIVATION_VERSION},
                 {'event_clipping_quality': EXACT_EMISSION_QUALITY}, {'event_operation': 'trial'},
                 {'event_identification_policy': NATIVE_SHADOW_IDENTIFICATION}):
    with pytest.raises(ValueError):
      event_clipping_quality({**row, **change})
  for name in ('event_restore_policy', 'event_clipping_quality', 'event_identification_policy'):
    with pytest.raises(ValueError):
      event_clipping_quality({k: v for k, v in row.items() if k != name})


def test_actual_outer_factory_v6_dispatch_and_parent_wrong_version_rejections():
  from openpilot.starpilot.flm.tests import test_declared_restore as existing
  row = {**existing.payload(), 'event_version': 6, 'event_restore_policy': RESTORE_ZERO_REFERENCE_VERSION,
    'event_clipping_quality': ZERO_REFERENCE_QUALITY}
  with mock.patch.object(existing, 'payload', return_value=row):
    existing.test_actual_child_and_outer_factory_dispatch_restore_v4_without_fd()
  for configured, reported in ((6, 5), (5, 6), (6, 4), (4, 6)):
    existing.test_actual_parent_rejects_restore_report_from_other_configured_version(configured, reported)
