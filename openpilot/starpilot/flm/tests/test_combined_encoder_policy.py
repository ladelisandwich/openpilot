"""Registered native encoders, no model/profile/admission authority."""
from dataclasses import replace
from unittest import mock

import pytest
from opendbc.car.gm.values import CAR as GM
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.toyota.values import CAR as TOYOTA
from openpilot.starpilot.flm.clipping_quality import STATE_ALIGNED_DEBT, STATE_ALIGNED_RELIEF, NATIVE_SHADOW_IDENTIFICATION
from openpilot.starpilot.flm.torque_emission_replay import EncoderBranch
from openpilot.starpilot.flm.tests.test_state_aligned_encoder import captured_event
from openpilot.starpilot.flm.tests.test_steering_projection import make_interface, control, apply, settle, install_observer


def shadow_branch():
  return EncoderBranch(quality_contract=STATE_ALIGNED_RELIEF, identification_policy=NATIVE_SHADOW_IDENTIFICATION)


@pytest.mark.parametrize('platform', (TOYOTA.TOYOTA_COROLLA_TSS2, GM.GMC_ACADIA, HONDA.HONDA_CIVIC, HONDA.HONDA_CIVIC_BOSCH))
def test_real_native_shadow_bounded_outward_request_still_fails_final_quality(platform):
  event = captured_event(platform, modified=platform == HONDA.HONDA_CIVIC_BOSCH)
  # Both requests lie in a flat native rate limit. No unlimited torque is emitted.
  a, b = shadow_branch(), shadow_branch()
  assert a.step(event, .95) == b.step(event, 1.)
  assert a.previous == b.previous
  with pytest.raises(ValueError, match='clipping regression'):
    EncoderBranch(quality_contract=STATE_ALIGNED_RELIEF).step(event, 1.)
  with pytest.raises(ValueError, match='native evidence'):
    shadow_branch().step(event, event.witness.requested_torque, actual=True)
  with pytest.raises(ValueError, match='combined quality proof'):
    EncoderBranch(quality_contract=STATE_ALIGNED_DEBT, identification_policy=NATIVE_SHADOW_IDENTIFICATION)


@pytest.mark.parametrize('platform', (TOYOTA.TOYOTA_COROLLA_TSS2, GM.GMC_ACADIA))
def test_real_driver_measured_unwind_relief_is_weaker_and_final_only(platform):
  ci = make_interface(platform)
  assert install_observer(ci)
  settle(ci, control(.1))
  witness = None
  for _ in range(4):
    _, _, witness = apply(ci, control(.01))
    if witness is not None:
      break
  assert witness is not None
  assert witness.unlimited_raw and 0 < witness.unlimited_raw < witness.final_raw
  event = replace(captured_event(platform), witness=witness, limited=True)
  candidate = (witness.unlimited_raw - 1) / witness.effective_max
  assert candidate > 0
  with pytest.raises(ValueError, match='clipping regression'):
    EncoderBranch(quality_contract=STATE_ALIGNED_DEBT).step(event, candidate)
  branch = EncoderBranch(quality_contract=STATE_ALIGNED_RELIEF)
  assert branch.step(event, candidate) is not None
  assert branch.previous is not None
  assert 0 < branch.previous <= witness.final_raw
  # Actual recorded no-clipping status cannot be bypassed by relief.
  with pytest.raises(ValueError, match='limiter/support'):
    EncoderBranch(quality_contract=STATE_ALIGNED_RELIEF).step(replace(event, limited=False), candidate)


@pytest.mark.parametrize('platform', (TOYOTA.TOYOTA_COROLLA_TSS2, HONDA.HONDA_CIVIC_BOSCH))
def test_shadow_unsupported_projection_and_chronology_still_fail_closed(platform):
  event = captured_event(platform, modified=platform == HONDA.HONDA_CIVIC_BOSCH)
  with mock.patch('opendbc.car.steering_projection.project_request', return_value=None), pytest.raises(ValueError):
    shadow_branch().step(event, 1.)
  branch = shadow_branch()
  branch.step(event, 1.)
  with pytest.raises(ValueError, match='chronology/selection'):
    branch.step(event, 1.)
  with pytest.raises((ValueError, OverflowError)):
    shadow_branch().step(event, float('nan'))


def test_shadow_modified_civic_driver_override_remains_a_veto():
  event = captured_event(HONDA.HONDA_CIVIC_BOSCH, modified=True)
  witness = event.witness._replace(shape=tuple(event.witness.shape[:4]) + (18., True, -1.))
  branch = shadow_branch()
  branch.previous = witness.previous_torque
  branch.civic = (1., -.2, .095, False)
  with pytest.raises(ValueError, match='driver override'):
    branch.step(replace(event, witness=witness), .8)


@pytest.mark.parametrize('platform', (TOYOTA.TOYOTA_COROLLA_TSS2, GM.GMC_ACADIA))
def test_shadow_can_measure_new_clipped_flat_region_but_final_cannot_admit_it(platform):
  ci = make_interface(platform)
  assert install_observer(ci)
  witness = settle(ci, control(.1))[-1][2]
  assert witness.unlimited_raw == witness.final_raw
  event = replace(captured_event(platform), witness=witness, limited=False)
  branch = shadow_branch()
  assert branch.step(event, 1.) is not None
  assert abs(branch.previous) <= witness.effective_max
  assert branch.previous != round(witness.effective_max)
  with pytest.raises(ValueError, match='limiter/support'):
    EncoderBranch(quality_contract=STATE_ALIGNED_RELIEF).step(event, 1.)


@pytest.mark.parametrize('platform,modified', ((HONDA.HONDA_CIVIC, False), (HONDA.HONDA_CIVIC_BOSCH, True)))
def test_real_honda_unwind_uses_decoded_output_and_full_filter(platform, modified):
  from opendbc.car import structs
  from opendbc.car.steering_projection import normalized_applied_command
  firmware = (structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.eps, address=0x18DA30F1, subAddress=0,
    fwVersion=b'39990-TGG,A020\x00\x00', brand='honda'),) if modified else ()
  ci = make_interface(platform, firmware)
  assert install_observer(ci)
  settle(ci, control(.8))
  witness = None
  for _ in range(4):
    _, _, witness = apply(ci, control(.01))
    if witness is not None:
      break
  assert witness is not None and 0 < witness.codec_input_torque < witness.previous_torque
  event = replace(captured_event(platform, modified=modified), witness=witness, limited=True)
  with pytest.raises(ValueError, match='clipping regression'):
    EncoderBranch(quality_contract=STATE_ALIGNED_DEBT).step(event, .005)
  branch = EncoderBranch(quality_contract=STATE_ALIGNED_RELIEF)
  delivered = branch.step(event, .005)
  assert 0 < delivered <= normalized_applied_command(witness)
  # The encoded CAN raw sign is inverse to the physical normalized command.
  assert witness.final_raw < 0 < delivered
  if modified:
    assert branch.civic is not None and branch.civic != tuple(witness.shape[:4])
