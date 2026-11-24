"""Registered encoder/native-function units; no plant/profile or road authority."""
from dataclasses import replace
from unittest import mock

import pytest
from opendbc.car import DT_CTRL, rate_limit, structs
from opendbc.car.gm.values import CAR as GM
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.toyota.values import CAR as TOYOTA
from opendbc.car.steering_projection import _shaped_input, float32
from openpilot.starpilot.flm.clipping_quality import STATE_ALIGNED_DEBT
from openpilot.starpilot.flm.torque_emission_replay import Emission, EncoderBranch
from openpilot.starpilot.flm.tests.test_steering_projection import make_interface, control, apply, settle, install_observer


def captured_event(platform, *, modified=False):
  firmware = (structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.eps, address=0x18DA30F1, subAddress=0,
    fwVersion=b'39990-TGG,A020\x00\x00', brand='honda'),) if modified else ()
  ci = make_interface(platform, firmware)
  assert install_observer(ci)
  settle(ci, control(0.))
  requested = .8 if platform in (HONDA.HONDA_CIVIC, HONDA.HONDA_CIVIC_BOSCH) else .1
  for tick in range(20):
    _, packets, witness = apply(ci, control(requested))
    if witness is None:
      continue
    clipped = witness.unlimited_raw is None or witness.unlimited_raw != witness.final_raw
    if clipped and (witness.family == 'honda' or witness.previous_torque > 2):
      assert (witness.can_address, witness.can_data, witness.can_bus) in packets
      stamp = 10_000_000_000 + tick * 10_000_000
      # No native permission is granted by this codec unit fixture.
      return Emission(stamp, stamp, stamp + 1_000_000, witness, 'a' * 64, 'b' * 64, 'c' * 64,
        True, True, False, False, clipped, False, False, 0, 0, '')
  raise AssertionError('registered native encoder did not provide clipped context')


@pytest.mark.parametrize('platform', (TOYOTA.TOYOTA_COROLLA_TSS2, GM.GMC_ACADIA))
def test_registered_measured_driver_same_request_shadow_and_increased_demand(platform):
  event = captured_event(platform)
  witness = event.witness
  old = EncoderBranch()
  old.previous = witness.previous_torque - 1
  with pytest.raises(ValueError, match='clipping regression'):
    old.step(event, witness.requested_torque)
  new = EncoderBranch(quality_contract=STATE_ALIGNED_DEBT)
  new.previous = witness.previous_torque - 1
  assert new.step(event, witness.requested_torque) is not None
  assert new.previous == witness.final_raw - 1
  worse = EncoderBranch(quality_contract=STATE_ALIGNED_DEBT)
  worse.previous = witness.previous_torque - 1
  with pytest.raises(ValueError, match='clipping regression'):
    worse.step(event, witness.requested_torque + 1 / witness.effective_max)
  # With exact recorded one-tick state, old and new contracts are identical.
  old, new = EncoderBranch(), EncoderBranch(quality_contract=STATE_ALIGNED_DEBT)
  assert old.step(event, witness.requested_torque) == new.step(event, witness.requested_torque)
  assert old.previous == new.previous == witness.final_raw


def test_common_state_unlimited_reference_recorded_opposite_and_original_unlimited_vetoes():
  event = captured_event(TOYOTA.TOYOTA_COROLLA_TSS2)
  witness = event.witness
  branch = EncoderBranch(quality_contract=STATE_ALIGNED_DEBT)
  branch.previous = witness.unlimited_raw
  up = dict(witness.limits)['STEER_DELTA_UP']
  with pytest.raises(ValueError, match='clipping regression'):
    branch.step(event, (witness.unlimited_raw + up + 1) / witness.effective_max)
  branch = EncoderBranch(quality_contract=STATE_ALIGNED_DEBT)
  branch.previous = witness.unlimited_raw + 100
  with pytest.raises(ValueError, match='clipping regression'):
    branch.step(event, (witness.unlimited_raw + 1) / witness.effective_max)
  # Corrupting the original limited-context bit cannot authorize new clipping.
  with pytest.raises(ValueError, match='limiter/support'):
    EncoderBranch(quality_contract=STATE_ALIGNED_DEBT).step(replace(event, limited=False), witness.requested_torque)


def test_unsupported_paired_projection_fails_without_advancing_candidate_native_state():
  event = captured_event(TOYOTA.TOYOTA_COROLLA_TSS2)
  witness = event.witness
  branch = EncoderBranch(quality_contract=STATE_ALIGNED_DEBT)
  branch.previous = witness.previous_torque - 1
  before = branch.previous, branch.civic, branch.last_frame, branch.last_apply_ns
  from opendbc.car.steering_projection import project_request
  def unsupported_reference(row, request, *, shape=None):
    return None if float32(request) == witness.requested_torque else project_request(row, request, shape=shape)
  with mock.patch('opendbc.car.steering_projection.project_request', side_effect=unsupported_reference), \
       pytest.raises(ValueError, match='paired native request support'):
    branch.step(event, .79)
  assert (branch.previous, branch.civic, branch.last_frame, branch.last_apply_ns) == before


def test_honda_normalized_pre_lookup_comparison_and_modified_filter_shadow_is_pure():
  for platform, modified in ((HONDA.HONDA_CIVIC, False), (HONDA.HONDA_CIVIC_BOSCH, True)):
    event = captured_event(platform, modified=modified)
    witness = event.witness
    branch = EncoderBranch(quality_contract=STATE_ALIGNED_DEBT)
    branch.previous = witness.previous_torque - .001
    if modified:
      branch.civic = (witness.shape[0] + .0001,) + tuple(witness.shape[1:4])
    before = branch.civic
    shapes = []
    def observed_shape(row, request, shape, *, observed_branch=branch, prior_civic=before, observed_shapes=shapes):
      assert observed_branch.civic == prior_civic  # paired evaluation never advances filter
      observed_shapes.append(shape)
      return _shaped_input(row, request, shape)
    candidate_shape = None
    if modified:
      assert before is not None
      candidate_shape = before + tuple(witness.shape[4:])
    expected_source = _shaped_input(witness, witness.requested_torque, candidate_shape)
    params = dict(witness.limits)
    expected = rate_limit(expected_source, branch.previous,
      -params['STEER_DELTA_DOWN'] * DT_CTRL, params['STEER_DELTA_UP'] * DT_CTRL)
    with mock.patch('openpilot.starpilot.flm.torque_emission_replay._shaped_input', side_effect=observed_shape):
      assert branch.step(event, witness.requested_torque) is not None
    assert branch.previous == expected
    assert len(shapes) >= 3 and all(shape == candidate_shape for shape in shapes)


def test_honda_nonfinite_reference_and_paired_original_pressed_fail_closed():
  event = captured_event(HONDA.HONDA_CIVIC_BOSCH, modified=True)
  branch = EncoderBranch(quality_contract=STATE_ALIGNED_DEBT)
  branch.previous = event.witness.previous_torque - .001
  def bad_reference(row, request, shape):
    return float('nan') if float32(request) == row.requested_torque else _shaped_input(row, request, shape)
  with mock.patch('openpilot.starpilot.flm.torque_emission_replay._shaped_input', side_effect=bad_reference), \
       pytest.raises(ValueError, match='paired native request support'):
    branch.step(event, .7)
  # Real pressed predicate: candidate−.2 agrees with driver−1 and remains below
  # its .8s trigger; ORIGINAL+.8 opposes driver and crosses its .1s trigger.
  shape = tuple(event.witness.shape[:4]) + (18., True, -1.)
  event = replace(event, witness=event.witness._replace(shape=shape))
  branch = EncoderBranch(quality_contract=STATE_ALIGNED_DEBT)
  branch.previous = event.witness.previous_torque
  branch.civic = (1., -.2, .095, False)
  before = branch.previous, branch.civic, branch.last_frame, branch.last_apply_ns
  with pytest.raises(ValueError, match='driver override'):
    branch.step(event, -.2)
  assert (branch.previous, branch.civic, branch.last_frame, branch.last_apply_ns) == before
