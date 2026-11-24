"""Registered same-tick CAN identity is local; it grants no fitted profile."""
from dataclasses import asdict, replace

import pytest
from opendbc.car import structs
from opendbc.car.gm.values import CAR as GM
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.hyundai.values import CAR as HYUNDAI
from opendbc.car.toyota.values import CAR as TOYOTA
from opendbc.car.volkswagen.values import CAR as VOLKSWAGEN
from openpilot.starpilot.flm.clipping_quality import (
  STATE_ALIGNED_RELIEF, EXACT_EMISSION_QUALITY, NATIVE_SHADOW_IDENTIFICATION, event_clipping_quality,
)
from openpilot.starpilot.flm.event_activation import (
  EXACT_EMISSION_ACTIVATION_VERSION, activation_matches_model, activation_model_from_dict, bind_activation_model,
)
from openpilot.starpilot.flm.torque_emission_replay import EncoderBranch, EXACT_EMISSION_CODECS, exact_local_emission
from openpilot.starpilot.flm.tests.test_event_activation import fitted_model, primitive_plan
from openpilot.starpilot.flm.tests.test_state_aligned_encoder import captured_event
from openpilot.starpilot.flm.tests.test_steering_projection import make_interface, control, apply, settle, install_observer

PLATFORMS = (TOYOTA.TOYOTA_COROLLA_TSS2, GM.GMC_ACADIA, HYUNDAI.HYUNDAI_SONATA,
             HYUNDAI.HYUNDAI_IONIQ_6, HONDA.HONDA_CIVIC, HONDA.HONDA_CIVIC_BOSCH)


def first_emission(ci, request):
  for _ in range(4):
    _, packets, witness = apply(ci, control(request))
    if witness is not None:
      return packets, witness
  raise AssertionError('registered original encoder did not emit')


def crossing(platform):
  modified = platform == HONDA.HONDA_CIVIC_BOSCH
  honda = platform in (HONDA.HONDA_CIVIC, HONDA.HONDA_CIVIC_BOSCH)
  firmware = (structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.eps, address=0x18DA30F1, subAddress=0,
    fwVersion=b'39990-TGG,A020\x00\x00', brand='honda'),) if modified else ()
  original, candidate = make_interface(platform, firmware), make_interface(platform, firmware)
  for ci in (original, candidate):
    assert install_observer(ci)
    settle(ci, control(-.8 if honda else -.1))
  maximum = original.CC.learning_projection.effective_max
  request = .01 if honda else 1 / maximum
  proposed = .02 if honda else 2 / maximum
  packets, witness = first_emission(original, request)
  trial_packets, trial = first_emission(candidate, proposed)
  assert witness.frame == trial.frame and witness.steer_requested is trial.steer_requested is True
  assert witness.codec_id in EXACT_EMISSION_CODECS
  assert witness.final_raw == trial.final_raw
  # Real registered packers preserve ALL current CAN payloads in this fixture,
  # despite different controller requests. No native permission is fabricated.
  assert packets == trial_packets
  assert (witness.can_address, witness.can_data, witness.can_bus) == (trial.can_address, trial.can_data, trial.can_bus)
  event = replace(captured_event(platform, modified=modified), witness=witness, limited=True)
  return original, candidate, event, proposed


@pytest.mark.parametrize('platform', PLATFORMS)
def test_real_same_tick_can_zero_crossing_and_candidate_state_commit(platform):
  _, candidate, event, proposed = crossing(platform)
  witness = event.witness
  assert proposed > witness.requested_torque > 0
  with pytest.raises(ValueError, match='clipping regression'):
    EncoderBranch(quality_contract=STATE_ALIGNED_RELIEF).step(event, proposed)
  branch = EncoderBranch(quality_contract=EXACT_EMISSION_QUALITY)
  assert branch.step(event, proposed) is not None
  if witness.family == 'honda':
    assert branch.previous == candidate.CC.last_torque
    if witness.shape:
      civic = candidate.CC.modified_civic_steering
      assert branch.civic == (civic.torque_lpf, civic.prev_torque_cmd,
        civic.steering_pressed_filter_s, civic.steering_pressed_robust_prev)
      assert branch.civic != tuple(witness.shape[:4])
  else:
    assert branch.previous == witness.final_raw


@pytest.mark.parametrize('platform', (TOYOTA.TOYOTA_COROLLA_TSS2, GM.GMC_ACADIA, HYUNDAI.HYUNDAI_IONIQ_6))
def test_local_equality_does_not_exempt_future_strict_new_clipping(platform):
  original, _, event, proposed = crossing(platform)
  branch = EncoderBranch(quality_contract=EXACT_EMISSION_QUALITY)
  branch.step(event, proposed)
  for index in range(64):
    _, witness = first_emission(original, event.witness.requested_torque)
    stamp = event.apply_ns + (index + 1) * 100_000_000
    future = replace(event, witness=witness, control_ns=stamp, state_ns=stamp, apply_ns=stamp + 1)
    if witness.unlimited_raw == witness.final_raw:
      # Original is now unclipped: a clipped candidate remains forbidden.
      with pytest.raises(ValueError, match='limiter/support'):
        branch.step(replace(future, limited=False), 1.)
      return
    branch.step(future, witness.requested_torque)
  raise AssertionError('fixed release did not reach its original unclipped endpoint')


def test_exact_local_metadata_enable_direction_and_one_sided_equality_negatives():
  _, _, event, _ = crossing(GM.GMC_ACADIA)
  witness = event.witness
  raw = witness.final_raw
  assert exact_local_emission(witness, raw, raw, 4, 4, 5)
  for reference, candidate in ((raw + 1, raw), (raw, raw + 1), (float(raw), raw), (raw, True)):
    assert not exact_local_emission(witness, reference, candidate, 4, 4, 5)
  for debts in ((4, -4, 5), (4, 4, -5), (4, 0, 5), (4, 4, float('nan'))):
    assert not exact_local_emission(witness, raw, raw, *debts)
  for changed in (witness._replace(steer_requested=False), witness._replace(codec_id='unknown.CODEC'),
                  witness._replace(family='honda'), witness._replace(can_data=b'')):
    assert not exact_local_emission(changed, raw, raw, 4, 4, 5)
  # A real known codec outside the audited allowlist keeps the old policy.
  other = captured_event(VOLKSWAGEN.VOLKSWAGEN_GOLF_MK7).witness
  assert not exact_local_emission(other, other.final_raw, other.final_raw, 4, 4, 5)
  with pytest.raises(ValueError, match='clipping regression'):
    EncoderBranch(quality_contract=EXACT_EMISSION_QUALITY).step(
      replace(event, witness=witness._replace(steer_requested=False)), 2 / witness.effective_max)


def test_honda_paired_override_remains_before_local_equality():
  event = captured_event(HONDA.HONDA_CIVIC_BOSCH, modified=True)
  witness = event.witness._replace(shape=tuple(event.witness.shape[:4]) + (18., True, -1.))
  branch = EncoderBranch(quality_contract=EXACT_EMISSION_QUALITY)
  branch.previous = witness.previous_torque
  branch.civic = (1., -.2, .095, False)
  before = branch.previous, branch.civic, branch.last_frame, branch.last_apply_ns
  with pytest.raises(ValueError, match='driver override'):
    branch.step(replace(event, witness=witness), -.2)
  assert (branch.previous, branch.civic, branch.last_frame, branch.last_apply_ns) == before


def test_new_exact_policy_model_binding_and_matching_identification_are_required():
  payload = {'event_version': 3, 'event_clipping_quality': EXACT_EMISSION_QUALITY,
    'event_identification_policy': NATIVE_SHADOW_IDENTIFICATION}
  assert event_clipping_quality(payload) == EXACT_EMISSION_QUALITY
  with pytest.raises(ValueError):
    event_clipping_quality({key: value for key, value in payload.items() if key != 'event_identification_policy'})
  model = fitted_model()
  old = primitive_plan(model)
  plan = replace(old, version=EXACT_EMISSION_ACTIVATION_VERSION).validate()
  assert plan.clipping_quality == EXACT_EMISSION_QUALITY
  assert plan.identification_policy == NATIVE_SHADOW_IDENTIFICATION
  bound = bind_activation_model(model, plan)
  assert activation_matches_model(plan, activation_model_from_dict(asdict(bound)))
  assert not activation_matches_model(old, bound)
  assert not activation_matches_model(plan, bind_activation_model(model, old))


def test_honda_same_integer_does_not_collapse_continuous_or_civic_state():
  from opendbc.car import DT_CTRL, rate_limit
  from opendbc.car.steering_projection import _shaped_input, float32, normalized_applied_command
  original, _, event, proposed = crossing(HONDA.HONDA_CIVIC_BOSCH)
  witness = event.witness
  branch = EncoderBranch(quality_contract=EXACT_EMISSION_QUALITY)
  # A fixed dyadic state difference lies within this real lookup's integer bin;
  # equality is still exact, never an epsilon comparison or a searched outcome.
  branch.previous = witness.previous_torque + 1 / 2**20
  branch.civic = (witness.shape[0] + 1 / 2**20,) + tuple(witness.shape[1:4])
  shape = branch.civic + tuple(witness.shape[4:])
  params = dict(witness.limits)
  expected = rate_limit(_shaped_input(witness, float32(proposed), shape), branch.previous,
    -params['STEER_DELTA_DOWN'] * DT_CTRL, params['STEER_DELTA_UP'] * DT_CTRL)
  recorded = rate_limit(witness.codec_input_torque, witness.previous_torque,
    -params['STEER_DELTA_DOWN'] * DT_CTRL, params['STEER_DELTA_UP'] * DT_CTRL)
  assert expected != recorded
  assert branch.step(event, proposed) == normalized_applied_command(witness)
  assert branch.previous == expected and branch.previous != recorded
  assert branch.civic[0] == _shaped_input(witness, float32(proposed), shape)
  assert branch.civic[1] == float32(proposed)
  # Continue the REAL original native clock. Prior local equality grants no
  # future exemption: persistent outward demand must encounter a strict veto.
  for index in range(256):
    _, future_witness = first_emission(original, witness.requested_torque)
    stamp = event.apply_ns + (index + 1) * 100_000_000
    limited = future_witness.unlimited_raw is None or future_witness.unlimited_raw != future_witness.final_raw
    future = replace(event, witness=future_witness, control_ns=stamp, state_ns=stamp, apply_ns=stamp + 1, limited=limited)
    try:
      branch.step(future, 1.)
    except ValueError as error:
      assert 'clipping regression' in str(error) or 'limiter/support' in str(error)
      return
  raise AssertionError('local packet identity bypassed future strict native quality')
