"""Ioniq6 route9b native brake withdrawal before the host's pedal sample."""
from contextlib import ExitStack

import pytest

from opendbc.can import CANPacker
from opendbc.safety.tests import test_hyundai_ioniq6_long as native_fixture
from openpilot.cereal import log, messaging
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture
from openpilot.starpilot.aol.tests.test_gm_held_native import Peers
from openpilot.starpilot.aol.transport_pause import TransportPauseFeedback
from openpilot.starpilot.aol.wire import PandaSlot, SafetyState, encode_safety

BASE = 10_000_000_000
# Actual route9b/segment2 ECAN TCS frames, counters195 and196. CRC is validated by real native RX.
TCS_BEFORE = bytes.fromhex('3389c3008803880311140000010000010f00000000000000')
TCS_BRAKE = bytes.fromhex('72f4c4008403840301140200010000000f00000000000000')


def profile(alternate):
  _, cp = native_fixture.params('lkas_alt' if alternate else 'lkas')
  assert cp is not None
  cp.safetyConfigs[0].safetyParam |= 0x0800
  return cp


def physical_full(case, packer, raw):
  case.mode(raw, stamp=BASE // 1000)
  case.safety.set_aol_test_heartbeat(True)
  for name, values in (('ACCELERATOR', {'GEAR': 5}), ('TCS', {'ACCEnable': 0}),
      ('WHEEL_SPEEDS', dict.fromkeys(('WHL_SpdFLVal', 'WHL_SpdFRVal', 'WHL_SpdRLVal', 'WHL_SpdRRVal'), 45.)),
      ('MDPS', {}), ('CRUISE_BUTTONS', {'COUNTER': 1})):
    assert case.safety.safety_rx_hook(case.packet(packer.make_can_msg(name, 1, values)))
  case.safety.aol_set_host_request(3)
  for counter, values in ((2, {'LDA_BTN': 1}), (3, {}), (4, {'CRUISE_BUTTONS': 2}), (5, {})):
    assert case.safety.safety_rx_hook(case.packet(packer.make_can_msg('CRUISE_BUTTONS', 1, dict(COUNTER=counter, **values))))
  case.safety.aol_set_host_request(3)
  assert case.safety.get_controls_allowed() and case.safety.aol_get_permission_mask() == 3


@pytest.mark.parametrize('alternate', [False, True])
def test_real_native_brake_before_host_sample_keeps_granted_lateral(alternate):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  native = native_fixture.TestHyundaiIoniq6Long(methodName='runTest')
  with OpenpilotPrefix(), ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    cleanup.callback(native.doCleanups)
    cp = profile(alternate)
    sd, _, step = case._authority_callers(True, cp=cp, genuine_events=True)
    native.setUp()
    packer = CANPacker('hyundai_canfd_generated')
    physical_full(native, packer, int(cp.safetyConfigs[0].safetyParam))
    for tick in range(10):
      native.refresh_required_rx(packer, tick + 6, (BASE + tick * 10_000_000) // 1000)
      native.safety.set_aol_test_heartbeat(True)
      native.safety.aol_set_host_request(3)
      assert native.safety.aol_get_permission_mask() == 3
      axis, state, command = step(tick)
      assert state.enabled and axis.lateralActive and command.longActive
    native.safety.set_timer((BASE + 95_000_000) // 1000)
    assert native.safety.safety_rx_hook(native.packet((0x175, TCS_BEFORE, 1)))
    before = native.safety.get_controls_allowed()
    assert before
    positive = packer.make_can_msg('SCC_CONTROL', 1, {'COUNTER': 0, 'ACCMode': 1, 'aReqRaw': 1., 'aReqValue': 1.})
    assert native.safety.safety_tx_hook(native.packet(positive))
    assert native.safety.safety_rx_hook(native.packet((0x175, TCS_BRAKE, 1)))
    mask = native.safety.aol_get_permission_mask()
    assert mask == 1
    positive = packer.make_can_msg('SCC_CONTROL', 1, {'COUNTER': 1, 'ACCMode': 1, 'aReqRaw': 1., 'aReqValue': 1.})
    assert not native.safety.safety_tx_hook(native.packet(positive))
    # Health was sampled before the permission read; parsed host brake has not arrived.
    axis, state, command = step(10, native_request_lat=True, native_request_long=True,
      allowed=True, long_allowed=False, panda_controls_allowed=before)
    assert state.enabled and axis.lateralActive and command.latActive
    assert not axis.longitudinalActive and not command.longActive
    assert not sd.aol_authority_lost and log.OnroadEvent.EventName.controlsMismatch not in sd.events.names
    axis, state, command = step(11, brake=True, native_request_lat=True, native_request_long=True,
      long_allowed=False, panda_controls_allowed=False)
    assert not state.enabled and not command.longActive
    assert log.OnroadEvent.EventName.controlsMismatch not in sd.events.names
    axis, state, command = step(12, native_request_lat=True, native_request_long=False,
      long_allowed=False, panda_controls_allowed=False)
    assert axis.lateralActive and command.latActive and not command.longActive


@pytest.mark.parametrize('alternate', [False, True])
@pytest.mark.parametrize('fault', ['lat_denied', 'native_invalid', 'rx_fault', 'relay_fault', 'missing_prior'])
def test_native_early_withdrawal_preserves_all_existing_fault_vetoes(alternate, fault):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with OpenpilotPrefix(), ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    sd, _, step = case._authority_callers(True, cp=profile(alternate), genuine_events=True)
    for tick in range(10):
      axis, state, command = step(tick)
      assert state.enabled and axis.lateralActive and command.longActive
    options = dict(native_request_lat=True, native_request_long=True, long_allowed=False, panda_controls_allowed=True)
    if fault == 'lat_denied':
      options['allowed'] = False
    elif fault == 'native_invalid':
      options['native_valid'] = False
    elif fault == 'rx_fault':
      options['panda_fault'] = True
    elif fault == 'relay_fault':
      options['relay_fault'] = True
    else:
      sd.aol_published_input_proof = None
    axis, state, command = step(10, **options)
    assert log.OnroadEvent.EventName.controlsMismatch in sd.events.names
    assert not state.enabled and not axis.lateralActive and not command.latActive and not command.longActive


def feedback_peers(cp, now, *, denied=False, controls=True):
  config = cp.safetyConfigs[0]
  slot = PandaSlot(0, 'ioniq6-unit', 28, config.safetyParam, cp.alternativeExperience, controls)
  native = SafetyState(1, True, now, now + 200_000_000, 28, config.safetyParam,
    True, not denied, True, True, 'ioniq6-unit', 'ioniq6-session', now, (slot,))
  wire = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=now)
  wire.aolSafetyWire = encode_safety(native)
  pandas = messaging.new_message('pandaStates', 1, valid=True, logMonoTime=now)
  pandas.pandaStates[0] = dict(safetyModel='hyundaiCanfd', safetyParam=config.safetyParam,
    alternativeExperience=cp.alternativeExperience, controlsAllowed=controls)
  axis = messaging.new_message('aolAxisState', valid=True, logMonoTime=now)
  axis.aolAxisState = dict(qualified=True, sequence=1 + (now - BASE) // 10_000_000, sessionId='ioniq6-session', faultSessionId='ioniq6-session',
    observedMonoTime=now, validUntilMonoTime=now + 30_000_000, sourceCarStateMonoTime=now,
    desiredLateral=True, lateralActive=True, desiredLongitudinal=True, longitudinalActive=True,
    nativeAcknowledged=True, mode='combined', faultReason='none', intentProducerSessionId='ioniq6-card', intentIntegrityEpoch=1)
  events = messaging.new_message('onroadEvents', 0, valid=True, logMonoTime=now)
  return Peers([m.as_reader() for m in (wire, pandas, axis, events)])


@pytest.mark.parametrize('alternate', [False, True])
@pytest.mark.parametrize('controls', [False, True])
def test_card_feedback_keeps_existing_latch_during_native_first_withdrawal(alternate, controls):
  cp = profile(alternate)
  feedback = TransportPauseFeedback(cp)
  cs = fixture.car_state()
  cs.cruiseState.available = cs.cruiseState.enabled = True
  assert not feedback.observe(feedback_peers(cp, BASE), cp, cs, BASE,
    latched=True, intent_producer_session='ioniq6-card')
  assert feedback.ever_lateral_active and feedback.lateral_baseline_ns == BASE
  assert feedback.session == 'ioniq6-session' and feedback.lateral_inventory
  assert feedback.availability_producer == 'ioniq6-card'
  now = BASE + 10_000_000
  assert not feedback.observe(feedback_peers(cp, now, denied=True, controls=controls), cp, cs, now,
    latched=True, intent_producer_session='ioniq6-card')
  assert not feedback.failed
  assert feedback.healthy_feedback_ns == BASE and feedback.healthy_native_ns == BASE
  assert feedback.lateral_baseline_ns == now
