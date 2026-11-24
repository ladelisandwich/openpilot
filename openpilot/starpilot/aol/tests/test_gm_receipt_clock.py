"""Route19 causal host regressions; time and receive transport are declared fixtures."""
from contextlib import ExitStack
from unittest.mock import patch
import pytest
from opendbc.car.gm.tests.test_volt_sdgm_control import sdgm_params
from openpilot.cereal import log, messaging
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.starpilot.aol.runtime import AxisDecision
from openpilot.starpilot.aol.intent import read_settings
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture
from openpilot.starpilot.aol.tests.test_input_deadline_continuity import BASE, intent
from openpilot.starpilot.car.gm.steering_companion import SteeringCompanion


@pytest.mark.parametrize('c9', [False, True])
@pytest.mark.parametrize('native_ready', [False, True])
def test_cold_never_active_sdgm_source_expiry_is_not_take_control(c9, native_ready):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with OpenpilotPrefix(), ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    sd, _, step = case._authority_callers(False, cp=sdgm_params(alpha=True, brake_c9=c9), genuine_events=True)
    sd.aol_settings = read_settings(sd.params)
    sd.initialized = False
    cleanup.enter_context(patch('openpilot.selfdrive.selfdrived.selfdrived.SIMULATION', False))
    cleanup.enter_context(patch.object(sd.sm, 'all_checks', return_value=False))
    cs = fixture.car_state()
    cs.vEgo = 5.36448  #12mph: startup report, not a low-speed steering exemption.
    now = [BASE + 29_900_000]
    def finish_environment(_cs):
      now[0] = BASE + 32_985_260
    with patch('openpilot.selfdrive.selfdrived.selfdrived.time.monotonic_ns', side_effect=lambda: now[0]), \
         patch.object(sd, 'update_conditional_mode', side_effect=finish_environment):
      axis, _, command = step(2, observed_state=cs, state_stamp=BASE,
        intent_message=intent(BASE,3), native_request_long=False, native_valid=native_ready)
    assert not sd.initialized and not getattr(sd, 'aol_ever_active', False)
    assert not axis.lateralActive and not axis.longitudinalActive and not command.latActive and not command.longActive
    assert not sd.aol_authority_lost and log.OnroadEvent.EventName.controlsMismatch not in sd.events.names


@pytest.mark.parametrize('c9', [False, True])
@pytest.mark.parametrize('condition', ['healthy','genuine_future','old_after_drain','faulty_latest','invalid_latest','missing_exact','wrong_source'])
def test_steering_receipt_published_during_drain_is_checked_after_drain(c9, condition):
  sd = SelfdriveD.__new__(SelfdriveD)
  sd.CP = sdgm_params(alpha=True, brake_c9=c9)
  sd.events = Events()
  sd.aol_car_state_log_ns = BASE
  sd.conditional_car_state_valid = True
  sd.active = sd.enabled = False
  sd.aol_axis_decision = AxisDecision(desired_lateral=True, lateral_active=True, native_acknowledged=True)
  sd.steering_authority_unavailable = False
  cs = fixture.car_state()
  source = messaging.new_message('starpilotCarState', valid=True, logMonoTime=BASE)
  source.starpilotCarState.sourceCarStateMonoTime = BASE
  later = messaging.new_message('starpilotCarState', valid=True, logMonoTime=BASE+2_000_000)
  later.starpilotCarState.sourceCarStateMonoTime = BASE+2_000_000
  if condition == 'old_after_drain':
    later.logMonoTime = BASE+32_000_000
    later.starpilotCarState.sourceCarStateMonoTime = later.logMonoTime
  if condition == 'faulty_latest':
    later.starpilotCarState.lateralAuthorityUnavailable = True
  if condition == 'invalid_latest':
    later.valid = False
  if condition == 'wrong_source':
    later.starpilotCarState.sourceCarStateMonoTime -= 1
  sd.sm = messaging.SubMaster(['starpilotCarState'])
  sd.sm.update_msgs(BASE/1e9, [source.as_reader()])
  sock = object()
  sd.steering_companion = SteeringCompanion(sock)
  queue = ([source.as_reader()] if condition != 'missing_exact' else []) + [later.as_reader()]
  now = [BASE+1_000_000]
  def deliver(received):
    assert received is sock
    if not queue:
      return None
    message = queue.pop(0)
    if condition != 'genuine_future':
      now[0] = max(now[0], int(message.logMonoTime)+100_000)
    return message
  with patch('openpilot.selfdrive.selfdrived.selfdrived.time.monotonic_ns', side_effect=lambda:now[0]), \
       patch('openpilot.starpilot.car.gm.steering_companion.messaging.recv_one_or_none', side_effect=deliver):
    sd._update_steering_authority(cs)
  assert not queue
  assert sd.steering_authority_unavailable == (condition != 'healthy')
  assert (log.OnroadEvent.EventName.controlsMismatch in sd.events.names) == (condition != 'healthy')
  if condition == 'healthy':
    assert BASE in sd.steering_companion.samples


@pytest.mark.parametrize('c9', [False, True])
@pytest.mark.parametrize('condition', ['cold', 'initialized', 'prior_aol', 'ordinary_enabled', 'immediate_fault', 'foreign'])
def test_expiry_of_known_main_intent_does_not_invent_prior_actuation(c9, condition):
  from openpilot.starpilot.aol.wire import decode_intent
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with OpenpilotPrefix(), ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    sd, _, _ = case._authority_callers(False, cp=sdgm_params(alpha=True, brake_c9=c9), genuine_events=True)
    sd.initialized = condition == 'initialized'
    sd.aol_ever_active = condition == 'prior_aol'
    sd.enabled = condition == 'ordinary_enabled'
    sd.aol_last_intent = decode_intent(intent(BASE,3).aolIntentWire)
    sd.aol_car_state_log_ns = BASE
    sd.conditional_car_state_valid = True
    sd.events.clear()
    sd.events.add(log.OnroadEvent.EventName.selfdriveInitializing)
    if condition == 'immediate_fault':
      sd.events.add(log.OnroadEvent.EventName.relayMalfunction)
    if condition == 'foreign':
      sd.CP = sd.CP.as_builder()
      sd.CP.brand = 'hyundai'
    cs = fixture.car_state()
    cs.vEgo = 5.36448
    sd._expire_axis_source(cs, BASE+32_985_260)
    assert not sd.aol_axis_decision.lateral_active and not sd.aol_axis_decision.longitudinal_active
    assert sd.aol_car_state_log_ns == 0 and not sd.conditional_car_state_valid
    assert (log.OnroadEvent.EventName.controlsMismatch in sd.events.names) == (condition != 'cold')
    assert sd.aol_authority_lost == (condition != 'cold')
