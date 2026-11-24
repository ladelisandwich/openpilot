"""Route0e: actual SD/Controls constructors, synthetic typed input/native transport."""
from contextlib import ExitStack
from unittest.mock import Mock, patch

import pytest
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR
from openpilot.cereal import log
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture
from openpilot.starpilot.aol.tests.test_input_deadline_continuity import BASE, intent



def refresh_saved_settings(sd):
  # The shared caller fixture clears constructor settings to None. Exercise
  # the production saved-settings owner once instead of installing a field.
  stop = Mock()
  stop.is_set.side_effect = [False, True]
  with patch('openpilot.selfdrive.selfdrived.selfdrived.time.sleep'):
    sd.params_thread(stop)
  assert sd.aol_settings is not None and sd.aol_settings.enabled


def stopped(brake):
  state = fixture.car_state(brake=brake)
  state.vEgo = state.vEgoRaw = 0.
  state.standstill = True
  state.cruiseState.available = True
  state.cruiseState.enabled = False
  return state


@pytest.mark.parametrize('identity', (CAR.TOYOTA_HIGHLANDER_TSS2, CAR.TOYOTA_COROLLA_TSS2))
@pytest.mark.parametrize('brake', (False, True))
@pytest.mark.parametrize('expired', (False, True))
def test_stopped_pcm_off_fresh_input_after_prior_lateral_deadline(identity, brake, expired):
  with ExitStack() as cleanup:
    cleanup.enter_context(OpenpilotPrefix())
    cleanup.enter_context(patch.dict('os.environ', {'SIMULATION': '0', 'REPLAY': '0'}))
    case = fixture.IoniqAvailabilityTests(methodName='runTest')
    cleanup.callback(case.doCleanups)
    cp = CarInterface.get_params(identity, {0: {}, 1: {}, 2: {}}, [], False, False, False)
    sd, controls, drive = case._authority_callers(False, cp=cp, genuine_events=True)
    refresh_saved_settings(sd)
    for tick in (0, 1):
      axis, host, command = drive(tick, observed_state=stopped(brake), brake=brake, native_request_long=False)
      assert axis.lateralActive and command.latActive
      assert not host.enabled and not command.longActive
    prior, host, command = drive(2, offset_ns=8_437_887, state_stamp=BASE,
      intent_message=intent(BASE, 3), observed_state=stopped(brake), brake=brake, native_request_long=False)
    assert prior.lateralActive and command.latActive and not host.enabled and not command.longActive
    assert prior.validUntilMonoTime == BASE + 38_437_887
    # Exact recorded deltas at the takeover: current source9.155010ms old;
    # prior output479133ns expired. No source/permission deadline is extended.
    source = BASE + 29_762_010
    now = BASE + (38_917_020 if expired else 33_917_020)
    if expired:
      with patch('openpilot.selfdrive.controls.controlsd.time.monotonic_ns', return_value=now):
        old, _ = controls.state_control()
      assert not old.latActive and not old.longActive
    axis, host, command = drive(5, offset_ns=8_917_020 if expired else 3_917_020,
      state_stamp=source, intent_message=intent(source, 4), observed_state=stopped(brake),
      brake=brake, native_request_long=False)
    assert log.OnroadEvent.EventName.controlsMismatch not in sd.events.names, 'TOYOTA_HELD_BRAKE_EXPIRED_PRIOR_OUTPUT'
    assert axis.sourceCarStateMonoTime == source and axis.lateralActive and command.latActive
    assert not host.enabled and not host.active and not command.longActive and not axis.longitudinalActive
    assert not sd.aol_authority_lost and sd.aol_last_intent.allowedLatch


@pytest.mark.parametrize('restriction', ('native_denied', 'native_invalid', 'can_invalid', 'eps', 'cancel', 'intent_withdrawn', 'source_reversed', 'reverse'))
def test_stopped_brake_current_restrictions_still_withdraw(restriction):
  with ExitStack() as cleanup:
    cleanup.enter_context(OpenpilotPrefix())
    cleanup.enter_context(patch.dict('os.environ', {'SIMULATION': '0', 'REPLAY': '0'}))
    case = fixture.IoniqAvailabilityTests(methodName='runTest')
    cleanup.callback(case.doCleanups)
    cp = CarInterface.get_params(CAR.TOYOTA_HIGHLANDER_TSS2, {0: {}, 1: {}, 2: {}}, [], False, False, False)
    sd, controls, drive = case._authority_callers(False, cp=cp, genuine_events=True)
    refresh_saved_settings(sd)
    for tick in (0, 1):
      axis, _, command = drive(tick, observed_state=stopped(True), brake=True, native_request_long=False)
      assert axis.lateralActive and command.latActive
    prior, _, command = drive(2, offset_ns=8_437_887, state_stamp=BASE,
      intent_message=intent(BASE, 3), observed_state=stopped(True), brake=True, native_request_long=False)
    assert prior.lateralActive and command.latActive
    source = BASE - 1 if restriction == 'source_reversed' else BASE + 29_762_010
    state = stopped(True)
    if restriction == 'can_invalid':
      state.canValid = False
    elif restriction == 'eps':
      state.steerFaultPermanent = True
    elif restriction == 'cancel':
      state.buttonEvents = [fixture.car.CarState.ButtonEvent(type='cancel', pressed=True)]
    elif restriction == 'reverse':
      state.gearShifter = fixture.car.CarState.GearShifter.reverse
    axis, host, command = drive(5, offset_ns=8_917_020, state_stamp=source,
      intent_message=intent(source, 4, latched=restriction != 'intent_withdrawn'),
      observed_state=state, brake=True, native_request_long=False,
      allowed=restriction != 'native_denied', native_valid=restriction != 'native_invalid')
    assert not axis.lateralActive and not command.latActive
    assert not axis.longitudinalActive and not command.longActive and not host.enabled


def test_stopped_brake_replacement_keeps_actual_card_latch():
  from openpilot.starpilot.aol.tests.stock_lateral_test_helpers import callers

  with ExitStack() as cleanup:
    sd, _, drive, card, receive = callers(cleanup, 'toyota', True)
    refresh_saved_settings(sd)
    prior, _, command = drive(2, offset_ns=8_437_887, state_stamp=BASE,
      intent_message=intent(BASE, 3), observed_state=stopped(True), brake=True, native_request_long=False)
    assert prior.lateralActive and command.latActive
    assert receive(BASE + 8_437_887, state=stopped(True))
    source, now = BASE + 29_762_010, BASE + 38_917_020
    axis, host, command = drive(5, offset_ns=8_917_020, state_stamp=source,
      intent_message=intent(source, 4), observed_state=stopped(True), brake=True, native_request_long=False)
    assert log.OnroadEvent.EventName.controlsMismatch not in sd.events.names
    assert axis.lateralActive and command.latActive and not command.longActive and not host.enabled
    assert receive(now, state=stopped(True))
    assert card.aol_card_intent.allowed_latch
