from contextlib import ExitStack

import pytest

from opendbc.car.hyundai.canfd_angle_aol import MODELS
from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture
from openpilot.starpilot.aol.tests.test_input_deadline_continuity import (
  test_fresh_source_replaces_expired_output_without_restoring_disengagement as _fresh_source,
  test_late_carstate_during_environment_work_reaches_final_owners_once as _restrictive_source,
)


@pytest.mark.parametrize('timing', ['timely', 'late_poll', 'processing_crosses_deadline'])
def test_stock_angle_fresh_source_keeps_independent_lateral_with_stock_cruise_off(timing):
  _fresh_source('stock_angle_gas', timing)


@pytest.mark.parametrize('pulse', ['stock_angle_gas_invalid_envelope', 'stock_angle_gas_permanent_eps'])
def test_stock_angle_fresh_source_still_observes_invalid_envelope_and_eps_fault(pulse):
  _restrictive_source(pulse)


@pytest.mark.parametrize('restriction', ['cancel', 'eps', 'can_invalid', 'native_denied', 'native_invalid', 'intent_withdrawn'])
def test_stock_angle_expired_output_never_renews_through_restrictions(restriction):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    with OpenpilotPrefix():
      sd, _, step, _, _ = case._ioniq_availability_callers(full_op=False, stock_angle=True)
      initial, _, command = step(2, gas=True)
      assert initial.lateralActive and command.latActive
      state = fixture.car_state()
      state.vEgoRaw = state.vEgo
      if restriction == 'cancel':
        state.buttonEvents = [fixture.car.CarState.ButtonEvent(type='cancel', pressed=True)]
      elif restriction == 'eps':
        state.steerFaultPermanent = True
      elif restriction == 'can_invalid':
        state.canValid = False
      axis, _, command = step(6, observed_state=state, gas=True, allowed=restriction != 'native_denied',
                              native_valid=restriction != 'native_invalid', latched=restriction != 'intent_withdrawn')
      assert not axis.lateralActive and not command.latActive
      assert not axis.longitudinalActive and not command.longActive and not sd.enabled


def test_stock_angle_explicit_lateral_pause_keeps_latch_beyond_native_ack_timeout():
  from openpilot.starpilot.aol.intent import PAUSE_LATERAL

  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    with OpenpilotPrefix():
      sd, _, step, card, receive = case._ioniq_availability_callers(full_op=False, stock_angle=True)
      card.aol_card_intent._perform(PAUSE_LATERAL)
      for tick in range(2, 37):
        pause = tick < 36
        if not pause:
          card.aol_card_intent._perform(PAUSE_LATERAL)
        axis, _, command = step(tick, pause=pause, allowed=not pause, native_request_lat=not pause)
        assert axis.lateralActive == command.latActive == (not pause)
        assert not axis.longitudinalActive and not command.longActive and not sd.enabled
        assert receive(10_000_000_000 + tick * 10_000_000,
                       axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
        assert card.aol_card_intent.allowed_latch


@pytest.mark.parametrize('pause_speed', [0., 30.])
@pytest.mark.parametrize('identity', sorted(MODELS))
def test_moving_stock_angle_brake_keeps_latch_and_respects_saved_pause(pause_speed, identity):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    with OpenpilotPrefix():
      params = Params()
      params.put('PauseAOLOnBrake', pause_speed, block=True)
      sd, _, step, card, receive = case._ioniq_availability_callers(full_op=False, stock_angle=identity)
      assert card.CP.alternativeExperience == 32
      assert not card.CP.openpilotLongitudinalControl
      for tick in range(2, 37):
        brake = tick < 36
        state = fixture.car_state(brake=brake)
        state.vEgo = state.vEgoRaw = 3.
        expected = not brake or pause_speed == 0.
        axis, _, command = step(tick, observed_state=state, allowed=expected, native_request_lat=expected)
        assert axis.lateralActive == command.latActive == expected
        assert not axis.longitudinalActive and not command.longActive and not sd.enabled
        assert not sd.aol_authority_lost
        assert log.OnroadEvent.EventName.controlsMismatch not in sd.events.names
        assert receive(10_000_000_000 + tick * 10_000_000, state=state,
                       axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
        assert card.aol_card_intent.allowed_latch
      assert float(params.get('PauseAOLOnBrake')) == pause_speed
