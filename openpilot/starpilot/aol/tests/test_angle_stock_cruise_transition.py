from contextlib import ExitStack

import pytest

from opendbc.car import structs
from opendbc.car.hyundai.canfd_angle_aol import MODELS
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.intent import read_settings
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture


@pytest.mark.parametrize('identity', sorted(MODELS))
@pytest.mark.parametrize('tick', [3, 7])
@pytest.mark.parametrize('gesture', ['cancel', 'brake'])
def test_current_independent_angle_survives_stock_cruise_status_lag(identity, tick, gesture):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    with OpenpilotPrefix():
      saved = Params()
      saved.put('LKASButtonControl', 9, block=True)
      saved.put('MainCruiseButtonControl', 0, block=True)
      for key in ('CancelButtonControl', 'LongCancelButtonControl', 'VeryLongCancelButtonControl'):
        saved.put(key, 0, block=True)
      sd, _, step, card, receive = case._ioniq_availability_callers(full_op=False, stock_angle=identity)
      sd.aol_settings = read_settings(saved)
      assert sd.aol_settings == card.aol_card_intent.settings
      assert sd.aol_settings.pause_brake_mps == 0
      engaged = fixture.car_state()
      engaged.vEgoRaw = engaged.vEgo
      engaged.cruiseState.enabled = True
      axis, _, command = step(2, observed_state=engaged, panda_controls_allowed=True)
      assert sd.enabled and axis.lateralActive and command.latActive
      assert not axis.longitudinalActive and not command.longActive
      assert receive(10_020_000_000, state=engaged, axis=sd.pm.messages['aolAxisState'],
                     event=sd.pm.messages['onroadEvents'])
      state = fixture.car_state()
      state.vEgoRaw = state.vEgo
      state.cruiseState.enabled = True
      if gesture == 'cancel':
        state.buttonEvents = [structs.CarState.ButtonEvent(type='cancel', pressed=True)]
      else:
        state.brakePressed = True
      axis, _, command = step(tick, observed_state=state, panda_controls_allowed=gesture == 'cancel')
      assert sd.enabled == (gesture == 'cancel')
      assert not axis.longitudinalActive and not command.longActive
      assert axis.lateralActive and command.latActive
      assert receive(10_000_000_000 + tick * 10_000_000, state=state, axis=sd.pm.messages['aolAxisState'],
                     event=sd.pm.messages['onroadEvents'])
      assert card.aol_card_intent.allowed_latch
      settled = fixture.car_state(brake=gesture == 'brake')
      settled.vEgoRaw = settled.vEgo
      axis, _, command = step(tick + 1, observed_state=settled)
      assert not sd.enabled and not axis.longitudinalActive and not command.longActive
      assert axis.lateralActive and command.latActive
      assert receive(10_000_000_000 + (tick + 1) * 10_000_000, state=settled,
                     axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
      assert card.aol_card_intent.allowed_latch
