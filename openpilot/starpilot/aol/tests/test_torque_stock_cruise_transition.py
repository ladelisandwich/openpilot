from contextlib import ExitStack

import pytest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.hyundai.canfd_stock_aol import qualified
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.intent import read_settings
from openpilot.starpilot.aol.tests.stock_lateral_test_helpers import car_state, hkg_family_case


@pytest.mark.parametrize('identity,lka', [(CAR.GENESIS_GV70_1ST_GEN, False),
                                         (CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN, True),
                                         (CAR.KIA_EV6, True)])
@pytest.mark.parametrize('tick', [3, 7])
@pytest.mark.parametrize('gesture', ['cancel', 'brake'])
def test_configured_torque_stock_lateral_survives_stock_cruise_transition(identity, lka, tick, gesture):
  with ExitStack() as cleanup:
    with OpenpilotPrefix():
      saved = Params()
      saved.put('LKASButtonControl', 9, block=True)
      saved.put('MainCruiseButtonControl', 0, block=True)
      for key in ('CancelButtonControl', 'LongCancelButtonControl', 'VeryLongCancelButtonControl'):
        saved.put(key, 0, block=True)
      case = hkg_family_case('hkg_canfd', False)
      cleanup.callback(case.doCleanups)
      fingerprint = gen_empty_fingerprint()
      if lka:
        fingerprint[CanBus(None, fingerprint).CAM][0x50] = 16
      cp = CarInterface.get_params(identity, fingerprint, [], False, False, False)
      assert cp.steerControlType == structs.CarParams.SteerControlType.torque
      assert qualified(cp) and not cp.openpilotLongitudinalControl and cp.pcmCruise
      case.family_cp = cp.as_reader()
      sd, _, step, card, receive = case.family_callers(False)
      sd.aol_settings = read_settings(saved)
      assert sd.aol_settings == card.aol_card_intent.settings
      assert sd.aol_settings.pause_brake_mps == 0
      assert card.aol_card_intent.independent_cruise_buttons
      engaged = car_state()
      engaged.vEgoRaw = engaged.vEgo
      engaged.cruiseState.enabled = True
      axis, _, command = step(2, observed_state=engaged, panda_controls_allowed=True)
      assert sd.enabled and axis.lateralActive and command.latActive
      assert not axis.longitudinalActive and not command.longActive
      assert receive(10_020_000_000, state=engaged, axis=sd.pm.messages['aolAxisState'],
                     event=sd.pm.messages['onroadEvents'])
      state = car_state()
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
      settled = car_state(brake=gesture == 'brake')
      settled.vEgoRaw = settled.vEgo
      axis, _, command = step(tick + 1, observed_state=settled)
      assert not sd.enabled and not axis.longitudinalActive and not command.longActive
      assert axis.lateralActive and command.latActive
      assert receive(10_000_000_000 + (tick + 1) * 10_000_000, state=settled,
                     axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
      assert card.aol_card_intent.allowed_latch
