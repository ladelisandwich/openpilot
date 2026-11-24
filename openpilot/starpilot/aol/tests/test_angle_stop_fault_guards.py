from contextlib import ExitStack

import pytest

from opendbc.car.hyundai.values import CAR
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.intent import read_settings
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture


@pytest.mark.parametrize('identity', [CAR.HYUNDAI_IONIQ_5_PE, CAR.KIA_EV9])
@pytest.mark.parametrize('fault', ['rolling_denial', 'invalid_native', 'invalid_can', 'permanent_eps', 'panda_rx_fault'])
def test_temporary_stop_intent_retention_cannot_mask_real_faults(identity, fault):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    with OpenpilotPrefix():
      saved = Params()
      saved.put_bool('AlwaysOnLateral', True, block=True)
      saved.put('LKASButtonControl', 9, block=True)
      saved.put('MainCruiseButtonControl', 0, block=True)
      saved.put('AolBrakePauseSpeedMps', 0.0, block=True)
      for key in ('CancelButtonControl', 'LongCancelButtonControl', 'VeryLongCancelButtonControl'):
        saved.put(key, 0, block=True)
      sd, _, step, card, receive = case._ioniq_availability_callers(full_op=False, stock_angle=identity)
      sd.aol_settings = read_settings(saved)
      assert sd.aol_settings == card.aol_card_intent.settings
      rolling = fixture.car_state()
      rolling.vEgo = rolling.vEgoRaw = 1.0
      axis, _, command = step(2, observed_state=rolling, panda_controls_allowed=False)
      assert axis.lateralActive and command.latActive
      assert receive(10_020_000_000, state=rolling, axis=sd.pm.messages['aolAxisState'],
                     event=sd.pm.messages['onroadEvents'])
      state = fixture.car_state()
      state.vEgo = state.vEgoRaw = 1.0 if fault == 'rolling_denial' else 0.0
      state.standstill = fault != 'rolling_denial'
      state.canValid = fault != 'invalid_can'
      state.steerFaultPermanent = fault == 'permanent_eps'
      axis, _, command = step(3, observed_state=state, allowed=False, native_request_lat=True,
                              native_request_long=False, panda_controls_allowed=False,
                              native_valid=fault != 'invalid_native', panda_fault=fault == 'panda_rx_fault')
      assert not axis.lateralActive and not command.latActive
      assert not axis.longitudinalActive and not command.longActive
      assert not receive(10_030_000_000, state=state, axis=sd.pm.messages['aolAxisState'],
                         event=sd.pm.messages['onroadEvents'])
      assert not card.aol_card_intent.allowed_latch
      axis, _, command = step(4, observed_state=rolling, panda_controls_allowed=False)
      assert not axis.lateralActive and not command.latActive
      assert not axis.longitudinalActive and not command.longActive
      assert not card.aol_card_intent.allowed_latch
