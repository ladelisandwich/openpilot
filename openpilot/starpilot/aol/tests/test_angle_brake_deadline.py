from contextlib import ExitStack

import pytest

from opendbc.car.hyundai.canfd_angle_aol import MODELS
from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture
from openpilot.starpilot.aol.intent import read_settings


@pytest.mark.parametrize('identity', sorted(MODELS))
@pytest.mark.parametrize('pause_speed,speed', [(0., 3.), (30., 3.), (30., 30.), (30., 31.)])
def test_fresh_first_brake_after_expired_angle_output_preserves_saved_pause(identity, pause_speed, speed):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    with OpenpilotPrefix():
      saved = Params()
      saved.put('PauseAOLOnBrake', pause_speed, block=True)
      saved.put('LKASButtonControl', 9, block=True)
      saved.put('MainCruiseButtonControl', 0, block=True)
      sd, _, step, card, receive = case._ioniq_availability_callers(full_op=False, stock_angle=identity)
      sd.aol_settings = read_settings(saved)
      assert sd.aol_settings == card.aol_card_intent.settings
      before, _, command = step(2)
      assert before.lateralActive and command.latActive
      state = fixture.car_state(brake=True)
      state.vEgo = state.vEgoRaw = speed
      expected = speed >= pause_speed
      axis, _, command = step(6, observed_state=state, pause=not expected,
                              allowed=expected, native_request_lat=expected)
      assert axis.lateralActive == command.latActive == expected
      assert not axis.longitudinalActive and not command.longActive and not sd.enabled
      assert not sd.aol_authority_lost
      assert log.OnroadEvent.EventName.controlsMismatch not in sd.events.names
      assert receive(10_060_000_000, state=state, axis=sd.pm.messages['aolAxisState'],
                     event=sd.pm.messages['onroadEvents'])
      assert card.aol_card_intent.allowed_latch
      assert float(saved.get('PauseAOLOnBrake')) == pause_speed


@pytest.mark.parametrize('identity', ['HYUNDAI_IONIQ_5_PE', 'KIA_EV9', 'GENESIS_GV70_2026'])
@pytest.mark.parametrize('restriction', ['native_denied', 'native_invalid', 'intent_withdrawn', 'can_invalid',
                                       'permanent_eps', 'reverse', 'paused_intent', 'old_source'])
def test_fresh_brake_zero_never_replaces_missing_current_authority(identity, restriction):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    with OpenpilotPrefix():
      saved = Params()
      saved.put('PauseAOLOnBrake', 0., block=True)
      saved.put('LKASButtonControl', 9, block=True)
      saved.put('MainCruiseButtonControl', 0, block=True)
      sd, _, step, _, _ = case._ioniq_availability_callers(full_op=False, stock_angle=identity)
      sd.aol_settings = read_settings(saved)
      initial, _, command = step(2)
      assert initial.lateralActive and command.latActive
      state = fixture.car_state(brake=True)
      state.vEgo = state.vEgoRaw = 3.
      if restriction == 'can_invalid':
        state.canValid = False
      elif restriction == 'permanent_eps':
        state.steerFaultPermanent = True
      elif restriction == 'reverse':
        state.gearShifter = 'reverse'
      kwargs = {'observed_state': state, 'allowed': restriction != 'native_denied',
                'native_valid': restriction != 'native_invalid', 'latched': restriction != 'intent_withdrawn',
                'pause': restriction == 'paused_intent'}
      if restriction == 'old_source':
        kwargs['state_stamp'] = 10_020_000_000
      axis, _, command = step(6, **kwargs)
      assert not axis.lateralActive and not command.latActive
      assert not axis.longitudinalActive and not command.longActive and not sd.enabled
      assert float(saved.get('PauseAOLOnBrake')) == 0.
