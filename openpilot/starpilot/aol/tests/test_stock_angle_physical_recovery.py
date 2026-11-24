"""Actual host callers; native observations are explicit synthetic test inputs.

Qualification still requires the physical RX/C++ heartbeat/native TX join.
"""
from contextlib import ExitStack

import pytest

from opendbc.car import structs
from opendbc.car.hyundai.canfd_angle_aol import MODELS
from opendbc.car.hyundai.values import CAR
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.intent import read_settings
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture


@pytest.mark.parametrize('identity', sorted(MODELS))
def test_stock_angle_physical_recovery_retains_independent_intent(identity):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup, OpenpilotPrefix():
    cleanup.callback(case.doCleanups)
    saved = Params()
    saved.put('LKASButtonControl', 9, block=True)
    saved.put('MainCruiseButtonControl', 0, block=True)
    for key in ('CancelButtonControl', 'LongCancelButtonControl', 'VeryLongCancelButtonControl'):
      saved.put(key, 0, block=True)
    sd, _, step, card, receive = case._ioniq_availability_callers(
      full_op=False, initially_armed=False, stock_angle=identity)
    sd.aol_settings = read_settings(saved)
    assert card.aol_card_intent.settings == sd.aol_settings
    original_settings = {key: saved.get(key) for key in
                         ('AlwaysOnLateral', 'LKASButtonControl', 'MainCruiseButtonControl', 'DisengageOnAccelerator')}
    state = fixture.car_state()
    state.vEgoRaw = state.vEgo
    state.cruiseState.enabled = True
    # A stock SET may request lateral before the native physical latch exists.
    for tick in range(2, 32):
      axis, _, command = step(tick, observed_state=state, allowed=False,
                              native_request_lat=True, panda_controls_allowed=True)
      assert sd.enabled and not command.latActive
      receive(10_000_000_000 + tick * 10_000_000, state=state,
              axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
    # Physical LKAS arrives before the next native permission publication.
    state.buttonEvents = [structs.CarState.ButtonEvent(type='lkas', pressed=True)]
    for tick in (32, 33):
      axis, _, command = step(tick, observed_state=state, allowed=False,
                              native_request_lat=True, panda_controls_allowed=True)
      receive(10_000_000_000 + tick * 10_000_000, state=state,
              axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
      assert not command.latActive
      assert card.aol_card_intent.allowed_latch
      state.buttonEvents = []
    for tick in (34, 35):
      axis, _, command = step(tick, observed_state=state, allowed=True,
                              native_request_lat=True, panda_controls_allowed=True)
      receive(10_000_000_000 + tick * 10_000_000, state=state,
              axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
      assert card.aol_card_intent.allowed_latch
      assert axis.lateralActive and command.latActive
    state.cruiseState.enabled = False
    state.buttonEvents = [structs.CarState.ButtonEvent(type='cancel', pressed=True)]
    axis, _, command = step(36, observed_state=state, allowed=True,
                            native_request_lat=True, panda_controls_allowed=False)
    receive(10_360_000_000, state=state, axis=sd.pm.messages['aolAxisState'],
            event=sd.pm.messages['onroadEvents'])
    assert not sd.enabled and not command.longActive
    assert card.aol_card_intent.allowed_latch and axis.lateralActive and command.latActive
    assert {key: saved.get(key) for key in original_settings} == original_settings


@pytest.mark.parametrize('identity', [CAR.HYUNDAI_IONIQ_5_PE, CAR.KIA_EV9])
@pytest.mark.parametrize('veto', ['deadline', 'held', 'can', 'eps', 'native_invalid', 'panda_rx', 'relay', 'explicit_off', 'ack_then_denied'])
def test_stock_angle_pending_arm_never_renews_or_masks_restrictions(identity, veto):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup, OpenpilotPrefix():
    cleanup.callback(case.doCleanups)
    saved = Params()
    saved.put('LKASButtonControl', 9, block=True)
    saved.put('MainCruiseButtonControl', 0, block=True)
    sd, _, step, card, receive = case._ioniq_availability_callers(
      full_op=False, initially_armed=False, stock_angle=identity)
    sd.aol_settings = read_settings(saved)
    state = fixture.car_state()
    state.vEgoRaw = state.vEgo
    state.cruiseState.enabled = True

    def cycle(tick, **kwargs):
      axis, _, command = step(tick, observed_state=state, allowed=False,
                              native_request_lat=True, panda_controls_allowed=True, **kwargs)
      receive(10_000_000_000 + tick * 10_000_000, state=state,
              axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
      return command

    for tick in range(2, 32):
      cycle(tick)
    state.buttonEvents = [structs.CarState.ButtonEvent(type='lkas', pressed=True)]
    cycle(32)
    assert card.aol_card_intent.allowed_latch
    arm = card.aol_card_intent.physical_arm_ns
    state.buttonEvents = []
    if veto in ('deadline', 'held'):
      for tick in range(33, 53):
        if veto == 'held':
          state.buttonEvents = [structs.CarState.ButtonEvent(type='lkas', pressed=True)]
        command = cycle(tick)
        assert not command.latActive
        assert card.aol_card_intent.physical_arm_ns == arm
        assert card.aol_card_intent.allowed_latch == (tick < 52)
      state.buttonEvents = []
      axis, _, _ = step(53, observed_state=state, allowed=True, native_request_lat=True, panda_controls_allowed=True)
      receive(10_530_000_000, state=state, axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
      assert not card.aol_card_intent.allowed_latch
      return
    if veto == 'explicit_off':
      state.buttonEvents = [structs.CarState.ButtonEvent(type='lkas', pressed=False)]
      cycle(33)
      state.buttonEvents = [structs.CarState.ButtonEvent(type='lkas', pressed=True)]
      cycle(34)
      assert not card.aol_card_intent.allowed_latch
      return
    if veto == 'ack_then_denied':
      axis, _, command = step(33, observed_state=state, allowed=True, native_request_lat=True, panda_controls_allowed=True)
      receive(10_330_000_000, state=state, axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
      assert command.latActive and card.aol_card_intent.allowed_latch
      cycle(34)
      assert not card.aol_card_intent.allowed_latch
      return
    if veto == 'can':
      state.canValid = False
    elif veto == 'eps':
      state.steerFaultPermanent = True
    command = cycle(33, native_valid=veto != 'native_invalid', panda_fault=veto == 'panda_rx', relay_fault=veto == 'relay')
    assert not command.latActive
    assert not card.aol_card_intent.allowed_latch


@pytest.mark.parametrize('veto', ['no_physical', 'already_acknowledged', 'paused', 'disabled', 'producer', 'source_zero',
                                'source_future', 'source_age', 'axis_expired', 'axis_lease', 'event_future', 'native_expired', 'profile'])
def test_pending_stock_angle_arm_requires_original_current_proofs(veto):
  from dataclasses import replace
  from openpilot.starpilot.car.hyundai.aol import pending_stock_angle_arm

  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup, OpenpilotPrefix():
    cleanup.callback(case.doCleanups)
    saved = Params()
    saved.put('LKASButtonControl', 9, block=True)
    saved.put('MainCruiseButtonControl', 0, block=True)
    sd, _, step, card, receive = case._ioniq_availability_callers(
      full_op=False, initially_armed=False, stock_angle=CAR.HYUNDAI_IONIQ_5_PE)
    sd.aol_settings = read_settings(saved)
    state = fixture.car_state()
    state.vEgoRaw = state.vEgo
    state.cruiseState.enabled = True
    for tick in range(2, 33):
      state.buttonEvents = [structs.CarState.ButtonEvent(type='lkas', pressed=True)] if tick == 32 else []
      axis, _, _ = step(tick, observed_state=state, allowed=False, native_request_lat=True, panda_controls_allowed=True)
      receive(10_000_000_000 + tick * 10_000_000, state=state,
              axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
    owner = card.aol_card_intent
    now = 10_320_000_001
    producer = card.slc_producer_session
    cp = card.CP.as_reader().as_builder()
    acknowledged = 0
    assert pending_stock_angle_arm(cp, state, card.sm, owner, now_ns=now, producer=producer)
    if veto == 'no_physical':
      owner.physical_arm_ns = 0
    elif veto == 'already_acknowledged':
      acknowledged = owner.physical_arm_ns
    elif veto == 'paused':
      owner.pause_lateral = True
    elif veto == 'disabled':
      owner.settings = replace(owner.settings, enabled=False)
    elif veto == 'producer':
      producer = 'different-producer'
    elif veto in ('source_zero', 'source_future', 'source_age', 'axis_lease'):
      axis = card.sm['aolAxisState'].as_builder()
      if veto == 'axis_lease':
        axis.validUntilMonoTime = axis.observedMonoTime + 30_000_001
      else:
        axis.sourceCarStateMonoTime = 0 if veto == 'source_zero' else now - 30_000_001 if veto == 'source_age' else now + 1
      card.sm.data['aolAxisState'] = axis.as_reader()
    elif veto == 'axis_expired':
      card.sm.logMonoTime['aolAxisState'] = now - 30_000_001
    elif veto == 'event_future':
      card.sm.logMonoTime['onroadEvents'] = now + 1
    elif veto == 'native_expired':
      card.sm.logMonoTime['aolSafetyWire'] = now - 200_000_001
    elif veto == 'profile':
      cp.safetyConfigs[0].safetyParam ^= 4
    assert not pending_stock_angle_arm(cp, state, card.sm, owner, now_ns=now, producer=producer,
                                      acknowledged_arm_ns=acknowledged)
