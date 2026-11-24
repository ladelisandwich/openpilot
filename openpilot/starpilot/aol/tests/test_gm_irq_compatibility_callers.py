"""Actual Card preserves diagnostic-only Bolt IRQ parity without granting authority."""
from dataclasses import replace

import pytest
from opendbc.car.gm.tests.test_bolt_pedal import params
from opendbc.car.gm.values import CAR
from opendbc.car.structs import car
from openpilot.cereal import messaging
from openpilot.starpilot.aol.runtime import current_native
from openpilot.starpilot.aol.wire import PandaSlot, SafetyState, encode_safety

NOW = 10_000_000_000


def owner():
  cp = params(CAR.CHEVROLET_BOLT_CC_2018_2021, pedal=True, camera=True)
  cp.alternativeExperience = 32
  assert cp.safetyConfigs[0].safetyParam == 157
  cs = car.CarState(canValid=True, gearShifter=car.CarState.GearShifter.drive, vEgo=20.)
  cs.cruiseState.available = True
  return cp, cs


def snapshot(cp, now, *, kind='idle', faults=0, expired=False, session='session'):
  sm = messaging.SubMaster(['aolAxisState', 'aolSafetyWire', 'pandaStates', 'onroadEvents'])
  config = cp.safetyConfigs[0]
  native = SafetyState(1, True, now, now + 200_000_000, config.safetyModel.raw, config.safetyParam,
                      False, False, False, False, 'panda', session, now,
                      (PandaSlot(0, 'panda', config.safetyModel.raw, config.safetyParam, 32, False, faults=faults),))
  if expired:
    native = replace(native, observedMonoTime=now-201_000_000, validUntilMonoTime=now-1)
  wire = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=native.observedMonoTime)
  wire.aolSafetyWire = encode_safety(native)
  pandas = messaging.new_message('pandaStates', 1, valid=True, logMonoTime=now)
  pandas.pandaStates[0].safetyModel = config.safetyModel
  pandas.pandaStates[0].safetyParam = config.safetyParam
  pandas.pandaStates[0].alternativeExperience = 32
  if faults:
    pandas.pandaStates[0].faults = [name for bit, name in ((1, 'relayMalfunction'), (4, 'interruptRateCan1'), (8, 'interruptRateCan2')) if faults & bit]
  events = messaging.new_message('onroadEvents', int(kind == 'soft'), valid=True, logMonoTime=now)
  if kind == 'soft':
    events.onroadEvents[0].name = 'commIssueAvgFreq'
    events.onroadEvents[0].softDisable = True
  axis = messaging.new_message('aolAxisState', valid=True, logMonoTime=now)
  a = axis.aolAxisState
  a.sessionId = a.faultSessionId = session
  a.sequence = (now-NOW)//10_000_000 + 1
  a.sourceCarStateMonoTime = a.observedMonoTime = a.faultEventMonoTime = now
  a.validUntilMonoTime = now + 30_000_000
  a.intentProducerSessionId = 'producer'
  a.intentIntegrityEpoch = 1
  a.qualified = True
  a.nativeAcknowledged = kind != 'soft'
  a.desiredLateral = kind == 'request'
  a.faultReason = 'availabilityPause' if kind == 'soft' else 'none'
  sm.update_msgs(now/1e9, [wire.as_reader(), pandas.as_reader(), events.as_reader(), axis.as_reader()])
  return sm

@pytest.mark.parametrize("faults,admitted", [(4, True), (8, True), (12, True), (5, False), (9, False), (13, False)])
def test_actual_card_diagnostic_faults_preserve_only_qualified_intent(faults, admitted):
  """Real parser/Card honors the diagnostic policy; no native authority is synthesized."""
  import os
  from contextlib import ExitStack
  from unittest.mock import patch
  from openpilot.common.params import Params
  from openpilot.common.prefix import OpenpilotPrefix
  from openpilot.selfdrive.car import card as module
  from openpilot.starpilot.aol.tests.test_gm import TestGmAol
  from openpilot.starpilot.aol.tests.test_gm_held_native import parse
  from openpilot.starpilot.aol.wire import decode_intent

  with OpenpilotPrefix(), ExitStack() as cleanup, patch.dict(os.environ, {'SIMULATION': '1', 'AOL_REPLAY_RUNTIME': '0'}):
    saved = Params()
    saved.put_bool('AlwaysOnLateral', True, block=True)
    saved.put_bool('AlphaLongitudinalEnabled', True, block=True)
    cp, _ = owner()
    card = TestGmAol.card(cp, saved)
    cleanup.callback(card.vehicle_startup.close)
    card.ci_initialized = True
    card.publish_sendcan = lambda *args, **kwargs: None
    published = {}
    card.pm.send = lambda name, message: published.__setitem__(name, message.as_reader())
    for tick in range(20):
      stamp = NOW-(20-tick)*10_000_000
      rows = parse(card, stamp, counter=tick % 4, packets_only=True)
      card.CI.update([(stamp, rows)])

    def cycle(tick, kind):
      now = NOW+tick*10_000_000
      peers = snapshot(card.CP, now, kind=kind, faults=faults)
      messages = []
      for name in ('aolSafetyWire', 'pandaStates', 'aolAxisState', 'onroadEvents'):
        size = len(peers[name]) if name in ('pandaStates', 'onroadEvents') else 0 if name == 'aolSafetyWire' else None
        message = messaging.new_message(name, size, valid=True, logMonoTime=now)
        setattr(message, name, peers[name])
        if name == 'aolAxisState':
          message.aolAxisState.intentProducerSessionId = card.slc_producer_session
        messages.append(message)
      if kind == 'soft':
        messages[-1].logMonoTime = NOW+10_000_000
        messages[-2].aolAxisState.faultEventMonoTime = NOW+10_000_000
      if kind == 'critical':
        events = messaging.new_message('onroadEvents', 1, valid=True, logMonoTime=now)
        events.onroadEvents[0].name = 'controlsMismatch'
        events.onroadEvents[0].immediateDisable = True
        messages[-1] = events
        messages[-2].aolAxisState.faultReason = 'critical'
      control = messaging.new_message('carControl', valid=True, logMonoTime=now)
      card.sm.update_msgs(now/1e9, [m.as_reader() for m in messages]+[control.as_reader()])
      rows = parse(card, now, counter=tick % 4, packets_only=True)
      batch = messaging.log_from_bytes(module.can_list_to_can_capnp(rows)).as_builder()
      batch.logMonoTime = now
      with patch.object(module, 'REPLAY', False), \
           patch.object(module.messaging, 'drain_sock_raw', return_value=[batch.to_bytes()]), \
           patch.object(module.time, 'monotonic_ns', return_value=now), \
           patch.object(module.time, 'monotonic', return_value=now/1e9), \
           patch.object(module.time, 'clock_gettime_ns', return_value=now), patch.object(card.sm, 'update'):
        card.step()
      assert card.CS_prev.canValid and not card.CS_prev.canTimeout
      assert not card.last_actuators_output.torqueOutputCan
      result = decode_intent(published['aolIntentWire'].aolIntentWire)
      assert result is not None
      return result

    for tick in range(3):
      result = cycle(tick, 'idle')
      if admitted:
        assert result.lateralArmed and result.allowedLatch
      # A pending host intent is not authority: mixed faults must prevent the
      # native receipt from reaching an actuator decision, even before feedback.
      native = current_native(card.sm, card.CP, now_ns=NOW+tick*10_000_000, axis_session_id='session')
      assert (native is not None) == admitted
      assert not card.last_actuators_output.torqueOutputCan
