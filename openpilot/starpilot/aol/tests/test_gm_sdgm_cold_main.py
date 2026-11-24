"""Recorded SDGM cold MAIN feedback: event occurrence and output use distinct leases.

Native all-OFF peers here are explicit protocol fixtures, not permission grants.
The companion joined test earns permission through current GM native RX/CPP.
"""
from contextlib import ExitStack
from dataclasses import replace
import os
from unittest.mock import patch

import pytest
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.gm.tests.test_volt_sdgm_control import sdgm_params
from opendbc.car.gm.values import DBC
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car import card as module
from openpilot.starpilot.aol.tests import test_gm as fixtures
from openpilot.starpilot.aol.tests.test_gm_held_native import peers
from openpilot.starpilot.aol.tests.test_gm_startup_main import parsed_packets
from openpilot.starpilot.aol.wire import PandaSlot, SafetyState

NOW = 10_000_000_000


@pytest.mark.parametrize('braking', (False, True))
@pytest.mark.parametrize('variation', ('none', 'event_expired', 'source_expired', 'native_expired',
  'critical', 'producer', 'epoch', 'session', 'native_request', 'main_off', 'settings_off'))
def test_sdgm_initial_main_waits_on_original_soft_occurrence(braking, variation):
  with OpenpilotPrefix(), ExitStack() as cleanup, patch.dict(os.environ, {'SIMULATION': '1', 'AOL_REPLAY_RUNTIME': '0'}):
    saved = Params()
    saved.put_bool('AlwaysOnLateral', True, block=True)
    card = fixtures.TestGmAol.card(sdgm_params(alpha=True), saved)
    cleanup.callback(card.vehicle_startup.close)
    assert card.CP.safetyConfigs[0].safetyParam == 0x5007
    packer = CANPacker(DBC[card.CP.carFingerprint][Bus.pt])
    card.ci_initialized = True
    card.publish_sendcan = lambda *args, **kwargs: None

    def packets(now, tick, main=True):
      rows = parsed_packets(card.CP, packer, now, tick, 8., main)
      if braking:
        from opendbc.car.gm.tests.test_volt_camera_control import feed_camera
        from types import SimpleNamespace
        _, extra = feed_camera(SimpleNamespace(update=lambda _: None), packer, now,
          counter=tick % 4, speed=8., brake=True, active=False, eps_status=1)
        addresses = {0xBE, 0xC9, 0xF1}
        rows = [row for row in rows if row[0] not in addresses]
        rows += [row for row in extra if row[0] in addresses and row[0] not in (0xBE, 0xC9)]
        rows.append(packer.make_can_msg('ECMAcceleratorPos', 0, {'BrakePedalPos': 10}))
        rows.append(packer.make_can_msg('ECMEngineStatus', 0, {'CruiseMainOn': int(main), 'BrakePressed': 1}))
      return rows

    for tick in range(20):
      stamp = NOW - (20 - tick) * 10_000_000
      card.CI.update([(stamp, packets(stamp, tick))])

    def cycle(tick, age=None, mutate=False):
      now = NOW + tick * 10_000_000
      source = now - 1
      config = card.CP.safetyConfigs[0]
      slot = PandaSlot(0, 'sdgm-idle', config.safetyModel.raw, config.safetyParam, 32, False)
      native = SafetyState(1, True, now, now + 200_000_000, config.safetyModel.raw, config.safetyParam,
        False, False, False, False, 'sdgm-idle', 'gm-session',
        sourcePandaStatesMonoTime=now, pandaInventory=(slot,))
      if mutate and variation == 'native_expired':
        native = replace(native, observedMonoTime=now - 201_000_000,
                         validUntilMonoTime=now - 1, sourcePandaStatesMonoTime=now - 201_000_000)
      if mutate and variation == 'native_request':
        native = replace(native, requestedLateral=True)
      incoming, _ = peers(card.CP, card.slc_producer_session, now, controls=False,
                           native=native, sequence=tick + 1, long_active=False)
      axis = incoming.messages['aolAxisState'].as_builder()
      a = axis.aolAxisState
      a.sourceCarStateMonoTime = now - 31_000_000 if mutate and variation == 'source_expired' else source
      a.nativeAcknowledged = age is None
      a.desiredLateral = a.lateralActive = a.desiredLongitudinal = a.longitudinalActive = False
      a.faultReason = 'none' if age is None else 'availabilityPause'
      event_ns = now if age is None else now - age
      if mutate and variation == 'event_expired':
        event_ns = now - 1_500_000_001
      a.faultEventMonoTime = event_ns
      if mutate and variation == 'producer':
        a.intentProducerSessionId = 'foreign'
      if mutate and variation == 'epoch':
        a.intentIntegrityEpoch += 1
      if mutate and variation == 'session':
        a.sessionId = a.faultSessionId = 'foreign'
      events = messaging.new_message('onroadEvents', (1 + int(braking)) if age is not None else 0,
                                     valid=True, logMonoTime=event_ns)
      if age is not None:
        events.onroadEvents[0].name = 'commIssueAvgFreq'
        events.onroadEvents[0].softDisable = True
        if braking:
          events.onroadEvents[1].name = 'pedalPressed'
          events.onroadEvents[1].userDisable = True
        if mutate and variation == 'critical':
          events.onroadEvents[0].name = 'controlsMismatch'
          events.onroadEvents[0].softDisable = False
          events.onroadEvents[0].immediateDisable = True
          a.faultReason = 'critical'
      if mutate and variation == 'settings_off':
        saved.put_bool('AlwaysOnLateral', False, block=True)
        card.aol_card_intent.settings = module.read_settings(saved)
      control = messaging.new_message('carControl', valid=True, logMonoTime=now)
      card.sm.update_msgs(now / 1e9, [incoming.messages[k] for k in ('aolSafetyWire', 'pandaStates')] +
                           [axis.as_reader(), events.as_reader(), control.as_reader()])
      rows = packets(now, tick, main=not (mutate and variation == 'main_off'))
      batch = messaging.log_from_bytes(module.can_list_to_can_capnp(rows)).as_builder()
      batch.logMonoTime = now
      with patch.object(module, 'REPLAY', False), \
           patch.object(module.messaging, 'drain_sock_raw', return_value=[batch.to_bytes()]), \
           patch.object(module.time, 'monotonic_ns', return_value=now), \
           patch.object(module.time, 'monotonic', return_value=now / 1e9), \
           patch.object(module.time, 'clock_gettime_ns', return_value=now), patch.object(card.sm, 'update'):
        card.step()
      assert card.CS_prev.canValid and not card.CS_prev.canTimeout
      assert card.CS_prev.brakePressed == braking
      assert not card.last_actuators_output.torqueOutputCan

    cycle(0)  # MAIN already true; no button event or Long engagement.
    assert card.aol_card_intent.allowed_latch
    healthy = card.aol_transport_feedback.healthy_feedback_ns
    # Hold ONE original event occurrence while real parser/Card samples advance
    # at100Hz; independently fresh native/source messages never restamp it.
    for tick in range(1, 38):
      age = (tick + 3) * 10_000_000
      cycle(tick, age, mutate=variation != 'none')
      if variation != 'none':
        assert not card.aol_card_intent.allowed_latch
        return
      assert card.aol_card_intent.allowed_latch, 'cold MAIN intent was revoked by a current SOFT occurrence'
      assert card.aol_transport_feedback.healthy_feedback_ns == healthy
      assert not card.aol_transport_feedback.failed
    cycle(38)  # Healthy idle reply after a cold wait longer than native200ms.
    assert card.aol_card_intent.allowed_latch and not card.aol_transport_feedback.failed
