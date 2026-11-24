"""Healthy GM MAIN restores independent LAT without requiring ordinary engagement."""
import os
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.gm.tests.test_ascm_intercept import params as ascm_params
from opendbc.car.gm.values import CAR, DBC
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car import card as module
from openpilot.starpilot.aol.tests import test_gm as gm_fixtures
from openpilot.starpilot.aol.tests.test_gm_held_native import peers
from openpilot.starpilot.aol.tests.test_gm_startup_main import parsed_packets
from openpilot.starpilot.aol.runtime import decide_axes
from openpilot.starpilot.aol.wire import PandaSlot, SafetyState, decode_safety

NOW = 10_000_000_000


@pytest.mark.parametrize('alpha', (False, True))
@pytest.mark.parametrize('veto', (None, 'fault', 'stale', 'native', 'main_off', 'pre_enabled', 'startup_active', 'held'))
def test_actual_card_resolved_fault_restores_main_without_ordinary_engagement(alpha, veto):
  with OpenpilotPrefix(), patch.dict(os.environ, {'SIMULATION': '1', 'AOL_REPLAY_RUNTIME': '0'}):
    saved = Params()
    saved.put_bool('AlwaysOnLateral', True, block=True)
    saved.put_bool('AlphaLongitudinalEnabled', alpha, block=True)
    saved.put_bool('VoltOnePedalMode', alpha, block=True)
    saved.put_bool('GMAutoHold', alpha, block=True)
    cp = ascm_params(CAR.CHEVROLET_VOLT_ASCM, sascm=alpha, accelerator=True, radar=True, alpha=alpha)
    card = gm_fixtures.TestGmAol.card(cp, saved)
    assert card.CP.alternativeExperience == 32
    if alpha:
      assert card.CP.safetyConfigs[0].safetyParam == 0xD114
    packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
    for tick in range(20):
      stamp = NOW - (20 - tick) * 10_000_000
      card.CI.update([(stamp, parsed_packets(cp, packer, stamp, tick, 8., True))])
    initial_cp = card.CP.to_dict()

    def cycle(tick, *, enabled=False, active=False, fault=False, main=True, brake=False, stale=False, native_rejection=False):
      now = NOW + tick * 10_000_000
      config = card.CP.safetyConfigs[0]
      slot = PandaSlot(0, 'gm-unit', config.safetyModel.raw, config.safetyParam, 32, False)
      native = SafetyState(1, True, now, now + 200_000_000, config.safetyModel.raw, config.safetyParam,
                           False, False, native_rejection, False, 'gm-unit', 'gm-session',
                           sourcePandaStatesMonoTime=now, pandaInventory=(slot,))
      incoming, _ = peers(card.CP, card.slc_producer_session, now, controls=False,
                          native=native, sequence=tick + 1, long_active=False)
      axis = incoming.messages['aolAxisState'].as_builder()
      a = axis.aolAxisState
      a.sourceCarStateMonoTime = now - 1
      a.desiredLateral = a.lateralActive = a.desiredLongitudinal = a.longitudinalActive = False
      events = messaging.new_message('onroadEvents', int(fault), valid=True, logMonoTime=now)
      if fault:
        events.onroadEvents[0].name = 'controlsMismatch'
        events.onroadEvents[0].immediateDisable = True
        a.faultReason = 'critical'
      control = messaging.new_message('carControl', valid=True, logMonoTime=now)
      control.carControl.enabled = enabled
      control.carControl.latActive = control.carControl.longActive = active
      sd = messaging.new_message('selfdriveState', valid=True, logMonoTime=now - 31_000_000 if stale else now)
      sd.selfdriveState.enabled, sd.selfdriveState.active = enabled, active
      sd.selfdriveState.state = 'enabled' if active else 'preEnabled' if enabled else 'disabled'
      card.sm.update_msgs(now / 1e9, [incoming.messages[k] for k in ('aolSafetyWire', 'pandaStates')] +
                          [axis.as_reader(), events.as_reader(), control.as_reader(), sd.as_reader()])
      card.CC_prev = control.carControl.as_reader()
      packets = parsed_packets(cp, packer, now, tick, 8., main)
      if brake:
        addresses = {packer.make_can_msg(name, 0, {})[0] for name in ('ECMEngineStatus', 'ECMAcceleratorPos')}
        packets = [packet for packet in packets if not (packet[0] in addresses and packet[2] == 0)]
        packets += [packer.make_can_msg('ECMEngineStatus', 0, {'CruiseMainOn': int(main), 'BrakePressed': 1}),
                    packer.make_can_msg('ECMAcceleratorPos', 0, {'BrakePedalPos': 30})]
      batch = messaging.log_from_bytes(module.can_list_to_can_capnp(packets)).as_builder()
      batch.logMonoTime = now
      with patch.object(module, 'REPLAY', False), \
           patch.object(module.messaging, 'drain_sock_raw', return_value=[batch.to_bytes()]), \
           patch.object(module.time, 'monotonic_ns', return_value=now), \
           patch.object(module.time, 'monotonic', return_value=now / 1e9), \
           patch.object(card.sm, 'update'):
        cs, _ = card.state_update()
      assert cs.canValid and not cs.canTimeout
      assert cs.cruiseState.available == main
      return cs

    try:
      cycle(0, enabled=veto == 'startup_active', active=veto == 'startup_active')
      cycle(1, enabled=veto == 'startup_active', active=veto == 'startup_active', fault=True)
      assert card.aol_card_intent._main_cycle_required and not card.aol_card_intent.allowed_latch
      cycle(2, enabled=veto == 'startup_active', active=veto == 'startup_active')
      assert card.aol_card_intent.allowed_latch  # Fresh healthy feedback restores the persistent MAIN level.
      if veto == 'held':
        card.aol_card_intent._main_held = True
      cs = cycle(3, enabled=True, active=veto != 'pre_enabled', fault=veto == 'fault',
            main=veto != 'main_off', stale=veto == 'stale', native_rejection=veto == 'native')
      if veto == 'native':
        assert not card.aol_card_intent.allowed_latch  # An inconsistent native request still refuses MAIN arming.
        intent = SimpleNamespace(allowedLatch=True, pauseLateral=False, pauseLongitudinal=False)
        denied = decode_safety(card.sm['aolSafetyWire'])
        decision = decide_axes(standard_lateral=False, standard_longitudinal=False, intent=intent, native=denied,
          car_state=cs, initialized=True, model_ready=True, no_entry=False, immediate_disable=False,
          dm_lockout=False, pause_brake_mps=0.)
        assert decision.desired_lateral and not decision.lateral_active and not decision.longitudinal_active
        cycle(4, fault=True)
        assert not card.aol_card_intent.allowed_latch and card.aol_card_intent._main_cycle_required
      elif veto in ('fault', 'main_off'):
        assert not card.aol_card_intent.allowed_latch
      else:
        assert card.aol_card_intent.allowed_latch and not card.aol_card_intent._main_cycle_required
        assert not card.aol_card_intent.pause_lateral
        cs = cycle(4, brake=True)
        assert cs.brakePressed
        assert card.aol_card_intent.allowed_latch and card.aol_card_intent.output(cs)[0]
      assert card.CP.to_dict() == initial_cp
      assert saved.get_bool('AlwaysOnLateral')
    finally:
      card.vehicle_startup.close()
