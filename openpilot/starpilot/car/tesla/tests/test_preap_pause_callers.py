"""Physical CAN through actual preAP startup, Card, SD, Controls, CI and native TX."""
import gc
from unittest.mock import patch

import pytest

from opendbc.car import structs
from opendbc.car.tesla.tests.preap_stream import PreAPStream
from opendbc.car.tesla.tests.test_preap_startup import exercise_startup
from opendbc.car.tesla.values import CAR
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.cereal import messaging
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
from openpilot.starpilot.aol.wire import SafetyState, PandaSlot, encode_safety
from openpilot.starpilot.lateral.tests.test_lane_runtime import feed
from openpilot.starpilot.tests.test_tesla_screen_integration import selfdrive_step


@pytest.mark.parametrize('pause', ('reverse', 'neutral', 'neutral_reverse_neutral', 'temporary'))
def test_preap_actual_physical_arm_pause_return_and_cancel(pause, monkeypatch):
  def probe(card, ci, controls, saved):
    safety = libsafety_py.libsafety
    safety.init_tests()
    safety.set_alternative_experience(32)
    assert safety.set_safety_hooks(39, 0) == 0
    with patch('openpilot.selfdrive.selfdrived.selfdrived.REPLAY', False):
      sd = SelfdriveD(CP=card.CP)
    stream = PreAPStream()
    previous_command = structs.CarControl()
    previous_receipt = None
    selfdriven = {}
    observed = {}
    try:
      for tick in range(210):
        now = 2_000_000_000 + tick * 10_000_000
        paused = 110 <= tick < 140
        lever = 2 if tick == 65 else 1 if tick == 175 else 0
        gear = 4
        if paused:
          if pause == 'reverse':
            gear = 2
          elif pause == 'neutral':
            gear = 3
          elif pause == 'neutral_reverse_neutral':
            gear = 3 if tick < 115 or tick >= 135 else 2
        incoming = stream.frames(tick, lever=lever, cruise=1,
                                 gear=gear,
                                 eps_status=0 if paused and pause == 'temporary' else 1)
        safety.set_timer(now // 1000)
        safety.set_aol_test_heartbeat(bool(card.aol_card_intent.allowed_latch or sd.enabled))
        for address, payload, bus in incoming:
          assert safety.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, payload))
        safety.safety_tick()
        peers = []
        cc = messaging.new_message('carControl', valid=True, logMonoTime=now)
        cc.carControl = previous_command
        peers.append(cc.as_reader())
        peers.append(selfdriven['onroadEvents'] if selfdriven else
                     messaging.new_message('onroadEvents', 0, valid=True, logMonoTime=now).as_reader())
        panda = messaging.new_message('pandaStates', 1, valid=True, logMonoTime=now)
        panda.pandaStates[0].safetyModel = card.CP.safetyConfigs[0].safetyModel
        panda.pandaStates[0].safetyParam = 0
        panda.pandaStates[0].alternativeExperience = 32
        panda.pandaStates[0].controlsAllowed = bool(safety.get_controls_allowed())
        panda.pandaStates[0].safetyRxChecksInvalid = not bool(safety.safety_config_valid())
        peers.append(panda.as_reader())
        if previous_receipt is not None:
          peers.append(previous_receipt.as_reader())
        if selfdriven:
          peers.extend(selfdriven[key] for key in ('aolAxisState', 'selfdriveState'))
        card.sm.update_msgs(now / 1e9, peers)
        card.CC_prev = previous_command
        with patch('openpilot.selfdrive.car.card.messaging.drain_sock_raw', return_value=[b'can']), \
             patch('openpilot.selfdrive.car.card.can_capnp_to_list', return_value=[(now, incoming)]), \
             patch('openpilot.selfdrive.car.card.time.monotonic_ns', return_value=now), \
             patch('openpilot.selfdrive.car.card.time.clock_gettime_ns', return_value=now), \
             patch('openpilot.selfdrive.car.card.REPLAY', False), patch.object(card.sm, 'update'):
          cs, radar = card.state_update()
        published = {}
        factory = messaging.new_message

        def stamped(service, size=None, factory=factory, now=now, **kwargs):
          return factory(service, size, **({'logMonoTime': now} | kwargs))

        with patch('openpilot.selfdrive.car.card.messaging.new_message', side_effect=stamped), \
             patch.object(card.pm, 'send', side_effect=lambda service, msg, published=published: published.update({service: msg.as_reader()})):
          card.state_publish(cs, radar)
        published['pandaStates'] = panda.as_reader()
        mask = safety.aol_get_permission_mask()
        native = SafetyState(1, True, now, now + 30_000_000, 39, 0, bool(mask & 1), bool(mask & 2),
                             bool(safety.aol_get_request_mask() & 1), False, 'tesla-preap-panda', sd.aol_session_id, now,
                             (PandaSlot(0, 'tesla-preap-panda', 39, 0, 32, bool(safety.get_controls_allowed()),
                                        not bool(safety.safety_config_valid())),))
        receipt = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=now)
        receipt.aolSafetyWire = encode_safety(native)
        selfdriven.update(selfdrive_step(sd, published, receipt, previous_command, card.CP, safety, now))
        wanted = sd.aol_axis_decision
        safety.set_aol_test_heartbeat(bool(card.aol_card_intent.allowed_latch or sd.enabled))
        safety.aol_set_host_request(int(wanted.desired_lateral) | (int(wanted.desired_longitudinal) << 1))
        mask = safety.aol_get_permission_mask()
        native = SafetyState(1, True, now, now + 30_000_000, 39, 0, bool(mask & 1), bool(mask & 2),
                             wanted.desired_lateral, wanted.desired_longitudinal, 'tesla-preap-panda', sd.aol_session_id, now,
                             (PandaSlot(0, 'tesla-preap-panda', 39, 0, 32, bool(safety.get_controls_allowed()),
                                        not bool(safety.safety_config_valid())),))
        receipt = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=now)
        receipt.aolSafetyWire = encode_safety(native)
        feed(controls, now, tick, active=bool(cs.cruiseState.enabled), enabled=bool(cs.cruiseState.enabled))
        controls.sm.update_msgs((now + 1000) / 1e9, [published['carState'], selfdriven['aolAxisState'],
          selfdriven['selfdriveState'], selfdriven['onroadEvents'], receipt.as_reader()])
        command, lateral_log = controls.state_control()
        controls.publish(command, lateral_log)
        assert not command.longActive
        assert command.latActive == wanted.lateral_active
        _, outgoing = ci.apply(command.as_reader(), now)
        for address, payload, bus in outgoing:
          assert safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, payload)), (tick, hex(address))
        observed[tick] = (bool(command.latActive), bool(card.aol_card_intent.allowed_latch), mask,
                          bool(cs.canValid), bool(safety.get_controls_allowed()))
        previous_command, previous_receipt = command, receipt
      assert all(row[3] for tick, row in observed.items() if tick >= 60)
      assert not observed[60][0]
      assert observed[90][0] and observed[90][1]
      for tick in range(112, 140):
        assert not observed[tick][0] and observed[tick][1] and observed[tick][2] == 0
      assert observed[155][0] and observed[155][1]
      if pause in ('reverse', 'neutral', 'neutral_reverse_neutral'):
        assert not observed[155][4], 'returning Drive must not restore ordinary controlsAllowed'
      assert not any(observed[tick][0] or observed[tick][1] for tick in range(177, 210))
    finally:
      safety.init_tests()
      safety.set_alternative_experience(0)
      safety.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)
      del sd
      gc.collect()
  exercise_startup(CAR.TESLA_MODEL_S_PREAP, False, False, False, monkeypatch, probe=probe)
