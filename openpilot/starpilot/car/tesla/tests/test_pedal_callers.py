"""Actual selected startup -> physical CAN -> Card/SD/Controls/Card.apply -> registered native."""
import gc
from unittest.mock import patch
import pytest

from opendbc.car import structs
from opendbc.car.tesla.tests.preap_stream import PreAPStream
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.cereal import messaging
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
from openpilot.starpilot.aol.wire import SafetyState, PandaSlot, encode_safety
from openpilot.starpilot.lateral.tests.test_lane_runtime import feed
from openpilot.starpilot.tests.test_tesla_screen_integration import selfdrive_step
from openpilot.starpilot.car.tesla.tests.pedal_fixture import actual_pedal_startup, configure_native, feedback_frame


@pytest.mark.parametrize('bus', [0, 2])
@pytest.mark.parametrize('radar_mode', ['off', 'front', 'nosecone'])
@pytest.mark.parametrize('fault', ['gas', 'brake', 'dropout', 'reverse', 'feedback_state', 'counter', 'channels', 'wrong_bus',
                                   'eps', 'door', 'belt', 'eps_temporary'])
def test_real_selected_pedal_fault_withdrawal_and_physical_reengagement(bus, radar_mode, fault, monkeypatch):
  safety = libsafety_py.libsafety
  release = safety.set_safety_hooks(structs.CarParams.SafetyModel.allOutput, 0) != 0
  with actual_pedal_startup(bus, radar_mode, release, monkeypatch) as (card, controls, calibration, cp_bytes):
    ci, word = card.CI, int(card.CP.safetyConfigs[0].safetyParam)
    configure_native(safety, calibration, word, cp_bytes)
    with patch('openpilot.selfdrive.selfdrived.selfdrived.REPLAY', False):
      sd = SelfdriveD(CP=card.CP)
    stream = PreAPStream()
    previous = structs.CarControl()
    receipt = None
    selfdriven, observed, diagnostics = {}, {}, {}
    alerts_seen = set()
    try:
      for tick in range(310):
        now = 2_000_000_000 + tick * 10_000_000
        bad = 120 <= tick < (180 if fault in ('dropout', 'reverse', 'wrong_bus') else 140)
        lever = 2 if tick in (65, 75, 220, 230) else 1 if tick == 275 else 0
        incoming = stream.frames(tick, lever=lever, cruise=1,
          brake=2 if bad and fault == 'brake' else 1, gear=2 if bad and fault == 'reverse' else 4,
          doors=1 if bad and fault == 'door' else 0, belt=0 if bad and fault == 'belt' else 1,
          eps_status=3 if bad and fault == 'eps' else 0 if bad and fault == 'eps_temporary' else 1,
          eps_error=15 if bad and fault == 'eps' else 0)
        if tick % 2 == 0 and not (bad and fault == 'dropout'):
          counter = 59 if bad and fault == 'counter' else tick // 2
          incoming.append(feedback_frame(calibration, counter, di=8. if bad and fault == 'gas' else 0.,
            state=1 if bad and fault == 'feedback_state' else 0,
            bad_channel=bad and fault == 'channels', wrong_bus=bad and fault == 'wrong_bus'))
        safety.set_timer(now // 1000)
        safety.set_aol_test_heartbeat(bool(card.aol_card_intent.allowed_latch or sd.enabled))
        for address, data, source in incoming:
          safety.safety_rx_hook(libsafety_py.make_CANPacket(address, source, data))
        safety.safety_tick()
        peers = []
        cc = messaging.new_message('carControl', valid=True, logMonoTime=now)
        cc.carControl = previous
        peers.append(cc.as_reader())
        peers.append(selfdriven['onroadEvents'] if selfdriven else
                     messaging.new_message('onroadEvents', 0, valid=True, logMonoTime=now).as_reader())
        panda = messaging.new_message('pandaStates', 1, valid=True, logMonoTime=now)
        panda.pandaStates[0].safetyModel = card.CP.safetyConfigs[0].safetyModel
        panda.pandaStates[0].safetyParam = word
        panda.pandaStates[0].alternativeExperience = 32
        panda.pandaStates[0].controlsAllowed = bool(safety.get_controls_allowed())
        panda.pandaStates[0].safetyRxChecksInvalid = not bool(safety.safety_config_valid())
        peers.append(panda.as_reader())
        if receipt is not None:
          peers.append(receipt.as_reader())
        if selfdriven:
          peers.extend(selfdriven[key] for key in ('aolAxisState', 'selfdriveState'))
        card.sm.update_msgs(now / 1e9, peers)
        card.CC_prev = previous
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
        assert published['starpilotCarState'].logMonoTime == published['carState'].logMonoTime
        status = published['starpilotCarState'].starpilotCarState
        assert status.sourceCarStateMonoTime == published['carState'].logMonoTime
        assert status.pedalLongActive == ci.CS.preap.enableLongControl
        assert status.pedalMaxRegen == (ci.CS.preap.pccEvent == 'pedalMaxRegen')
        mask = safety.aol_get_permission_mask()
        native = SafetyState(1, True, now, now + 30_000_000, 39, word, bool(mask & 1), bool(mask & 2),
                             bool(safety.aol_get_request_mask() & 1), bool(safety.aol_get_request_mask() & 2),
                             'tesla-pedal-native', sd.aol_session_id, now,
                             (PandaSlot(0, 'tesla-pedal-native', 39, word, 32, bool(safety.get_controls_allowed()),
                                        not bool(safety.safety_config_valid())),))
        receipt = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=now)
        receipt.aolSafetyWire = encode_safety(native)

        def with_actual_companion(service, size=None, published=published, factory=factory, **kwargs):
          if service == 'starpilotCarState' and service in published:
            return published[service].as_builder()
          return factory(service, size, **kwargs)

        with patch('openpilot.selfdrive.selfdrived.selfdrived.messaging.new_message', side_effect=with_actual_companion):
          selfdriven.update(selfdrive_step(sd, published, receipt, previous, card.CP, safety, now))
        alerts_seen.update(alert.alert_type for alert in sd.tesla_stock_alerts)
        wanted = sd.aol_axis_decision
        safety.set_aol_test_heartbeat(bool(card.aol_card_intent.allowed_latch or sd.enabled))
        safety.aol_set_host_request(int(wanted.desired_lateral) | (int(wanted.desired_longitudinal) << 1))
        mask = safety.aol_get_permission_mask()
        native = SafetyState(1, True, now, now + 30_000_000, 39, word, bool(mask & 1), bool(mask & 2),
                             wanted.desired_lateral, wanted.desired_longitudinal, 'tesla-pedal-native', sd.aol_session_id, now,
                             (PandaSlot(0, 'tesla-pedal-native', 39, word, 32, bool(safety.get_controls_allowed()),
                                        not bool(safety.safety_config_valid())),))
        receipt = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=now)
        receipt.aolSafetyWire = encode_safety(native)
        feed(controls, now, tick, active=sd.active, enabled=sd.enabled)
        controls.sm.update_msgs((now + 1000) / 1e9, [published['carState'],
          selfdriven['aolAxisState'], selfdriven['selfdriveState'], selfdriven['onroadEvents'], receipt.as_reader()])
        command, lateral = controls.state_control()
        controls.publish(command, lateral)
        current = messaging.new_message('carControl', valid=True, logMonoTime=now)
        current.carControl = command
        card.sm.update_msgs((now + 1000) / 1e9, [current.as_reader(), selfdriven['aolAxisState'], selfdriven['selfdriveState'], receipt.as_reader()])
        sent = {}
        with patch('openpilot.selfdrive.car.card.time.monotonic', return_value=(now + 1000) / 1e9), \
             patch('openpilot.selfdrive.car.card.time.monotonic_ns', return_value=now + 1000), \
             patch.object(card.pm, 'send', side_effect=lambda service, msg, sent=sent: sent.update({service:
               messaging.log_from_bytes(msg) if isinstance(msg, bytes) else msg.as_reader()})):
          card.controls_update(cs, command.as_reader())
        outgoing = [(item.address, bytes(item.dat), item.src) for item in sent['sendcan'].sendcan] if 'sendcan' in sent else []
        enabled_pedal = False
        for address, data, source in outgoing:
          assert safety.safety_tx_hook(libsafety_py.make_CANPacket(address, source, data)), (fault, tick, hex(address), data.hex())
          if address == 0x551:
            assert source == bus and len(data) == 6
            enabled_pedal |= bool(data[4] & 128)
        assert not command.longActive or bool(mask & 2)
        assert not enabled_pedal or command.longActive
        if bad and fault in ('gas', 'brake', 'reverse', 'feedback_state', 'counter', 'channels', 'eps', 'door', 'belt') and tick >= 122:
          assert not enabled_pedal
        if bad and fault == 'eps_temporary' and tick >= 122:
          assert not command.latActive, 'temporary EPS inhibition must suppress lateral commands'
        observed[tick] = (bool(command.longActive), enabled_pedal, bool(cs.canValid), bool(mask & 2))
        if tick in (64, 65, 66, 74, 75, 76, 95, 100, 110, 160, 250):
          diagnostics[tick] = (cs.cruiseState.enabled, ci.CS.preap.enableLongControl, card.aol_card_intent.allowed_latch, sd.enabled,
                               str(wanted), sd.events.names, mask, observed[tick], command.actuators.accel)
        previous = command
      assert any(observed[t][0] and observed[t][1] for t in range(95, 115)), ('actual initial pedal engagement missing', diagnostics)
      assert 'teslaPreapPedal/engaged/warning' in alerts_seen
      assert 'teslaPreapPedal/disengaged/warning' in alerts_seen
      assert not any(name.startswith('teslaStockCruise/') for name in alerts_seen)
      if fault not in ('gas', 'eps_temporary'):
        assert not any(observed[t][0] or observed[t][1] for t in range(200, 219)), 'fault/gear/brake recovery must not auto-resume long'
      else:
        assert any(observed[t][0] and observed[t][1] for t in range(160, 175)), (
          'healthy override/temporary-pause recovery did not preserve existing active long')
      assert any(observed[t][0] and observed[t][1] for t in range(250, 270)), 'fresh physical reengagement missing'
      assert not any(observed[t][0] or observed[t][1] for t in range(278, 310)), 'physical cancel must withdraw pedal long'
    finally:
      safety.set_aol_test_heartbeat(False)
      safety.set_safety_hooks(0, 0)
      del sd
      gc.collect()
