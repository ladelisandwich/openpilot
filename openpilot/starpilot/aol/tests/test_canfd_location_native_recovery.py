import dataclasses
import importlib
import os
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.hyundai.canfd_stock_aol import qualified
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.radar_interface import RadarInterface
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.cereal import log, messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car import card as card_module
from openpilot.starpilot.aol.intent import AOL_TOGGLE, read_settings
from openpilot.starpilot.aol.runtime import current_native
from openpilot.starpilot.aol.tests import test_ioniq_availability as caller_fixture
from openpilot.starpilot.aol.tests.test_classic_location_stock_intent import host_heartbeat_bridge as host_heartbeat_bridge
from openpilot.starpilot.aol.tests.test_classic_location_stock_intent import (
  assert_stock_location_warning_contract as assert_stock_location_refusals,
)
from openpilot.starpilot.aol.wire import PandaSlot, SafetyState, encode_safety

disable_ecu_module = importlib.import_module('opendbc.car.disable_ecu')
FAMILIES = ['radar', 'lka', 'lka_alt', 'camera', 'ccnc_camera', 'radar_buttons_alt', 'camera_buttons_alt', 'ccnc_camera_buttons_alt',
            'carnival_lka_buttons_alt', 'carnival_lka_alt_buttons_alt', 'carnival_hybrid_lka_buttons_alt', 'carnival_hybrid_lka_alt_buttons_alt']
WORDS = dict(zip(FAMILIES, [0x800, 0x811, 0x891, 0x809, 0xC09, 0x820, 0x829, 0xC29, 0x2830, 0x28B0, 0x2832, 0x28B2], strict=True))



def params(family):
  lka = family in ('lka', 'lka_alt') or family.startswith('carnival')
  fp = gen_empty_fingerprint()
  buttons = {0x1AA: 16} if family.endswith('buttons_alt') else {0x1CF: 8}
  fp[1 if lka else 0].update({**buttons, 0x130: 32, 0x35: 32, 0x100: 32, 0x175: 24, 0xA0: 24, 0xEA: 24, 0x1A0: 32})
  if lka:
    fp[2].update({0x110: 32, 0x362: 32} if 'lka_alt' in family else {0x50: 16, 0x2A4: 24})
  identity = (CAR.HYUNDAI_IONIQ_5_N if family.startswith('ccnc') else CAR.KIA_NIRO_EV_2ND_GEN if family.startswith('camera') else
              CAR.KIA_EV6 if lka else CAR.GENESIS_GV70_1ST_GEN)
  if family.startswith('carnival'):
    identity = CAR.KIA_CARNIVAL_HEV_4TH_GEN if 'hybrid' in family else CAR.KIA_CARNIVAL_2025
    if 'hybrid' in family:
      fp[1][0xFA] = 8
  return CarInterface.get_params(identity, fp, [], False, False, False)


def physical_frames(ci, tick, *, main=0, lkas=0, button=0, engaged=False):
  frames = []
  buses = CanBus(ci.CP)
  lka = bool(ci.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG)
  steer_name = ('LKAS_ALT' if ci.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT else 'LKAS') if lka else 'LFA'
  steer_bus = buses.ACAN if lka else buses.ECAN
  for parser in ci.can_parsers.values():
    packer = CANPacker(parser.dbc.name)
    for address in sorted(parser.addresses):
      message = parser.dbc.addr_to_msg[address]
      # Parser frequency is a dropout-tolerant validity floor; the physical button source is 50 Hz.
      frequency = 50 if message.name == ci.CS.cruise_btns_msg_canfd else parser.message_states[address].frequency
      period = max(1, round(100 / frequency)) if frequency > 0 else 1
      if tick % period:
        continue
      name = message.name
      assert name != steer_name or parser.bus != steer_bus, 'Fixture must not echo outgoing steering on protected RX bus'
      values = {}
      if 'COUNTER' in message.sigs:
        values['COUNTER'] = tick // period % (1 << message.sigs['COUNTER'].size)
      if name == ci.CS.gear_msg_canfd:
        values['GEAR'] = next(code for code, label in ci.CS.shifter_values.items() if label == 'D')
      if name == 'DOORS_SEATBELTS':
        values['DRIVER_SEATBELT'] = 1
      if name == 'TCS':
        values.update(ACCEnable=0, ACC_REQ=int(engaged), DriverBraking=0)
      if name == 'WHEEL_SPEEDS':
        values.update(dict.fromkeys(('WHL_SpdFLVal', 'WHL_SpdFRVal', 'WHL_SpdRLVal', 'WHL_SpdRRVal'), 36))
      if name == 'STEERING_SENSORS':
        values['STEERING_ANGLE'] = 1.
      if name == 'MDPS':
        values.update(MDPS_StrTqSnsrVal=0, MDPS_OutTqVal=0)
      if name == ci.CS.cruise_btns_msg_canfd:
        values.update(LDA_BTN=lkas, CRUISE_BUTTONS=button, ADAPTIVE_CRUISE_MAIN_BTN=0)
      if name == 'SCC_CONTROL':
        values.update(MainMode_ACC=main, ACCMode=int(engaged), VSetDis=60)
      frames.append(packer.make_can_msg(name, parser.bus, values))
  return frames


@pytest.mark.parametrize('family', FAMILIES)
@pytest.mark.parametrize('unavailable_ticks', [3, 8])
def test_actual_canfd_card_host_native_location_recovery(family, unavailable_ticks, monkeypatch, host_heartbeat_bridge):
  raw_fault = 0
  shared_long = False
  native = libsafety_py.libsafety
  case = caller_fixture.IoniqAvailabilityTests(methodName='runTest')
  card = None
  with OpenpilotPrefix(), patch.dict(os.environ, {'SIMULATION': '0', 'AOL_REPLAY_RUNTIME': '1'}):
    saved = Params()
    saved.put_bool('OpenpilotEnabledToggle', True, block=True)
    saved.put_bool('AlwaysOnLateral', True, block=True)
    saved.put('LKASButtonControl', AOL_TOGGLE, block=True)
    saved.put('MainCruiseButtonControl', 0, block=True)
    now = [10_000_000_000]
    for key in ('CancelButtonControl', 'LongCancelButtonControl', 'VeryLongCancelButtonControl'):
      saved.put(key, 0, block=True)
    try:
      saved.put_bool('AlphaLongitudinalEnabled', False, block=True)
      saved.put_bool('IsReleaseBranch', False, block=True)
      cp = params(family)
      assert qualified(cp) and cp.pcmCruise and not cp.openpilotLongitudinalControl
      assert cp.lateralTuning.which() == 'torque' and not CarInterface.startup_required(cp)
      card = card_module.Car(CI=CarInterface(cp), RI=RadarInterface(cp))
      assert qualified(card.CP, marked_only=True)
      assert card.CP.safetyConfigs[0].safetyParam == WORDS[family] and card.CP.alternativeExperience == 0
      assert not card.ci_initialized and not saved.get_bool('ControlsReady')
      card.slc_producer_session = 'authority-card'
      card.CI.update([])
      sd, controls, drive = case._authority_callers(False, cp=card.CP.as_reader().as_builder(), genuine_events=True)

      sd.aol_settings = read_settings(saved)
      assert sd.aol_settings.lkas_action == AOL_TOGGLE and sd.aol_settings.main_action == 0
      assert sd.CP.to_dict() == card.CP.to_dict() == controls.CP.to_dict()
      assert len(card.CP.safetyConfigs) == 1 and bool(card.CP.openpilotLongitudinalControl) == shared_long
      config = card.CP.safetyConfigs[0]
      model, word = int(config.safetyModel.raw), int(config.safetyParam)
      buses = CanBus(card.CP)
      lka = bool(card.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG)
      steer_name = ('LKAS_ALT' if card.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT else 'LKAS') if lka else 'LFA'
      steer_bus = buses.ACAN if lka else buses.ECAN
      decoder = CANParser('hyundai_canfd_generated', [(steer_name, 100)], steer_bus)
      steer_address = decoder.dbc.name_to_msg[steer_name].address
      native.set_alternative_experience(card.CP.alternativeExperience)
      assert native.set_safety_hooks(model, word) == 0
      native.init_tests()
      host_heartbeat_bridge.reset_host()
      native.set_alternative_experience(card.CP.alternativeExperience)
      queued, received, previous_sd = [], {}, {}
      tick = 0
      native_counts = {'accepted': 0, 'blocked': 0, 'nonzero': 0, 'lateral_only_nonzero': 0}
      freeze_native = [None]
      last_native = [None]
      telemetry_rate_bad = [False]
      patch_scope = monkeypatch.context()
      local = patch_scope.__enter__()
      try:
        diagnostic_requests = []
        def diagnostic_query(sender, receiver, bus, addresses, requests, responses):
          assert bus == 0 and addresses == [(0x7d0, None)]
          assert len(requests) == 1
          request = requests[0]
          assert request in (b'\x10\x03', b'\x28\x83\x01')
          diagnostic_requests.append(request)
          def receive(timeout):
            sender([(0x7d0, bytes([len(request)]) + request + bytes(7-len(request)), bus)])
            return {(0x7d0, None): b'\x50\x03'} if request == b'\x10\x03' else {}
          return SimpleNamespace(get_data=receive)
        local.setattr(disable_ecu_module, 'IsoTpParallelQuery', diagnostic_query)
        local.setattr(card_module, 'REPLAY', False)
        local.setattr(card_module.time, 'monotonic_ns', lambda: now[0])
        local.setattr(card_module.time, 'clock_gettime_ns', lambda clock: now[0])
        local.setattr(card_module.messaging, 'drain_sock_raw', lambda *args, **kwargs: list(queued))
        local.setattr(card.sm, 'update', lambda *args: None)
        def panda_cadence(original):
          due = [0.0]
          def update(stamp, messages):
            delivered = []
            for message in messages:
              if message.which() == 'pandaStates':
                if stamp + 1e-9 < due[0]:
                  continue
                due[0] = stamp + (0.0 if telemetry_rate_bad[0] else 0.1)
              delivered.append(message)
            return original(stamp, delivered)
          return update
        local.setattr(sd.sm, 'update_msgs', panda_cadence(sd.sm.update_msgs))
        motion_unavailable = [False]
        update_messages = sd.sm.update_msgs
        def motion_messages(timestamp, messages):
          delivered = []
          for message in messages:
            if message.which() == 'deviceMotion':
              message = message.as_builder()
              message.deviceMotion.inputsOK = not motion_unavailable[0]
              message = message.as_reader()
            delivered.append(message)
          return update_messages(timestamp, delivered)
        local.setattr(sd.sm, 'update_msgs', motion_messages)

        local.setattr(card.sm, 'update_msgs', panda_cadence(card.sm.update_msgs))
        create_message = messaging.new_message
        def clocked_message(service=None, size=None, **kwargs):
          kwargs.setdefault('logMonoTime', now[0])
          event = create_message(service, size, **kwargs)
          if service == 'aolSafetyWire' and freeze_native[0] is not None:
            event.logMonoTime = freeze_native[0].observedMonoTime
          if service == 'pandaStates' and raw_fault:
            for panda in event.pandaStates:
              panda.faults = ['interruptRateCan2']
          return event
        local.setattr(messaging, 'new_message', clocked_message)
        original_encode = caller_fixture.encode_safety
        def recorded_fault_wire(state):
          state = dataclasses.replace(state, pandaInventory=tuple(
            dataclasses.replace(slot, faults=slot.faults | raw_fault) for slot in state.pandaInventory))
          last_native[0] = state
          return original_encode(freeze_native[0] or state)
        local.setattr(caller_fixture, 'encode_safety', recorded_fault_wire)
        send = card.pm.send
        def capture(service, event):
          raw = event if isinstance(event, bytes) else event.to_bytes()
          received[service] = messaging.log_from_bytes(raw).as_builder()
          return send(service, raw)
        local.setattr(card.pm, 'send', capture)

        def step(*, main=1, lkas=0, button=0, engaged=False, withdraw_request=False, bad_checksum=False,
                 heartbeat_override=None, inject_relay=False, drop_wheel=False):
          nonlocal tick
          tick += 1
          now[0] = source_ns = 10_000_000_000 + tick * 10_000_000
          prior_axis = previous_sd.get('aolAxisState')
          prior_state = previous_sd.get('selfdriveState')
          native.set_timer(source_ns // 1000)
          if tick % 10 == 0:
            axis_bytes = prior_axis.as_builder().to_bytes() if prior_axis is not None else b''
            intent_message = received.get('aolIntentWire')
            intent_bytes = intent_message.to_bytes() if intent_message is not None else b''
            packed = host_heartbeat_bridge.plan_host(model, word, native.aol_get_request_mask(), native.aol_get_permission_mask(),
              source_ns, axis_bytes, len(axis_bytes), intent_bytes, len(intent_bytes),
              prior_state is not None and prior_state.selfdriveState.enabled)
            request = (packed & 0xFF) if not withdraw_request else 0
            heartbeat = bool(packed & 0x100) if heartbeat_override is None else heartbeat_override
            native.aol_set_host_request(request)
            native.set_aol_test_heartbeat(heartbeat)
            host_heartbeat_bridge.complete_host(native.aol_get_request_mask(), native.aol_get_permission_mask())
          frames = physical_frames(card.CI, tick, main=main, lkas=lkas, button=button, engaged=engaged)
          if drop_wheel:
            frames = [frame for frame in frames if frame[0] != 0x386]
          if bad_checksum:
            frames = [(address, bytes([data[0] ^ 1]) + data[1:] if address == 0x386 else data, bus)
                      for address, data, bus in frames]
          if inject_relay:
            camera = next(frame for frame in frames if frame[0] == 0x340 and frame[2] == 2)
            frames.append((camera[0], camera[1], 0))
          for address, data, bus in frames:
            accepted = native.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data))
            assert bool(accepted) == (not bad_checksum or address != 0x386), (hex(address), bus)
          if tick % 100 == 0:
            native.safety_tick()
          actual_request = native.aol_get_request_mask()
          mask = native.aol_get_permission_mask()



          rx_invalid = bool(native.safety_test_toyota_rx_summary_invalid())
          relay_fault = bool(native.get_relay_malfunction())
          assert not relay_fault or inject_relay
          batch = messaging.log_from_bytes(card_module.can_list_to_can_capnp(frames)).as_builder()
          batch.logMonoTime = source_ns
          queued[:] = [batch.to_bytes()]
          device = messaging.new_message('deviceState', valid=True, logMonoTime=source_ns)
          device.deviceState.started, device.deviceState.startedMonoTime = True, 9_000_000_000
          pandas = messaging.new_message('pandaStates', 1, valid=True, logMonoTime=source_ns)
          panda = pandas.pandaStates[0]
          panda.safetyModel, panda.safetyParam = config.safetyModel, word
          panda.alternativeExperience = card.CP.alternativeExperience
          panda.controlsAllowed = native.get_controls_allowed()
          panda.safetyRxChecksInvalid = rx_invalid
          panda.ignitionLine = True
          panda.faults = (['interruptRateCan2'] if raw_fault else []) + (['relayMalfunction'] if relay_fault else [])
          slot = PandaSlot(0, 'authority-panda', model, word, card.CP.alternativeExperience,
                           native.get_controls_allowed(), safetyRxChecksInvalid=rx_invalid, faults=raw_fault | int(relay_fault))
          wire = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=source_ns)
          wire.aolSafetyWire = encode_safety(SafetyState(1, True, source_ns, source_ns + 30_000_000,
            model, word, bool(mask & 1), bool(mask & 2), bool(actual_request & 1), bool(actual_request & 2),
            'authority-panda', card.aol_axis_session(), source_ns, (slot,)))
          messages = [device.as_reader(), pandas.as_reader(), wire.as_reader()]
          messages += [previous_sd[name] for name in ('selfdriveState', 'aolAxisState', 'onroadEvents') if name in previous_sd]
          if 'onroadEvents' not in previous_sd:
            messages.append(messaging.new_message('onroadEvents', 0, valid=True, logMonoTime=source_ns).as_reader())
          card.sm.update_msgs(source_ns / 1e9, messages)
          state, _ = card.state_update()
          if state.canValid:
            assert state.cruiseState.available
          for service in ('carState', 'carOutput', 'aolIntentWire', 'starpilotCarState'):
            received.pop(service, None)
          card.state_publish(state, None)
          state_envelope = received['carState']
          output_envelope = received['carOutput']
          assert state_envelope.logMonoTime == source_ns and state_envelope.valid == state.canValid
          assert output_envelope.logMonoTime == source_ns

          axis, host, command = drive(tick, intent_message=received['aolIntentWire'],
            car_state_message=state_envelope, car_output_message=output_envelope,
            companion=False, authority_message=received.get('starpilotCarState'),
            allowed=bool(mask & 1), long_allowed=bool(mask & 2), native_request_lat=bool(actual_request & 1),
            native_request_long=bool(actual_request & 2), panda_controls_allowed=native.get_controls_allowed(),
            panda_fault=rx_invalid, relay_fault=relay_fault)
          if sd.conditional_car_state_valid:
            assert sd.aol_car_state_log_ns == state_envelope.logMonoTime
          assert controls.sm.logMonoTime['carState'] == state_envelope.logMonoTime
          assert controls.sm.valid['carState'] == state_envelope.valid
          assert controls.sm['carState'].to_dict() == state_envelope.carState.to_dict()
          assert controls.sm.logMonoTime['carOutput'] == output_envelope.logMonoTime
          assert controls.sm.valid['carOutput'] == output_envelope.valid
          assert controls.sm['carOutput'].to_dict() == output_envelope.carOutput.to_dict()
          previous_sd.update(sd.pm.messages)
          validated = current_native(sd.sm, sd.CP, now_ns=source_ns, axis_session_id=sd.aol_session_id)
          if not rx_invalid and not relay_fault and freeze_native[0] is None and state.canValid and native.safety_config_valid():
            assert validated is not None, 'RECORDED_IRQ_ONLY_CURRENT_NATIVE_REJECTED'
            assert validated is not None and validated.lateralAllowed == bool(mask & 1)
            assert validated.requestedLateral == bool(actual_request & 1)
            assert validated.pandaInventory[0].faults == raw_fault
            assert list(sd.sm['pandaStates'][0].faults) == ([log.PandaState.FaultType.interruptRateCan2] if raw_fault else [])
          else:
            assert not axis.lateralActive and not axis.longitudinalActive
            assert not command.latActive and not command.longActive

          now[0] += 1
          card.sm.update_msgs(now[0] / 1e9, [controls.pm.messages['carControl']])
          received.pop('sendcan', None)
          card.controls_update(state, card.sm['carControl'])
          steering = []
          if state.canValid:
            assert 'sendcan' in received and received['sendcan'].valid
            steering = [frame for frame in received['sendcan'].sendcan
                        if frame.address == steer_address and frame.src == steer_bus]
            assert steering
            for frame in received['sendcan'].sendcan:
              accepted = bool(native.safety_tx_hook(libsafety_py.make_CANPacket(frame.address, frame.src, frame.dat)))
              assert accepted == (not relay_fault)
              native_counts['accepted' if accepted else 'blocked'] += 1
              if accepted and frame.address == steer_address and frame.src == steer_bus:
                assert steer_address in decoder.update([(source_ns, [(frame.address, bytes(frame.dat), frame.src)])])
                if not command.latActive:
                  assert decoder.vl[steer_name]['StrTqReqVal'] == 0
                  assert decoder.vl[steer_name]['ActToiSta'] == 0
                if decoder.vl[steer_name]['StrTqReqVal'] != 0:
                  native_counts['nonzero'] += 1
                  native_counts['lateral_only_nonzero'] += int(not native.get_controls_allowed())
          return state, axis, host, command, steering

        for _ in range(120):
          step(main=0)
        assert native.safety_config_valid()
        for _ in range(2):
          step(main=1, lkas=1, engaged=False)
        for _ in range(40):
          state, axis, host, command, steering = step(main=1, engaged=False)
        assert card.aol_card_intent.allowed_latch
        assert native.aol_get_permission_mask() == 1 and not native.get_controls_allowed()
        assert axis.lateralActive and command.latActive and not host.enabled
        assert not axis.longitudinalActive and not command.longActive
        assert native_counts['lateral_only_nonzero'] > 0
        motion_unavailable[0] = True
        fault_ticks = 0
        baseline = None
        for _ in range(15):
          state, axis, host, command, _ = step(main=1, engaged=False)
          if log.OnroadEvent.EventName.locationdTemporaryError not in sd.events.names:
            continue
          fault_ticks += 1
          if fault_ticks == 1:
            assert_stock_location_refusals(sd, card, state, now[0])
          assert log.OnroadEvent.EventName.pcmDisable in sd.events.names
          assert not axis.lateralActive and not axis.longitudinalActive
          assert not command.latActive and not command.longActive
          assert card.aol_card_intent.allowed_latch, 'CANFD_LOCATION_PCM_OFF_CLEARED_ESTABLISHED_INTENT'
          observed = (card.aol_transport_feedback.lateral_baseline_ns, card.aol_transport_feedback.lateral_native_ns)
          if baseline is None:
            baseline = observed
          else:
            assert observed == baseline
          if fault_ticks == unavailable_ticks:
            break
        assert fault_ticks == unavailable_ticks
        motion_unavailable[0] = False
        for _ in range(20):
          state, axis, host, command, _ = step(main=1, engaged=False)
          assert card.aol_card_intent.allowed_latch
          assert not axis.longitudinalActive and not command.longActive and not host.enabled
        assert axis.lateralActive and command.latActive
      finally:
        patch_scope.__exit__(None, None, None)
    finally:
      native.init_tests()
      native.set_alternative_experience(0)
      native.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)
      case.doCleanups()
      if card is not None:
        card.vehicle_startup.close()
