"""Synthetic transport; actual HKG parser/Card/SD state/Controls/CI/native chain.

The runner builds the selected native library from current source.
Forte settings match the reported route; delayed native acknowledgment is modeled explicitly.
"""

import os
from unittest.mock import patch
import pytest
from opendbc.car import Bus, structs
from opendbc.can import CANPacker
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.radar_interface import RadarInterface
from opendbc.car.hyundai.values import CAR
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car import card as card_module
from openpilot.starpilot.aol.intent import AOL_TOGGLE, read_settings
from openpilot.starpilot.aol.runtime import current_native
from openpilot.starpilot.aol.tests.test_forte_intent import params
from opendbc.safety.tests.test_hyundai import checksum

try:
  import _hkg24_callers as caller_fixture
except ModuleNotFoundError as error:
  if error.name != '_hkg24_callers':
    raise
  from openpilot.starpilot.aol.tests import test_ioniq_availability as caller_fixture
from openpilot.starpilot.aol.wire import PandaSlot, SafetyState, encode_safety, decode_safety
from opendbc.safety.tests.libsafety import libsafety_py


def forte_frames(ci, tick, *, main=False, lkas=False, button=0, engaged=False, main_button=False, gas=True):
  packer = CANPacker(ci.can_parsers[Bus.pt].dbc.name)
  gear = next((code for code, name in ci.CS.shifter_values.items() if name == 'D'))
  values: dict[str, dict[str, float]] = {
    'EMS16': {'CRUISE_LAMP_M': int(main), 'CRUISE_LAMP_S': int(engaged), 'AliveCounter': tick % 4, 'CF_Ems_AclAct': int(gas)},
    'WHL_SPD11': {
      'WHL_SPD_FL': 40,
      'WHL_SPD_FR': 40,
      'WHL_SPD_RL': 40,
      'WHL_SPD_RR': 40,
      'WHL_SPD_AliveCounter_LSB': tick % 4,
      'WHL_SPD_AliveCounter_MSB': tick // 4 % 4,
    },
    'TCS13': {'AliveCounterTCS': tick % 8},
    'MDPS12': {'CR_Mdps_StrColTq': 0.0, 'CR_Mdps_OutTq': 0.0},
    'SAS11': {'SAS_Angle': 0.0, 'SAS_Speed': 0.0},
    'CLU11': {'CF_Clu_AliveCnt1': tick % 16, 'CF_Clu_CruiseSwState': button, 'CF_Clu_CruiseSwMain': int(main_button)},
    'LVR12': {'CF_Lvr_Gear': gear},
    'BCM_PO_11': {'LDA_BTN': int(lkas)},
    'CLU13': {'CF_Clu_LdwsLkasSW': 0},
    'CGW1': {'CF_Gway_DrvSeatBeltSw': 1},
  }
  result = []
  for name in ('EMS12', 'TCS11', 'TCS15', 'CLU15', 'ESP12', 'CGW2', *values):
    msg = packer.make_can_msg(name, 0, values.get(name, {}))
    result.append(checksum(msg) if name in ('EMS16', 'WHL_SPD11', 'TCS13') else msg)
  result.append(packer.make_can_msg('LKAS11', 2, {'CF_Lkas_MsgCount': tick % 16}))
  return [(address, bytes(data), bus) for address, data, bus in result]


def _exercise_forte_join(gesture, withdrawal, monkeypatch):
  main_action = AOL_TOGGLE if gesture == 'main' else 0
  offset = 0
  bootstrap_refusals = []
  _cancel_diagnostics, _cancel_probe_hits = ([], [])
  native = libsafety_py.libsafety
  case = caller_fixture.IoniqAvailabilityTests(methodName='runTest')
  card = None
  with OpenpilotPrefix(), patch.dict(os.environ, {'SIMULATION': '0', 'AOL_REPLAY_RUNTIME': '1'}):
    saved = Params()
    saved.put_bool('OpenpilotEnabledToggle', True, block=True)
    saved.put_bool('AlwaysOnLateral', True, block=True)
    saved.put('LKASButtonControl', AOL_TOGGLE, block=True)
    saved.put('MainCruiseButtonControl', main_action, block=True)
    saved.put_bool('DisengageOnAccelerator', False, block=True)
    now = [10000000000]
    for key in ('CancelButtonControl', 'LongCancelButtonControl', 'VeryLongCancelButtonControl'):
      saved.put(key, 0, block=True)
    try:
      cp = params(CAR.KIA_FORTE_2019_NON_SCC, source=913)
      card = card_module.Car(CI=CarInterface(cp), RI=RadarInterface(cp))
      assert not card.ci_initialized and (not saved.get_bool('ControlsReady'))
      card.slc_producer_session = 'authority-card'
      card.CI.update([])
      sd, controls, drive = case._authority_callers(False, cp=card.CP.as_reader().as_builder(), genuine_events=True)
      sd.aol_settings = read_settings(saved)
      assert sd.aol_settings.lkas_action == AOL_TOGGLE and sd.aol_settings.main_action == main_action
      assert sd.CP.to_dict() == card.CP.to_dict() == controls.CP.to_dict()
      assert bool(card.CP.openpilotLongitudinalControl) is False
      config = card.CP.safetyConfigs[-1]
      model, word = (int(config.safetyModel.raw), int(config.safetyParam))
      native.set_alternative_experience(card.CP.alternativeExperience)
      assert native.set_safety_hooks(model, word) == 0
      native.init_tests()
      native.set_alternative_experience(card.CP.alternativeExperience)
      queued, received, previous_sd = ([], {}, {})
      last_wire = None
      axis_history = []
      trace = []
      tick = 0
      patch_scope = monkeypatch.context()
      local = patch_scope.__enter__()
      try:
        local.setattr(card_module, 'REPLAY', False)
        local.setattr(card_module.time, 'monotonic_ns', lambda: now[0])
        local.setattr(card_module.time, 'clock_gettime_ns', lambda clock: now[0])
        local.setattr(card_module.messaging, 'drain_sock_raw', lambda *args, **kwargs: list(queued))
        local.setattr(card.sm, 'update', lambda *args: None)
        create_message = messaging.new_message

        def clocked_message(service=None, size=None, **kwargs):
          kwargs.setdefault('logMonoTime', now[0])
          return create_message(service, size, **kwargs)

        local.setattr(messaging, 'new_message', clocked_message)
        send = card.pm.send

        def capture(service, event):
          raw = event if isinstance(event, bytes) else event.to_bytes()
          received[service] = messaging.log_from_bytes(raw).as_builder()
          return send(service, raw)

        local.setattr(card.pm, 'send', capture)

        def step(*, main=False, lkas=False, button=0, engaged=False, withdraw_request=False, main_button=False, gas=True, withhold_heartbeat=False):
          nonlocal tick, last_wire
          tick += 1
          now[0] = source_ns = 10000000000 + tick * 10000000
          prior_axis = previous_sd.get('aolAxisState')
          prior_state = previous_sd.get('selfdriveState')
          request = 0 if prior_axis is None else int(prior_axis.aolAxisState.desiredLateral) | int(prior_axis.aolAxisState.desiredLongitudinal) << 1
          if withdraw_request:
            request = 0
          heartbeat = bool(request or (prior_state is not None and prior_state.selfdriveState.enabled))
          native.set_timer(source_ns // 1000)
          native.set_aol_test_heartbeat(heartbeat and not withhold_heartbeat)
          native.aol_set_host_request(request)
          frames = forte_frames(card.CI, tick, main=main, lkas=lkas, button=button, engaged=engaged, main_button=main_button, gas=gas)
          for address, data, bus in frames:
            native.safety_rx_hook(libsafety_py.make_CANPacket(address, bus - offset, data))
          if tick % 100 == 0:
            native.safety_tick()
          actual_request = native.aol_get_request_mask()
          mask = native.aol_get_permission_mask()
          rx_invalid = bool(native.safety_test_toyota_rx_summary_invalid())
          relay_fault = bool(native.get_relay_malfunction())
          assert not relay_fault
          batch = messaging.log_from_bytes(card_module.can_list_to_can_capnp(frames)).as_builder()
          batch.logMonoTime = source_ns
          queued[:] = [batch.to_bytes()]
          device = messaging.new_message('deviceState', valid=True, logMonoTime=source_ns)
          device.deviceState.started, device.deviceState.startedMonoTime = (True, 9000000000)
          pandas = messaging.new_message('pandaStates', 1, valid=True, logMonoTime=source_ns)
          panda = pandas.pandaStates[-1]
          panda.safetyModel, panda.safetyParam = (config.safetyModel, word)
          panda.alternativeExperience = card.CP.alternativeExperience
          panda.controlsAllowed = native.get_controls_allowed()
          panda.safetyRxChecksInvalid = rx_invalid
          panda.ignitionLine = True
          slot = PandaSlot(
            int(False), 'authority-panda', model, word, card.CP.alternativeExperience, native.get_controls_allowed(), safetyRxChecksInvalid=rx_invalid
          )
          inventory = (slot,)
          wire = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=source_ns)
          wire.aolSafetyWire = encode_safety(
            SafetyState(
              1,
              True,
              source_ns,
              source_ns + 200000000,
              model,
              word,
              bool(mask & 1),
              bool(mask & 2),
              bool(actual_request & 1),
              bool(actual_request & 2),
              'authority-panda',
              card.aol_axis_session(),
              source_ns,
              inventory,
            )
          )
          if last_wire is None or tick % 10 == 0:
            last_wire = wire
          else:
            wire = last_wire
          messages = [device.as_reader(), pandas.as_reader(), wire.as_reader()]
          messages += [previous_sd[name] for name in ('selfdriveState', 'onroadEvents') if name in previous_sd]
          if 'aolAxisState' in previous_sd:
            axis_history.append(previous_sd['aolAxisState'])
            messages.append(axis_history[-2] if len(axis_history) > 1 else axis_history[-1])
          if 'onroadEvents' not in previous_sd:
            messages.append(messaging.new_message('onroadEvents', 0, valid=True, logMonoTime=source_ns).as_reader())
          card.sm.update_msgs(source_ns / 1000000000.0, messages)
          state, _ = card.state_update()
          if state.canValid:
            assert state.cruiseState.available == main
          for service in ('carState', 'carOutput', 'aolIntentWire', 'starpilotCarState'):
            received.pop(service, None)
          card.state_publish(state, None)
          state_envelope = received['carState']
          output_envelope = received['carOutput']
          assert state_envelope.logMonoTime == source_ns and state_envelope.valid == state.canValid
          assert output_envelope.logMonoTime == source_ns
          axis, host, command = drive(
            tick,
            intent_message=received['aolIntentWire'],
            car_state_message=state_envelope,
            car_output_message=output_envelope,
            companion=False,
            authority_message=received.get('starpilotCarState'),
            allowed=bool(mask & 1),
            long_allowed=bool(mask & 2),
            native_request_lat=bool(actual_request & 1),
            native_request_long=bool(actual_request & 2),
            panda_controls_allowed=native.get_controls_allowed(),
            panda_fault=rx_invalid,
            relay_fault=relay_fault,
            panda_state_message=pandas,
            safety_message=wire,
          )
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
          bootstrap_mismatch = decode_safety(wire.aolSafetyWire).axisSessionId != sd.aol_session_id
          if bootstrap_mismatch:
            bootstrap_refusals.append(source_ns)
            assert validated is None
            assert not axis.lateralActive and (not axis.longitudinalActive)
            assert not command.latActive and (not command.longActive)
          elif not rx_invalid:
            assert validated is not None
            assert validated.lateralAllowed == decode_safety(wire.aolSafetyWire).lateralAllowed
          else:
            assert not axis.lateralActive and (not axis.longitudinalActive)
            assert not command.latActive and (not command.longActive)
          now[0] += 1
          card.sm.update_msgs(now[0] / 1000000000.0, [controls.pm.messages['carControl']])
          received.pop('sendcan', None)
          card.controls_update(state, card.sm['carControl'])
          steering = []
          if state.canValid:
            assert 'sendcan' in received and received['sendcan'].valid
            steering = [frame for frame in received['sendcan'].sendcan if frame.address == 832 and frame.src == 0]
            assert steering
            for frame in received['sendcan'].sendcan:
              assert native.safety_tx_hook(libsafety_py.make_CANPacket(frame.address, frame.src - offset, frame.dat))
          trace.append(
            {
              'tick': tick,
              'latch': card.aol_card_intent.allowed_latch,
              'desired': bool(axis.desiredLateral),
              'active': bool(axis.lateralActive),
              'nativeAllowed': bool(mask & 1),
              'wireNs': int(wire.logMonoTime),
              'main': bool(state.cruiseState.available),
              'faultRearm': card.aol_card_intent.fault_rearm,
              'nativeRxInvalid': rx_invalid,
              'mainOffSeen': card.aol_card_intent.main_off_seen,
              'feedbackFailed': card.aol_transport_feedback.failed,
              'signature': card.aol_transport_feedback.signature,
              'canValid': state.canValid,
              'gear': str(state.gearShifter),
              'events': [str(e) for e in sd.events.names],
              'fault': str(axis.faultReason),
              'cardOutputValid': output_envelope.valid,
              'controlChecks': card.sm.all_checks(['carControl']),
              'controlFreq': card.sm.freq_ok['carControl'],
              'allChecks': sd.sm.all_checks(),
              'arm': getattr(card.aol_card_intent, 'physical_arm_ns', 0),
            }
          )
          card.CS_prev = state  # Car.step bookkeeping after these actual production calls.
          return (state, axis, host, command, steering)

        for _ in range(120):
          step()
        # Establish startup health through actual MAIN on/off traffic, as in
        # the recorded route, before the independent arming gesture under test.
        for _ in range(24):
          step(main=True, main_button=_ < 2)
        for _ in range(26):
          step(main=False, main_button=_ < 2)
        assert native.safety_config_valid() and not native.safety_test_toyota_rx_summary_invalid()
        assert card.aol_card_intent.main_off_seen and not card.aol_transport_feedback.failed, ('FORTE_STARTUP_NOT_READY', trace[-30:])
        assert not card.aol_card_intent.allowed_latch
        for _ in range(2):
          step(main=gesture == 'main', lkas=gesture == 'lkas', main_button=gesture == 'main', withdraw_request=withdrawal == 'no-request')
        for _ in range(24):
          state, axis, host, command, steering = step(main=gesture == 'main', withdraw_request=withdrawal == 'no-request')
          assert not command.longActive and (not axis.longitudinalActive) and (not host.enabled)
          if withdrawal == 'no-request':
            assert not command.latActive and (not axis.lateralActive)
        if withdrawal == 'no-request':
          assert card.aol_card_intent.allowed_latch, ('FORTE_ARMED_INTENT_PRESERVED_WITHOUT_OUTPUT_ACK', trace[-26:])
          assert not native.aol_get_permission_mask()
          return
        assert card.aol_card_intent.allowed_latch and axis.lateralActive and command.latActive, ('FORTE_PHYSICAL_ARM_LOST_BEFORE_NATIVE_ACK', trace[-28:])
        assert axis.nativeAcknowledged and native.aol_get_permission_mask() == 1
        assert not state.cruiseState.enabled and state.gasPressed
        for _ in range(12):
          state, axis, host, command, steering = step(main=gesture == 'main', withhold_heartbeat=withdrawal == 'native-revoke')
          if withdrawal != 'native-revoke':
            assert axis.lateralActive and command.latActive
        if withdrawal == 'native-revoke':
          assert not axis.lateralActive and not command.latActive
          assert native.aol_get_permission_mask() == 0

        else:
          for _ in range(2):
            step(main=False, lkas=gesture == 'lkas')
          for _ in range(12):
            state, axis, host, command, steering = step(main=False)
          assert not card.aol_card_intent.allowed_latch and (not axis.lateralActive) and (not command.latActive)
      finally:
        patch_scope.__exit__(None, None, None)
    finally:
      native.init_tests()
      native.set_alternative_experience(0)
      native.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)
      case.doCleanups()
      if card is not None:
        card.vehicle_startup.close()


@pytest.mark.parametrize('gesture', ('main', 'lkas'))
@pytest.mark.parametrize('withdrawal', ('off', 'no-request', 'native-revoke'))
def test_forte_physical_arm_actual_join(gesture, withdrawal, monkeypatch):
  _exercise_forte_join(gesture, withdrawal, monkeypatch)
