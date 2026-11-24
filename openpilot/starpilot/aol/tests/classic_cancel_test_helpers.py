from openpilot.starpilot.aol.tests.classic_startup_test_helpers import prepared_classic_card





import importlib
import os
from types import SimpleNamespace
from unittest.mock import patch

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR
from opendbc.car.hyundai.classic_long_aol import qualified as qualified_classic
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.test_hyundai import checksum
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car import card as card_module
from openpilot.starpilot.aol.intent import AOL_TOGGLE, read_settings
from openpilot.starpilot.aol.runtime import current_native
from openpilot.starpilot.aol.tests.test_classic_long_profiles import params as classic_params
from openpilot.starpilot.aol.tests import test_ioniq_availability as caller_fixture
from openpilot.starpilot.aol.wire import PandaSlot, SafetyState, encode_safety
from openpilot.starpilot.car.hyundai.classic_long_intent import ClassicLongCardIntent

disable_ecu_module = importlib.import_module('opendbc.car.disable_ecu')


def classic_frames(ci, tick, *, main=0, lkas=0, button=0, engaged=False):

  assert ci.CP.carFingerprint == CAR.HYUNDAI_SONATA
  packer = CANPacker('hyundai_can_generated')


  hz = {'EMS16':100, 'WHL_SPD11':100, 'TCS13':100, 'MDPS12':50, 'CLU11':50,
        'BCM_PO_11':50, 'LKAS11':100}
  scheduled = {(packer.dbc.name_to_msg[name].address, 0): name for name in hz if name != 'LKAS11'}
  for parser in ci.can_parsers.values():
    for address in parser.addresses:
      scheduled[(address, parser.bus)] = parser.dbc.addr_to_msg[address].name
  frames = []
  for (address,bus),name in sorted(scheduled.items()):
    parser = next((p for p in ci.can_parsers.values() if p.bus == bus), None)
    declared = parser.message_states[address].frequency if parser is not None and address in parser.addresses else 0
    frequency = hz.get(name, min(declared,100) if 0 < declared <= 100 else 100)
    period = max(1,round(100/frequency))
    if tick % period:
      continue
    counter = tick // period
    values = {}
    if name == 'CLU11':
      values.update(CF_Clu_AliveCnt1=counter%16, CF_Clu_CruiseSwState=button, CF_Clu_CruiseSwMain=main)
    elif name == 'EMS16':
      values.update(AliveCounter=counter%4, CF_Ems_AclAct=0)
    elif name == 'WHL_SPD11':
      values.update(WHL_SPD_AliveCounter_LSB=counter%4, WHL_SPD_AliveCounter_MSB=(counter//4)%4,
                    WHL_SPD_FL=40,WHL_SPD_FR=40,WHL_SPD_RL=40,WHL_SPD_RR=40)
    elif name == 'TCS13':
      values.update(AliveCounterTCS=counter%8, ACCEnable=0, ACC_REQ=int(engaged), DriverOverride=0)
    elif name == 'MDPS12':
      values.update(CR_Mdps_StrColTq=0, CR_Mdps_OutTq=0)
    elif name == 'BCM_PO_11':
      values['LDA_BTN']=lkas
    elif name == 'CGW1':
      values['CF_Gway_DrvSeatBeltSw']=1
    elif name == 'LVR12':
      values['CF_Lvr_Gear']=5
    elif name == 'SAS11':
      values['SAS_Angle']=1.
    elif name == 'CLU15':
      values['CF_Clu_VehicleSpeed']=40
    frame = packer.make_can_msg(name,bus,values)
    if name in ('EMS16','WHL_SPD11','TCS13'):
      address, data, bus = checksum(frame)
      frame = (address, bytes(data), bus)
    frames.append(frame)
  assert all(address not in (0x420,0x421) for address,_,_ in frames), 'Disabled SCC source must not be fabricated'
  assert all(address!=0x340 or bus==2 for address,_,bus in frames), 'Never echo outgoing LKAS on protected RX bus'
  return frames


def _invalid_controls_emits_no_commands(received):
  if 'sendcan' not in received:
    return True
  envelope = received['sendcan']
  return envelope.which() == 'sendcan' and not envelope.valid and len(envelope.sendcan) == 0


def _exercise_classic_join(withdrawal, monkeypatch, *, expect_retention):
  shared_long, incidental = True, 'cancel'
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
      saved.put_bool('AlphaLongitudinalEnabled', True, block=True)
      saved.put_bool('IsReleaseBranch', False, block=True)
      cp = classic_params(CAR.HYUNDAI_SONATA, lda=True)
      assert qualified_classic(cp) and not cp.pcmCruise and cp.openpilotLongitudinalControl
      assert cp.lateralTuning.which() == 'torque' and CarInterface.startup_required(cp)
      card = prepared_classic_card(cp)
      assert qualified_classic(card.CP, marked_only=True)
      assert card.CP.safetyConfigs[0].safetyParam == 0xC04 and card.CP.alternativeExperience == 32
      assert isinstance(card.aol_card_intent, ClassicLongCardIntent)
      if not expect_retention:
        assert not card.aol_card_intent.independent_cruise_buttons, 'Published classic policy changed; reclassify baseline'
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
      native.set_alternative_experience(card.CP.alternativeExperience)
      assert native.set_safety_hooks(model, word) == 0
      native.init_tests()
      native.set_alternative_experience(card.CP.alternativeExperience)
      queued, received, previous_sd = [], {}, {}
      tick = 0
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

        def step(*, main=0, lkas=0, button=0, engaged=False, withdraw_request=False):
          nonlocal tick
          tick += 1
          now[0] = source_ns = 10_000_000_000 + tick * 10_000_000
          prior_axis = previous_sd.get('aolAxisState')
          prior_state = previous_sd.get('selfdriveState')
          request = 0 if prior_axis is None else (
            int(prior_axis.aolAxisState.desiredLateral) | int(prior_axis.aolAxisState.desiredLongitudinal) << 1)
          if withdraw_request:
            request = 0
          heartbeat = bool(request or (prior_state is not None and prior_state.selfdriveState.enabled))
          native.set_timer(source_ns // 1000)
          native.set_aol_test_heartbeat(heartbeat)
          native.aol_set_host_request(request)
          frames = classic_frames(card.CI, tick, main=main, lkas=lkas, button=button, engaged=engaged)
          for address, data, bus in frames:
            assert native.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data)), (hex(address), bus)
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
          device.deviceState.started, device.deviceState.startedMonoTime = True, 9_000_000_000
          pandas = messaging.new_message('pandaStates', 1, valid=True, logMonoTime=source_ns)
          panda = pandas.pandaStates[0]
          panda.safetyModel, panda.safetyParam = config.safetyModel, word
          panda.alternativeExperience = card.CP.alternativeExperience
          panda.controlsAllowed = native.get_controls_allowed()
          panda.safetyRxChecksInvalid = rx_invalid
          panda.ignitionLine = True
          slot = PandaSlot(0, 'authority-panda', model, word, card.CP.alternativeExperience,
                           native.get_controls_allowed(), safetyRxChecksInvalid=rx_invalid)
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
          if not rx_invalid:
            assert validated is not None and validated.lateralAllowed == bool(mask & 1)
            assert validated.requestedLateral == bool(actual_request & 1)
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
                        if frame.address == 0x340 and frame.src == 0]
            assert steering
            for frame in received['sendcan'].sendcan:
              assert native.safety_tx_hook(libsafety_py.make_CANPacket(frame.address, frame.src, frame.dat))
          return state, axis, host, command, steering

        for _ in range(120):
          step()
        assert not card.aol_card_intent.allowed_latch
        assert native.safety_config_valid()
        for _ in range(2):
          step(lkas=1)
        for _ in range(15):
          state, axis, host, command, _ = step()
        assert state.canValid and axis.lateralActive and command.latActive
        assert not host.enabled and not command.longActive
        for _ in range(2):
          step(button=2)
        for _ in range(20):
          state, axis, host, command, _ = step(engaged=True)
        assert state.cruiseState.enabled and host.enabled and native.get_controls_allowed()
        assert axis.lateralActive and command.latActive
        if shared_long:
          assert axis.longitudinalActive and command.longActive


        edge = card.aol_card_intent._last_latch_edge_ns
        assert diagnostic_requests == []
        assert card._classic_startup_requests == (b'\x10\x03', b'\x28\x83\x01')
        assert card.ci_initialized and saved.get_bool('ControlsReady')
        if not expect_retention:
          for _ in range(2):
            step(button=4)
          for _ in range(5):
            state, axis, host, command, _ = step()
          assert not card.aol_card_intent.allowed_latch
          assert not axis.lateralActive and not command.latActive
          assert not axis.longitudinalActive and not command.longActive and not host.enabled
          assert not native.get_controls_allowed() and native.aol_get_permission_mask() == 0
          return
        for _ in range(2):
          state, axis, host, command, _ = step(main=int(incidental == 'main'), button=4 if incidental == 'cancel' else 0)
          assert card.aol_card_intent.allowed_latch, 'CLASSIC_HELD_CANCEL_LATCH_DROPPED'
          assert native.aol_get_permission_mask() & 1, 'HKG_NATIVE_LATERAL_DROPPED'
          assert axis.lateralActive and command.latActive, 'HKG_JOINED_LATERAL_DROPPED'
          assert not axis.longitudinalActive and not command.longActive and not host.enabled
        nonzero = False
        for _ in range(20):
          state, axis, host, command, steering = step()
          assert card.aol_card_intent.allowed_latch, 'CLASSIC_HELD_CANCEL_LATCH_DROPPED'
          assert card.aol_card_intent._last_latch_edge_ns == edge, 'HKG_INCIDENTAL_EDGE_RENEWED'
          assert native.aol_get_permission_mask() & 1, 'HKG_NATIVE_LATERAL_DROPPED'
          assert axis.lateralActive and command.latActive, 'HKG_JOINED_LATERAL_DROPPED'
          assert not state.cruiseState.enabled and not host.enabled and not native.get_controls_allowed()
          assert not axis.longitudinalActive and not command.longActive
          nonzero |= abs(command.actuators.torque) > 0
          assert steering
        if shared_long:


          for _ in range(2):
            state, axis, host, command, _ = step(main=1)
          assert card.aol_card_intent.allowed_latch and axis.lateralActive and command.latActive
          assert not host.enabled and not command.longActive and not native.get_controls_allowed()
          for _ in range(20):
            state, axis, host, command, _ = step()
            assert card.aol_card_intent.allowed_latch and axis.lateralActive and command.latActive
            assert card.aol_card_intent._last_latch_edge_ns == edge
            assert not host.enabled and not axis.longitudinalActive and not command.longActive
            assert not native.get_controls_allowed()
        assert nonzero, 'Actual Controls did not produce nonzero correction for measured steering error'
        invalid = controls.pm.messages['carControl'].as_builder()
        invalid.valid = False
        now[0] += 1
        card.sm.update_msgs(now[0] / 1e9, [invalid.as_reader()])
        received.pop('sendcan', None)
        card.controls_update(state, card.sm['carControl'])


        assert _invalid_controls_emits_no_commands(received), 'Invalid Controls packet produced valid or nonempty Card sendcan'
        assert _invalid_controls_emits_no_commands({})
        empty = messaging.new_message('sendcan', 0, valid=False)
        assert _invalid_controls_emits_no_commands({'sendcan': empty.as_reader()})
        for valid, frame_count in ((True, 0), (False, 1), (True, 1)):
          rejected = messaging.new_message('sendcan', frame_count, valid=valid)
          assert not _invalid_controls_emits_no_commands({'sendcan': rejected.as_reader()})
        if withdrawal == 'toggle-off':
          for _ in range(2):
            step(lkas=1)
        for _ in range(3):
          state, axis, host, command, _ = step(withdraw_request=withdrawal == 'native-request-off')
        assert not axis.lateralActive and not command.latActive
        assert not axis.longitudinalActive and not command.longActive and not host.enabled
        if withdrawal == 'toggle-off':
          assert not card.aol_card_intent.allowed_latch
        assert native.aol_get_permission_mask() == 0
      finally:
        patch_scope.__exit__(None, None, None)
    finally:
      native.init_tests()
      native.set_alternative_experience(0)
      native.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)
      case.doCleanups()
      if card is not None:
        card.vehicle_startup.close()

