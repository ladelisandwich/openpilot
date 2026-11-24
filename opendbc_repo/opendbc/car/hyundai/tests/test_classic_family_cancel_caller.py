from opendbc.car.hyundai.classic_long_aol import CLASSIC_LONG_IDS
from opendbc.car.hyundai.values import HyundaiFlags
import ast
import inspect
import textwrap
import pytest
import importlib
import os
from types import SimpleNamespace
from unittest.mock import patch

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.hyundai.interface import CarInterface
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

  assert qualified_classic(ci.CP, marked_only=True)
  packer = CANPacker('hyundai_can_generated')

  gas_name = 'E_EMS11' if ci.CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV) else 'EMS16'
  hz = {gas_name:100, 'ELECT_GEAR':20, 'WHL_SPD11':100, 'TCS13':100, 'MDPS12':50, 'CLU11':50,
        'BCM_PO_11':50, 'LKAS11':100}
  scheduled = {(packer.dbc.name_to_msg[name].address, 0): name for name in hz if name != 'LKAS11'}
  for parser in ci.can_parsers.values():
    for address in parser.addresses:
      scheduled[(address, parser.bus)] = parser.dbc.addr_to_msg[address].name
  drive_gear = next(code for code, label in ci.CS.shifter_values.items() if ci.CS.parse_gear_shifter(label) == structs.CarState.GearShifter.drive)
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
      values.update(CF_Clu_AliveCnt1=counter % 16, CF_Clu_CruiseSwState=button, CF_Clu_CruiseSwMain=main)
    elif name == 'EMS16':
      values.update(AliveCounter=counter % 4, CF_Ems_AclAct=0)
    elif name == 'E_EMS11':
      values['CR_Vcu_AccPedDep_Pos']=0
    elif name == 'ELECT_GEAR':
      values['Elect_Gear_Shifter']=drive_gear
    elif name == 'WHL_SPD11':
      values.update(WHL_SPD_AliveCounter_LSB=counter % 4, WHL_SPD_AliveCounter_MSB=(counter//4) % 4,
                    WHL_SPD_FL=72,WHL_SPD_FR=72,WHL_SPD_RL=72,WHL_SPD_RR=72)
    elif name == 'TCS13':
      values.update(AliveCounterTCS=counter % 8, ACCEnable=0, ACC_REQ=int(engaged), DriverOverride=0)
    elif name == 'MDPS12':
      values.update(CR_Mdps_StrColTq=0, CR_Mdps_OutTq=0)
    elif name == 'BCM_PO_11':
      values['LDA_BTN']=lkas
    elif name == 'CGW1':
      values['CF_Gway_DrvSeatBeltSw']=1
    elif name == 'LVR12':
      values['CF_Lvr_Gear']=drive_gear
    elif name == 'TCU12':
      values['CUR_GR'] = drive_gear
    elif name == 'SAS11':
      values['SAS_Angle']=1.
    elif name == 'CLU15':
      values.update(CF_Clu_VehicleSpeed=72, CF_Clu_Gear=drive_gear)
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


def family_cases():
  identities = []
  for identity in sorted(CLASSIC_LONG_IDS):
    if qualified_classic(classic_params(identity, lda=True)):
      identities.append(identity)
  assert len(identities) == 32
  return identities


@pytest.mark.parametrize('identity', family_cases())
def test_actual_classic_family_cancel_and_manual_withdrawal(identity, monkeypatch):
  withdrawal = 'toggle-off'
  expect_retention = True
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
      cp = classic_params(identity, lda=True)
      assert qualified_classic(cp) and not cp.pcmCruise and cp.openpilotLongitudinalControl
      assert cp.lateralTuning.which() == 'torque'
      expects_owner = True
      assert CarInterface.startup_required(cp) == expects_owner
      from opendbc.car import car_helpers, gen_empty_fingerprint
      from opendbc.car.hyundai import ecu_startup
      from opendbc.car.hyundai.tests.test_g90_continuity import ManualKeeper
      diagnostic_requests = []
      startup_owners = []
      native.set_alternative_experience(0)
      assert native.set_safety_hooks(structs.CarParams.SafetyModel.elm327, 1) == 0
      native.init_tests()
      startup_factory = CarInterface.startup_owner

      def traced_factory(prepared_cp, callbacks, *, requested):
        recv, send = callbacks

        def diagnostic_send(frames):
          for address, data, bus in frames:
            assert native.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data))
          if saved.get('CarParams') is None:
            assert not native.get_controls_allowed() and native.aol_get_permission_mask() == 0
          return send(frames)
        owner = startup_factory(prepared_cp, (recv, diagnostic_send), requested=requested)
        if owner is None:
          assert not expects_owner
          return None
        assert expects_owner
        # This fixture exercises the explicit eager startup/fallback API.
        # Default deferred takeover is covered by test_classic_deferred_takeover.
        owner.prepare_when_controls_ready = owner.prepare
        owner._query = startup_query
        owner.clock = lambda: now[0] / 1e9
        owner.keeper_factory = ManualKeeper
        startup_owners.append(owner)
        return owner

      def startup_query(sender, receiver, bus, addresses, requests, responses):
        request = requests[0]
        assert bus == 0 and addresses == [(0x7d0, None)]
        assert request in (b'\x10\x03', b'\x28\x83\x01')
        assert saved.get('CarParams') is None
        diagnostic_requests.append(request)

        def receive(timeout):
          sender([(0x7d0, bytes([len(request)]) + request + bytes(7 - len(request)), bus)])
          return {(0x7d0, None): b''} if request == b'\x10\x03' else {}
        return SimpleNamespace(get_data=receive)

      fp = gen_empty_fingerprint()
      fp[0][0x391] = 8
      identified = (identity, fp, '0' * 17, [], cp.fingerprintSource, True)
      initial_can = messaging.new_message('can', 1, valid=True, logMonoTime=now[0])
      initial_can.can[0].address, initial_can.can[0].src, initial_can.can[0].dat = 0x391, 0, bytes(8)
      with patch.object(car_helpers, 'fingerprint', return_value=identified), \
           patch.object(card_module.messaging, 'recv_one_retry', return_value=initial_can.as_reader()), \
           patch.object(CarInterface, 'startup_owner', side_effect=traced_factory), \
           patch.object(card_module.Car, 'startup_diagnostic_admission', return_value=True), \
           patch.object(ecu_startup, 'IsoTpParallelQuery', side_effect=startup_query), \
           patch('time.clock_gettime_ns', side_effect=lambda clock: now[0]), \
           patch('time.monotonic_ns', side_effect=lambda: now[0]), patch('time.sleep'):
        card = card_module.Car()
      if expects_owner:
        assert len(startup_owners) == 1 and card.vehicle_startup.owner is startup_owners[0]
        assert card.vehicle_startup.owner.published and card.vehicle_startup.owner.prepared_for(card.CP)
      else:
        assert not startup_owners and card.vehicle_startup.owner is None
      assert diagnostic_requests == ([b'\x10\x03', b'\x28\x83\x01'] if expects_owner else [])
      assert qualified_classic(card.CP, marked_only=True)
      expected_word = 0xC04
      expected_word |= 2 if card.CP.flags & HyundaiFlags.HYBRID else (1 if card.CP.flags & HyundaiFlags.EV else 0)
      expected_word |= 0x40 if card.CP.flags & HyundaiFlags.ALT_LIMITS else 0
      assert card.CP.safetyConfigs[0].safetyParam == expected_word and card.CP.alternativeExperience == 32
      assert isinstance(card.aol_card_intent, ClassicLongCardIntent)
      if not expect_retention:
        assert not card.aol_card_intent.independent_cruise_buttons, 'Published classic policy changed; reclassify baseline'
      assert not card.ci_initialized and not saved.get_bool('ControlsReady')
      card.slc_producer_session = 'authority-card'
      card.CI.update([])
      sd, controls, drive = case._authority_callers(False, cp=card.CP.as_reader().as_builder(), genuine_events=True,
        initial_panda_config=(int(structs.CarParams.SafetyModel.elm327), 1, 0,
                              bool(native.get_controls_allowed()), bool(native.safety_test_toyota_rx_summary_invalid())))

      sd.aol_settings = read_settings(saved)
      assert sd.aol_settings.lkas_action == AOL_TOGGLE and sd.aol_settings.main_action == 0
      assert sd.CP.to_dict() == card.CP.to_dict() == controls.CP.to_dict()
      assert len(card.CP.safetyConfigs) == 1 and bool(card.CP.openpilotLongitudinalControl) == shared_long
      config = card.CP.safetyConfigs[0]
      model, word = int(config.safetyModel.raw), int(config.safetyParam)
      configured = [False]
      dispatch_tree = ast.parse(textwrap.dedent(inspect.getsource(card_module.Car.step))).body[0].body
      assignments = [node for node in dispatch_tree if isinstance(node, ast.Assign) and
                     any(isinstance(t, ast.Name) and t.id == 'initialized' for t in node.targets)]
      decisions = [node for node in dispatch_tree if isinstance(node, ast.If) and ast.unparse(node.test) == 'not self.CP.passive and initialized']
      assert len(assignments) == len(decisions) == 1
      dispatch = compile(ast.Module(body=assignments + decisions, type_ignores=[]), 'actual-Card-step-control-dispatch', 'exec')
      queued, received, previous_sd = [], {}, {}
      tick = 0
      next_panda_tick = [1]
      patch_scope = monkeypatch.context()
      local = patch_scope.__enter__()
      try:

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
        original_receive, original_send = card.can_callbacks

        def checked_diagnostic_send(frames):
          for address, data, bus in frames:
            assert native.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data))
          return original_send(frames)
        card.can_callbacks = (original_receive, checked_diagnostic_send)
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
          if saved.get_bool('ControlsReady') and not configured[0]:
            native.set_alternative_experience(card.CP.alternativeExperience)
            assert native.set_safety_hooks(model, word) == 0
            native.init_tests()
            native.set_alternative_experience(card.CP.alternativeExperience)
            configured[0] = True
          native.set_timer(source_ns // 1000)
          if expects_owner:
            card.vehicle_startup.owner.keeper.tick()
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
          panda.safetyModel = config.safetyModel if configured[0] else structs.CarParams.SafetyModel.elm327
          panda.safetyParam = word if configured[0] else 1
          panda.alternativeExperience = card.CP.alternativeExperience if configured[0] else 0
          panda.controlsAllowed = native.get_controls_allowed()
          panda.safetyRxChecksInvalid = rx_invalid
          panda.ignitionLine = True
          actual_model = model if configured[0] else int(structs.CarParams.SafetyModel.elm327)
          actual_word = word if configured[0] else 1
          actual_experience = card.CP.alternativeExperience if configured[0] else 0
          slot = PandaSlot(0, 'authority-panda', actual_model, actual_word, actual_experience,
                           native.get_controls_allowed(), safetyRxChecksInvalid=rx_invalid)
          wire = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=source_ns)
          wire.aolSafetyWire = encode_safety(SafetyState(1, configured[0], source_ns, source_ns + 30_000_000,
            actual_model, actual_word, bool(mask & 1), bool(mask & 2), bool(actual_request & 1), bool(actual_request & 2),
            'authority-panda', card.aol_axis_session(), source_ns, (slot,)))
          messages = [device.as_reader(), wire.as_reader()]
          if tick >= next_panda_tick[0]:
            messages.append(pandas.as_reader())
            next_panda_tick[0] = tick + 10
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
            panda_fault=rx_invalid, relay_fault=relay_fault, panda_state_message=pandas, safety_message=wire)
          if sd.conditional_car_state_valid:
            assert sd.aol_car_state_log_ns == state_envelope.logMonoTime
          assert controls.sm.logMonoTime['carState'] == state_envelope.logMonoTime
          assert controls.sm.valid['carState'] == state_envelope.valid
          assert controls.sm['carState'].to_dict() == state_envelope.carState.to_dict()
          assert controls.sm.logMonoTime['carOutput'] == output_envelope.logMonoTime
          assert controls.sm.valid['carOutput'] == output_envelope.valid
          assert controls.sm['carOutput'].to_dict() == output_envelope.carOutput.to_dict()
          assert bytes(controls.sm['aolSafetyWire']) == bytes(wire.aolSafetyWire)
          for consumer in (unit for unit in (card, sd, controls) if 'pandaStates' in unit.sm.services):
            observed = consumer.sm['pandaStates'][0]
            assert int(observed.safetyModel.raw) in (int(structs.CarParams.SafetyModel.elm327), model)
            if int(observed.safetyModel.raw) == int(structs.CarParams.SafetyModel.elm327):
              assert observed.safetyParam == 1 and observed.alternativeExperience == 0
            else:
              assert configured[0] and observed.safetyParam == word and observed.alternativeExperience == card.CP.alternativeExperience
          previous_sd.update(sd.pm.messages)
          validated = current_native(sd.sm, sd.CP, now_ns=source_ns, axis_session_id=sd.aol_session_id)
          if validated is not None:
            assert configured[0] and not rx_invalid
            assert validated.lateralAllowed == bool(mask & 1)
            assert validated.requestedLateral == bool(actual_request & 1)
          else:
            assert not axis.lateralActive and not axis.longitudinalActive
            assert not command.latActive and not command.longActive
          if not configured[0]:
            assert validated is None

          now[0] += 1
          card.sm.update_msgs(now[0] / 1e9, [controls.pm.messages['carControl']])
          received.pop('sendcan', None)
          exec(dispatch, dict(vars(card_module), self=card, CS=state))
          steering = []
          if state.canValid and card.ci_initialized and not configured[0] and 'sendcan' in received:
            assert not command.enabled and not command.latActive and not command.longActive
            assert command.actuators.torque == 0 and command.actuators.accel == 0
            for frame in received['sendcan'].sendcan:
              accepted = bool(native.safety_tx_hook(libsafety_py.make_CANPacket(frame.address, frame.src, frame.dat)))
              assert accepted == (frame.address == 0x7d0 and len(frame.dat) == 8)
          if state.canValid and card.ci_initialized and configured[0] and 'sendcan' in received and received['sendcan'].valid:
            steering = [frame for frame in received['sendcan'].sendcan
                        if frame.address == 0x340 and frame.src == 0]
            assert steering
            for frame in received['sendcan'].sendcan:
              assert native.safety_tx_hook(libsafety_py.make_CANPacket(frame.address, frame.src, frame.dat))
          if 'sendcan' in received and not received['sendcan'].valid:
            assert len(received['sendcan'].sendcan) == 0
            assert not command.enabled and not command.latActive and not command.longActive
            assert command.actuators.torque == 0 and command.actuators.accel == 0
          return state, axis, host, command, steering

        for _ in range(120):
          step()
        assert card.ci_initialized and configured[0] and saved.get_bool('ControlsReady'), 'ORDINARY_CLASSIC_NATIVE_BOOTSTRAP_STALLED'
        if expects_owner:
          assert card.vehicle_startup.owner.handed_off
        assert not card.aol_card_intent.allowed_latch
        assert native.safety_config_valid()
        for _ in range(2):
          step(button=2)
        for _ in range(20):
          state, axis, host, command, _ = step(engaged=True)
        assert state.cruiseState.enabled and host.enabled and native.get_controls_allowed()
        assert axis.lateralActive and command.latActive
        if shared_long:
          assert axis.longitudinalActive and command.longActive

        assert card.aol_card_intent.allowed_latch, 'FULL_ENGAGEMENT_DID_NOT_KEEP_AOL'
        assert card.aol_card_intent._last_latch_edge_ns > 0
        edge = card.aol_card_intent._last_latch_edge_ns
        assert diagnostic_requests == [b'\x10\x03', b'\x28\x83\x01']
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
        cancel_observed = False
        for _cancel_index in range(2):
          state, axis, host, command, steering = step(main=int(incidental == 'main'), button=4 if incidental == 'cancel' else 0)
          assert card.aol_card_intent.allowed_latch, (
            'FAMILY_CANCEL_INTENT_DROPPED', identity, word, native.get_controls_allowed(), native.aol_get_permission_mask())
          assert native.aol_get_permission_mask() & 1, 'HKG_NATIVE_LATERAL_DROPPED'
          assert axis.lateralActive and command.latActive, 'HKG_JOINED_LATERAL_DROPPED'
          if any(event.type == structs.CarState.ButtonEvent.Type.cancel and event.pressed for event in state.buttonEvents):
            cancel_observed = True
            assert not axis.longitudinalActive and not command.longActive and not host.enabled
        assert cancel_observed
        nonzero = False
        for _ in range(20):
          state, axis, host, command, steering = step()
          assert card.aol_card_intent.allowed_latch, (
            'FAMILY_CANCEL_INTENT_DROPPED', identity, word, native.get_controls_allowed(), native.aol_get_permission_mask())
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
