"""Prepared classic LONG requires neutral SCC while perception initializes."""
import os
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from opendbc.car import structs
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.radar_interface import RadarInterface
from opendbc.car.hyundai.values import HyundaiFlags, CAR
from opendbc.car.hyundai.classic_long_aol import qualified as qualified_classic
from opendbc.car.hyundai.tests.test_classic_family_startup_caller import family_cases
from opendbc.car.hyundai.tests.test_g90_continuity import ManualKeeper
from opendbc.can import CANPacker, CANParser
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.test_hyundai import checksum
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car import card as card_module
from openpilot.starpilot.aol.tests.test_classic_long_profiles import params as classic_params

def neutral_frames(ci, tick, *, main=0, lkas=0, button=0, engaged=False):

  assert qualified_classic(ci.CP)
  packer = CANPacker('hyundai_can_generated')

  gas_name = 'E_EMS11' if ci.CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV) else 'EMS16'
  hz = {gas_name: 100, 'ELECT_GEAR': 20, 'WHL_SPD11': 100, 'TCS13': 100, 'MDPS12': 50, 'CLU11': 50, 'BCM_PO_11': 50, 'LKAS11': 100}
  scheduled = {(packer.dbc.name_to_msg[name].address, 0): name for name in hz if name != 'LKAS11'}
  for parser in ci.can_parsers.values():
    for address in parser.addresses:
      scheduled[(address, parser.bus)] = parser.dbc.addr_to_msg[address].name
  drive_gear = next(code for code, label in ci.CS.shifter_values.items() if ci.CS.parse_gear_shifter(label) == structs.CarState.GearShifter.drive)
  frames = []
  for (address, bus), name in sorted(scheduled.items()):
    parser = next((p for p in ci.can_parsers.values() if p.bus == bus), None)
    declared = parser.message_states[address].frequency if parser is not None and address in parser.addresses else 0
    frequency = hz.get(name, min(declared, 100) if 0 < declared <= 100 else 100)
    period = max(1, round(100 / frequency))
    if tick % period:
      continue
    counter = tick // period
    values = {}
    if name == 'CLU11':
      values.update(CF_Clu_AliveCnt1=counter % 16, CF_Clu_CruiseSwState=button, CF_Clu_CruiseSwMain=main)
    elif name == 'EMS16':
      values.update(AliveCounter=counter % 4, CF_Ems_AclAct=0)
    elif name == 'E_EMS11':
      values['CR_Vcu_AccPedDep_Pos'] = 0
    elif name == 'ELECT_GEAR':
      values['Elect_Gear_Shifter'] = drive_gear
    elif name == 'WHL_SPD11':
      values.update(
        WHL_SPD_AliveCounter_LSB=counter % 4, WHL_SPD_AliveCounter_MSB=(counter // 4) % 4, WHL_SPD_FL=72, WHL_SPD_FR=72, WHL_SPD_RL=72, WHL_SPD_RR=72
      )
    elif name == 'TCS13':
      values.update(AliveCounterTCS=counter % 8, ACCEnable=0, ACC_REQ=int(engaged), DriverOverride=0)
    elif name == 'MDPS12':
      values.update(CR_Mdps_StrColTq=0, CR_Mdps_OutTq=0)
    elif name == 'BCM_PO_11':
      values['LDA_BTN'] = lkas
    elif name == 'CGW1':
      values['CF_Gway_DrvSeatBeltSw'] = 1
    elif name == 'LVR12':
      values['CF_Lvr_Gear'] = drive_gear
    elif name == 'TCU12':
      values['CUR_GR'] = drive_gear
    elif name == 'SAS11':
      values['SAS_Angle'] = 1.0
    elif name == 'CLU15':
      values.update(CF_Clu_VehicleSpeed=72, CF_Clu_Gear=drive_gear)
    frame = packer.make_can_msg(name, bus, values)
    if name in ('EMS16', 'WHL_SPD11', 'TCS13'):
      address, data, bus = checksum(frame)
      frame = (address, bytes(data), bus)
    frames.append(frame)
  assert all(address not in (0x420, 0x421) for address, _, _ in frames), 'Disabled SCC source must not be fabricated'
  assert all(address != 0x340 or bus == 2 for address, _, bus in frames), 'Never echo outgoing LKAS on protected RX bus'
  return frames



@pytest.mark.parametrize('identity', family_cases()[:16])
@pytest.mark.parametrize('aol', [False, True])
def test_classic_neutral_replacement_first_half(identity, aol):
  run_neutral(identity, aol, 'normal')


@pytest.mark.parametrize('identity', family_cases()[16:])
@pytest.mark.parametrize('aol', [False, True])
def test_classic_neutral_replacement_second_half(identity, aol):
  run_neutral(identity, aol, 'normal')


def test_recorded_g90_startup_replacement_gap():
  run_neutral(CAR.GENESIS_G90, True, 'normal')


@pytest.mark.parametrize('variation', ['active', 'stale', 'native_mismatch', 'native_fault', 'invalid_can', 'changed_cp', 'source_missing'])
def test_classic_initializing_never_bypasses_current_health(variation):
  run_neutral(CAR.GENESIS_G90, True, variation)


def run_neutral(identity, aol, variation):
  native = libsafety_py.libsafety
  now = [9_000_000_000]
  card = None
  owner = None
  with OpenpilotPrefix(), patch.dict(os.environ, {'SIMULATION': '1', 'AOL_REPLAY_RUNTIME': '0'}):
    saved = Params()
    saved.put_bool('OpenpilotEnabledToggle', True, block=True)
    saved.put_bool('AlwaysOnLateral', aol, block=True)
    cp = classic_params(identity, lda=True)
    # Startup ownership is selected before Card adds the AOL safety marker.
    cp.safetyConfigs[0].safetyParam &= ~0xC00
    cp.alternativeExperience = 0
    owner = CarInterface.startup_owner(cp, (list, lambda frames: None), requested=True)
    assert owner is not None
    owner.clock = lambda: now[0] / 1e9
    owner.keeper_factory = ManualKeeper
    requests = []

    def query(sender, receiver, bus, addresses, commands, responses):
      command = commands[0]
      requests.append(command)
      assert bus == 0 and addresses == [(0x7D0, None)]
      def receive(timeout):
        sender([(0x7D0, bytes([len(command)]) + command + bytes(7 - len(command)), 0)])
        return {} if command == b'\x28\x83\x01' else {(0x7D0, None): b''}
      return SimpleNamespace(get_data=receive)

    try:
      with patch.object(owner, '_query', side_effect=query), patch('time.clock_gettime_ns', side_effect=lambda clock: now[0]), patch('time.sleep'):
        owner.prepare(admission=lambda: True)
        ci = CarInterface(owner.cp)
        card = card_module.Car(CI=ci, RI=RadarInterface(ci.CP), startup_owner=owner)
      assert owner.published and owner.prepared_for(card.CP)
      assert requests == [b'\x10\x03', b'\x28\x83\x01']
      assert not saved.get_bool('ControlsReady') and not card.ci_initialized
      card.CI.update([])
      native.set_alternative_experience(0)
      assert native.set_safety_hooks(structs.CarParams.SafetyModel.elm327, 1) == 0
      native.init_tests()
      emitted = []
      original_send = card.publish_sendcan
      def capture(frames, valid=True):
        emitted.extend((now[0], frame, valid) for frame in frames)
        original_send(frames, valid=valid)
      write_bool = card.params.put_bool
      def persist_readiness(key, value, block=False):
        # Persist Card's actual readiness request before advancing simulated time.
        # The real asynchronous writer otherwise races the fixture's virtual clock.
        return write_bool(key, value, block=block or key == 'ControlsReady')
      with patch.object(card, 'publish_sendcan', side_effect=capture), \
           patch.object(card.params, 'put_bool', side_effect=persist_readiness):
        for tick in range(620):
          now[0] = stamp = 10_000_000_000 + tick * 10_000_000
          native.set_timer(stamp // 1000)
          if saved.get_bool('ControlsReady') and native.get_current_safety_mode() != int(card.CP.safetyConfigs[0].safetyModel.raw):
            native.set_alternative_experience(card.CP.alternativeExperience)
            assert native.set_safety_hooks(card.CP.safetyConfigs[0].safetyModel.raw, card.CP.safetyConfigs[0].safetyParam) == 0
            native.init_tests()
          frames = neutral_frames(card.CI, tick)
          if variation == 'source_missing':
            frames = [frame for frame in frames if frame[0] != 0x251]
          for address, data, bus in frames:
            assert native.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data))
          if tick % 100 == 0:
            native.safety_tick()
          batch = messaging.log_from_bytes(card_module.can_list_to_can_capnp(frames)).as_builder()
          batch.logMonoTime = stamp
          control = messaging.new_message('carControl', valid=True, logMonoTime=stamp - (200_000_000 if variation == 'stale' else 0))
          if variation == 'active':
            control.carControl.enabled = control.carControl.latActive = True
            control.carControl.actuators.torque = .1
          events = messaging.new_message('onroadEvents', 1, valid=True, logMonoTime=stamp)
          events.onroadEvents[0].name = 'selfdriveInitializing'
          events.onroadEvents[0].noEntry = True
          pandas = messaging.new_message('pandaStates', 1, valid=True, logMonoTime=stamp)
          panda = pandas.pandaStates[0]
          panda.safetyModel = native.get_current_safety_mode()
          panda.safetyParam = native.get_current_safety_param() ^ int(variation == 'native_mismatch')
          panda.alternativeExperience = card.CP.alternativeExperience if saved.get_bool('ControlsReady') else 0
          panda.safetyRxChecksInvalid = bool(native.safety_test_toyota_rx_summary_invalid())
          panda.ignitionLine = True
          if variation == 'native_fault':
            panda.faults = ['relayMalfunction']
          if variation == 'changed_cp':
            card.CP.mass += 1
          card.sm.update_msgs(stamp / 1e9, [control.as_reader(), events.as_reader(), pandas.as_reader()])
          with patch.object(card_module, 'REPLAY', False), patch.object(card_module.messaging, 'drain_sock_raw', return_value=[batch.to_bytes()]), \
               patch.object(card_module.time, 'monotonic_ns', return_value=stamp), patch.object(card_module.time, 'monotonic', return_value=stamp / 1e9), \
               patch.object(card_module.time, 'clock_gettime_ns', return_value=stamp), patch.object(card.sm, 'update'):
            if variation == 'invalid_can':
              original_update = card.CI.update
              def invalid(*args, updater=original_update, **kwargs):
                state = updater(*args, **kwargs)
                state.canValid = False
                return state
              with patch.object(card.CI, 'update', side_effect=invalid):
                card.step()
            else:
              card.step()
          for _, (address, data, bus), valid in emitted[-30:]:
            assert valid and native.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data))
      if variation != 'normal':
        assert not emitted
        return
      assert card.ci_initialized and saved.get_bool('ControlsReady')
      assert owner.handed_off
      scc = [(stamp, frame) for stamp, frame, _ in emitted if frame[0] == 0x421]
      assert len(scc) >= 300 and scc[0][0] <= 10_100_000_000
      assert max(b[0] - a[0] for a, b in zip(scc, scc[1:], strict=False)) <= 30_000_000
      parser = CANParser('hyundai_can_generated', [('SCC12', 0)], 0)
      for stamp, frame in scc:
        parser.update([(stamp, [frame])])
        values = parser.vl['SCC12']
        assert values['ACCMode'] == 0 and values['aReqRaw'] == 0 and values['aReqValue'] == 0
      assert not native.get_controls_allowed() and native.aol_get_permission_mask() == 0
    finally:
      if card is not None:
        card.vehicle_startup.close()
      elif owner is not None:
        owner.published = True
        owner.close()
      native.init_tests()
      native.set_alternative_experience(0)
      native.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)
