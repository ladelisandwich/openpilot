import os
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from opendbc.can import CANPacker
from opendbc.car import CanData, structs
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.ioniq6_handoff import TimestampedCanPacket
from opendbc.car.hyundai.hyundaicanfd import hkg_can_fd_checksum
from opendbc.car.hyundai.radar_interface import RadarInterface
from opendbc.car.hyundai.tests.test_g90_continuity import ManualKeeper
from opendbc.car.hyundai.tests.test_radar_hda1_startup_lifecycle import physical_frames as fd_frames
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.test_hyundai import checksum
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car.card import Car
from openpilot.starpilot.aol.tests.test_classic_long_profiles import params as classic_params
from openpilot.starpilot.tests.test_ev6_startup import params as ev_params

CARS = (CAR.GENESIS_G90, CAR.KIA_EV6, CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN)


def classic_frames(ci, tick):
  frames = []
  gear = next(code for code, label in ci.CS.shifter_values.items() if label == "D")
  for parser in ci.can_parsers.values():
    packer = CANPacker(parser.dbc.name)
    for address in parser.addresses:
      message = parser.dbc.addr_to_msg[address]
      hz = parser.message_states[address].frequency
      period = max(1, round(100 / hz)) if hz else 1
      if tick % period:
        continue
      counter = tick // period
      values = {}
      if message.name == "EMS16":
        values["AliveCounter"] = counter % 4
      elif message.name == "WHL_SPD11":
        values.update(
          WHL_SPD_AliveCounter_LSB=counter % 4, WHL_SPD_AliveCounter_MSB=(counter // 4) % 4, WHL_SPD_FL=36, WHL_SPD_FR=36, WHL_SPD_RL=36, WHL_SPD_RR=36
        )
      elif message.name == "TCS13":
        values.update(AliveCounterTCS=counter % 8, ACCEnable=0, ACC_REQ=0)
      elif message.name == "CLU11":
        values["CF_Clu_AliveCnt1"] = counter % 16
      elif message.name == "LKAS11":
        values["CF_Lkas_MsgCount"] = counter % 16
      elif message.name == "CGW1":
        values["CF_Gway_DrvSeatBeltSw"] = 1
      elif message.name == "LVR12":
        values["CF_Lvr_Gear"] = gear
      elif message.name == "TCU12":
        values["CUR_GR"] = gear
      elif message.name == "CLU15":
        values["CF_Clu_Gear"] = gear
      frame = packer.make_can_msg(message.name, parser.bus, values)
      if message.name in ("EMS16", "WHL_SPD11", "TCS13"):
        address, data, bus = checksum(frame)
        frame = (address, bytes(data), bus)
      frames.append(frame)
  return frames


@pytest.mark.parametrize(
  "identity,initialize_only,pandas_count",
  [(identity, False, 1) for identity in CARS]
  + [(CAR.KIA_EV6, True, 1), (CAR.KIA_EV6, False, 2), (CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN, False, 2), (CAR.KIA_EV6, True, 2)],
  ids=["g90", "ev6-1p", "gv70-1p", "ev6-init-1p", "ev6-2p", "gv70-2p", "ev6-init-2p"],
)
@pytest.mark.parametrize("scenario", ["panda", "command", "sources", "healthy"])
def test_actual_card_only_publishes_prepared_takeover_with_current_owner_evidence(identity, initialize_only, pandas_count, scenario):
  now = [10_000_000_000]
  native = libsafety_py.libsafety
  card = None
  with OpenpilotPrefix(), patch.dict(os.environ, {"SIMULATION": "0", "AOL_REPLAY_RUNTIME": "1"}):
    saved = Params()
    for key, value in [("OpenpilotEnabledToggle", True), ("AlwaysOnLateral", False), ("AlphaLongitudinalEnabled", True)]:
      saved.put_bool(key, value, block=True)
    cp = classic_params(identity, lda=True) if identity == CAR.GENESIS_G90 else ev_params(car=identity, radar=True)
    if pandas_count == 2:
      active_config = cp.safetyConfigs[-1].to_dict()
      cp.safetyConfigs = [structs.CarParams.SafetyConfig(safetyModel=structs.CarParams.SafetyModel.noOutput), active_config]
    offset = 4 * (pandas_count - 1)
    native.set_alternative_experience(0)
    assert native.set_safety_hooks(structs.CarParams.SafetyModel.elm327, 1) == 0
    native.init_tests()
    pending = []

    def send(frames):
      assert not native.get_controls_allowed() and native.aol_get_permission_mask() == 0
      for address, data, bus in frames:
        assert native.safety_tx_hook(libsafety_py.make_CANPacket(address, bus - offset, data))
      pending.extend(frames)

    def recv(*args, **kwargs):
      now[0] += 10_000_000
      data = bytearray(32)
      data[2], data[4] = now[0] // 10_000_000 % 256, 7
      data[:2] = hkg_can_fd_checksum(0x51, None, data).to_bytes(2, "little")
      return [TimestampedCanPacket([CanData(0x51, bytes(data), getattr(owner, "template_bus", 0))], now[0])]

    owner = CarInterface.startup_owner(cp, (recv, send), requested=True)
    assert owner is not None
    owner.keeper_factory, owner.clock = ManualKeeper, lambda: now[0] / 1e9

    def query(sender, receiver, bus, addresses, commands, responses):
      command = commands[0]

      def data(timeout):
        sender([(owner.address, bytes([len(command)]) + command + bytes(7 - len(command)), owner.bus)])
        return {} if command == b"\x28\x83\x01" else {(owner.address, None): b""}

      return SimpleNamespace(get_data=data)

    owner._query = query

    def sleep(seconds):
      now[0] += round(seconds * 1e9)

    try:
      with (
        patch("time.clock_gettime_ns", side_effect=lambda clock: now[0]),
        patch("time.monotonic", side_effect=lambda: now[0] / 1e9),
        patch("time.sleep", side_effect=sleep),
      ):
        prepared = owner.prepare(admission=lambda: True)
        card = Car(CI=CarInterface(prepared), RI=RadarInterface(prepared), startup_owner=owner)
      assert owner.ready and owner.published and owner.prepared_for(card.CP)
      ci = card.CI
      ci.update([])
      config = cp.safetyConfigs[-1]
      if scenario != "panda":
        native.set_alternative_experience(cp.alternativeExperience)
        assert native.set_safety_hooks(int(config.safetyModel.raw), int(config.safetyParam)) == 0
        native.init_tests()
        native.set_alternative_experience(cp.alternativeExperience)
      for tick in range(1, 121):
        now[0] += 10_000_000
        frames = fd_frames(ci, tick) if cp.flags & HyundaiFlags.CANFD else classic_frames(ci, tick)
        state = ci.update([(now[0], frames)])
        native.set_timer(tick * 10_000)
        for address, data, bus in frames:
          assert native.safety_rx_hook(libsafety_py.make_CANPacket(address, bus - offset, data))
        if tick % 100 == 0:
          native.safety_tick()
      assert state.canValid and not state.canTimeout
      if scenario != "panda":
        assert native.safety_config_valid()
      else:
        assert native.get_current_safety_mode() == int(structs.CarParams.SafetyModel.elm327)
        assert native.get_current_safety_param() == 1
      if scenario == "sources":
        if cp.flags & HyundaiFlags.CANFD:
          parser = next(parser for parser in ci.can_parsers.values() if parser.bus == getattr(owner, "camera_bus", 2))
          name = "CAM_0x2a4"
        else:
          parser = next(parser for parser in ci.can_parsers.values() if parser.bus == 0)
          name = "EMS16"
        parser.message_states[parser.dbc.name_to_msg[name].address].timestamps.clear()
      command = messaging.new_message("carControl", valid=True, logMonoTime=now[0] - (151_000_000 if scenario == "command" else 0))
      pandas = messaging.new_message("pandaStates", pandas_count, valid=True, logMonoTime=now[0])
      if pandas_count == 2:
        auxiliary = pandas.pandaStates[0]
        auxiliary.safetyModel = structs.CarParams.SafetyModel.noOutput
        auxiliary.safetyParam, auxiliary.alternativeExperience, auxiliary.controlsAllowed = 0, 0, False
      panda = pandas.pandaStates[-1]
      panda.safetyModel, panda.safetyParam = native.get_current_safety_mode(), native.get_current_safety_param()
      panda.alternativeExperience, panda.controlsAllowed = cp.alternativeExperience, native.get_controls_allowed()
      panda.safetyRxChecksInvalid = native.safety_test_toyota_rx_summary_invalid()
      panda.ignitionLine = True
      card.sm.update_msgs(now[0] / 1e9, [command.as_reader(), pandas.as_reader()])
      envelopes, applications = [], []
      original_apply, original_send = ci.apply, card.pm.send

      def apply(*args, **kwargs):
        applications.append(True)
        return original_apply(*args, **kwargs)

      def publish(service, encoded):
        event = messaging.log_from_bytes(encoded)
        envelopes.append((event.valid, len(event.sendcan)))
        if scenario == "healthy" and event.valid:
          for frame in event.sendcan:
            assert native.safety_tx_hook(libsafety_py.make_CANPacket(frame.address, frame.src - offset, bytes(frame.dat)))
        original_send(service, encoded)

      with (
        patch.object(ci, "apply", side_effect=apply),
        patch.object(card.pm, "send", side_effect=publish),
        patch("time.clock_gettime_ns", side_effect=lambda clock: now[0]),
        patch("time.monotonic_ns", side_effect=lambda: now[0]),
        patch("time.monotonic", side_effect=lambda: now[0] / 1e9),
      ):
        card.controls_update(state, command.carControl.as_reader(), initialize_only=initialize_only)
      if scenario == "healthy":
        assert applications == [True] and len(envelopes) == 1 and envelopes[0][0]
        assert owner.handed_off
      else:
        assert applications == [], f"PREPARED_OWNER_BYPASSED_{scenario.upper()}_ADMISSION"
        assert all(envelope == (False, 0) for envelope in envelopes) and not owner.handed_off
      assert not native.get_controls_allowed() and native.aol_get_permission_mask() == 0
    finally:
      if card is not None:
        card.vehicle_startup.close()
