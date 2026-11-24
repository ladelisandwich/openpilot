"""Keep factory SCC until an inactive producer is current; never bypass native RX."""
import os
from unittest.mock import patch

import pytest

from opendbc.car import Bus, CanData, structs
from opendbc.car.hyundai.ecu_startup import Outcome
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.radar_interface import RadarInterface
from opendbc.car.hyundai.tests.test_torque_ev_startup import Keeper, physical_frames
from opendbc.car.hyundai.hyundaicanfd import hkg_can_fd_checksum
from opendbc.car.hyundai.ioniq6_handoff import TimestampedCanPacket
from openpilot.starpilot.tests.test_ev6_startup import params
from opendbc.can.packer import CANPacker
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car.card import Car
from openpilot.starpilot.vehicle_startup import VehicleStartupOwner


@pytest.mark.parametrize("scenario", (
  "healthy_suppressed", "healthy_positive", "stale_control", "active_control", "missing_source",
  "native_mode", "capture_lost", "cp_changed", "cruise_fault", "stock_engaged",
  "disable_negative", "disable_malformed", "restore_failure", "publisher_failure"))
def test_actual_card_defers_ev6_until_inactive_producer_and_then_native_rx(scenario):
  native = libsafety_py.libsafety
  assert native.set_safety_hooks(int(structs.CarParams.SafetyModel.elm327), 1) == 0
  native.init_tests()
  now = [100_000_000_000]
  requests, emitted = [], []
  muted = [False]

  def send(frames):
    for address, data, bus in frames:
      assert native.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data))
    emitted.extend(frames)
    if scenario == "publisher_failure" and any(data[:4] == b"\x03\x28\x83\x01" for _, data, _ in frames):
      muted[0] = True
      raise OSError("diagnostic publisher failed after possible ECU delivery")

  def query(sender, receiver, bus, addresses, commands, responses):
    command = commands[0]
    requests.append((now[0], command))

    class Query:
      def get_data(self, timeout):
        sender([(0x730, bytes([len(command)]) + command + bytes(7 - len(command)), bus)])
        if command == b"\x28\x83\x01":
          muted[0] = True
          if scenario in ("disable_negative", "restore_failure"):
            return {(0x730, None): b"\x7f\x28\x22"}
          if scenario == "disable_malformed":
            return {(0x730, None): b"\x68"}
          return {(0x730, None): b"\x68\x03"} if scenario == "healthy_positive" else {}
        if command == b"\x28\x00\x01":
          if scenario == "restore_failure":
            return {}
          muted[0] = False
        return {(0x730, None): b""}
    return Query()

  def sleep(seconds):
    now[0] += round(seconds * 1e9)

  def update_sm(host, command, stale=False):
    message = messaging.new_message("carControl", valid=True, logMonoTime=now[0] - (151_000_000 if stale else 0))
    message.carControl = command
    panda = messaging.new_message("pandaStates", 1, valid=True, logMonoTime=now[0])
    state = panda.pandaStates[0]
    state.safetyModel, state.safetyParam = native.get_current_safety_mode(), native.get_current_safety_param()
    state.alternativeExperience = host.CP.alternativeExperience
    state.ignitionLine = True
    state.safetyRxChecksInvalid = native.safety_test_toyota_rx_summary_invalid()
    host.sm.update_msgs(now[0] / 1e9, [message.as_reader(), panda.as_reader()])

  factory = CarInterface.startup_owner

  def startup_factory(*args, **kwargs):
    instance = factory(*args, **kwargs)
    instance.clock, instance.keeper_factory, instance._query = lambda: now[0] / 1e9, Keeper, query
    return instance

  with OpenpilotPrefix(), patch.object(CarInterface, "startup_owner", side_effect=startup_factory), \
       patch.dict(os.environ, {"SIMULATION": "0", "AOL_REPLAY_RUNTIME": "1"}), \
       patch("time.clock_gettime_ns", side_effect=lambda clock: now[0]), \
       patch("time.monotonic_ns", side_effect=lambda: now[0]), \
       patch("time.monotonic", side_effect=lambda: now[0] / 1e9), \
       patch("time.sleep", side_effect=sleep), \
       patch("opendbc.car.hyundai.ecu_startup.IsoTpParallelQuery", side_effect=query):
    saved = Params()
    for key in ("OpenpilotEnabledToggle", "AlwaysOnLateral", "AlphaLongitudinalEnabled"):
      saved.put_bool(key, True, block=True)
    holder = VehicleStartupOwner()
    counter = [0]

    lose_capture = [False]

    def receive():
      if lose_capture[0]:
        return []
      now[0] += 10_000_000
      counter[0] += 1
      data = bytearray(32)
      data[2] = counter[0] % 256
      data[4] = 7
      data[:2] = hkg_can_fd_checksum(0x51, None, data).to_bytes(2, "little")
      return [TimestampedCanPacket([CanData(0x51, bytes(data), 0)], now[0])]
    prepared = holder.prepare(params(), CarInterface, (receive, send), requested=True,
                              admission=lambda: True,
                              defer_until_controls=True)
    startup = holder.owner
    assert startup is not None
    startup.clock, startup.keeper_factory, startup._query = lambda: now[0] / 1e9, Keeper, query
    try:
      assert startup.outcome is Outcome.PENDING and not requests and not emitted
      assert not startup.before_control(configured=True, sources_current=True, control_current=True)
      card = Car(CI=CarInterface(prepared), RI=RadarInterface(prepared), startup_owner=startup)
      startup.admission = card.startup_diagnostic_admission
      card.params.put_bool("IsOffroad", False, block=True)
      original_cp = card.CP.to_dict()
      packer = CANPacker(card.CI.can_parsers[Bus.pt].dbc.name)
      # Explicit synthetic delayed producer: stock has not been told to stop.
      for tick in range(1, 701):
        now[0] += 10_000_000
        state = card.CI.update([(now[0], physical_frames(card.CP, packer, tick))])
      assert state.canValid and not state.canTimeout and not state.accFaulted
      assert not requests and not muted[0] and startup.keeper is None
      command = structs.CarControl(enabled=scenario == "active_control", latActive=scenario == "active_control").as_reader()
      if scenario == "missing_source":
        parser = card.CI.can_parsers[Bus.pt]
        parser.message_states[parser.dbc.name_to_msg["TCS"].address].timestamps.clear()
      if scenario == "cruise_fault":
        state.accFaulted = True
      if scenario == "stock_engaged":
        state.cruiseState.enabled = True
      if scenario == "capture_lost":
        lose_capture[0] = True
      if scenario == "cp_changed":
        card.CP.steerActuatorDelay += 0.01
      config = card.CP.safetyConfigs[-1]
      if scenario == "native_mode":
        native.set_alternative_experience(card.CP.alternativeExperience)
        assert native.set_safety_hooks(int(config.safetyModel.raw), int(config.safetyParam)) == 0
      update_sm(card, command, stale=scenario == "stale_control")
      if scenario == "capture_lost":
        with pytest.raises(RuntimeError, match="lost its current factory capture"):
          card.controls_update(state, command, initialize_only=True)
        assert not requests and not muted[0] and not card.ci_initialized
        assert startup.outcome is Outcome.PENDING
        return
      if scenario in ("disable_negative", "disable_malformed", "restore_failure", "publisher_failure"):
        with pytest.raises(RuntimeError, match="pending"):
          card.controls_update(state, command, initialize_only=True)
        assert startup.outcome is Outcome.ABORT_UNCERTAIN and not card.ci_initialized
        assert card.CP.to_dict() == original_cp and not startup.handed_off
        assert muted[0] == (scenario == "restore_failure")
        assert startup.restored == (scenario != "restore_failure")
        return
      card.controls_update(state, command, initialize_only=True)
      if not scenario.startswith("healthy"):
        assert not requests and not muted[0] and not card.ci_initialized
        assert startup.outcome is Outcome.PENDING
        if scenario != "cp_changed":
          assert card.CP.to_dict() == original_cp
        return
      assert [command for _, command in requests] == [b"\x10\x03", b"\x28\x83\x01"]
      assert muted[0] and card.ci_initialized and startup.outcome is Outcome.SENT_UNCONFIRMED
      assert startup.keeper.started and not startup.handed_off
      # No SCC while native is diagnostic, mismatched, or awaiting RX health.
      emitted.clear()
      card.controls_update(state, command, initialize_only=True)
      assert not emitted and not startup.handed_off
      native.set_alternative_experience(card.CP.alternativeExperience)
      assert native.set_safety_hooks(int(config.safetyModel.raw), int(config.safetyParam)) == 0
      native.init_tests()
      update_sm(card, command)
      card.controls_update(state, command, initialize_only=True)
      assert not emitted and not startup.handed_off
      for tick in range(701, 811):
        now[0] += 10_000_000
        frames = physical_frames(card.CP, packer, tick)
        state = card.CI.update([(now[0], frames)])
        native.set_timer(tick * 10_000)
        for address, data, bus in frames:
          native.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data))
        if tick % 100 == 0:
          native.safety_tick()
      assert native.safety_config_valid()
      update_sm(card, command)
      captured = []

      def publish(frames, valid=True):
        if valid:
          send(frames)
          card.vehicle_startup.sent(frames, valid=valid)
          captured.extend(frames)

      with patch.object(card, "publish_sendcan", side_effect=publish):
        card.controls_update(state, command, initialize_only=True)
      assert any(address == 0x1A0 and bus == 1 for address, _, bus in captured)
      assert startup.handed_off and not startup.keeper.started
      assert not native.get_controls_allowed() and native.aol_get_permission_mask() == 0
      assert card.CP.to_dict() == original_cp
    finally:
      holder.close()
