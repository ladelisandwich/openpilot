"""SDGM fresh held-brake successor across the recorded source/decision timing pair.

Model/readiness and receive transport are bench fixtures; ECU I/O is skipped.
Neither ordinary controls_allowed nor an active steering command is seeded.
"""

from contextlib import ExitStack
from unittest.mock import patch

import pytest
from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.gm.tests.test_volt_sdgm_control import sdgm_params
from opendbc.car.gm.values import DBC
from opendbc.safety.tests.libsafety import libsafety_py as native
from openpilot.cereal import messaging
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.intent import read_settings
from openpilot.starpilot.aol.tests import test_gm as gm
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture
from openpilot.starpilot.aol.tests.test_gm_startup_main import parsed_packets
from openpilot.starpilot.aol.wire import PandaSlot, SafetyState, encode_safety
from openpilot.starpilot.lateral.tests.test_lane_runtime import feed


def sdgm_packets(cp, packer, now, tick, speed, main, gesture):
  speed = (5.858360767 if tick < 390 else 5.836455345) if speed > 0 else 0.0
  frames = parsed_packets(cp, packer, now, tick, speed, main)
  engine = packer.make_can_msg(
    "ECMEngineStatus",
    0,
    {"CruiseMainOn": int(main), "BrakePressed": int(tick > 300)},
  )
  brake = packer.make_can_msg("ECMAcceleratorPos", 0, {"BrakePedalPos": 8 if tick > 300 else 0})
  frames = [f for f in frames if f[0] not in (engine[0], brake[0])]
  return frames + [engine, brake]


@pytest.mark.parametrize("alpha", [False, True])
@pytest.mark.parametrize("c9", [False, True])
def test_actual_sdgm_held_brake_fresh_successor_after_expired_output(c9, alpha):
  exercise_sdgm(c9=c9, alpha=alpha)


def exercise_sdgm(c9, alpha, condition="healthy"):
  main_action = 0
  gesture = "held_brake"
  cp = sdgm_params(alpha=alpha, brake_c9=c9)
  packet_factory = sdgm_packets
  below_engage = False
  case = fixture.IoniqAvailabilityTests(methodName="runTest")
  with OpenpilotPrefix(), ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    sd, controls, step = case._authority_callers(False, cp=cp, genuine_events=True)
    sd.params.put("MainCruiseButtonControl", main_action, block=True)
    sd.aol_settings = read_settings(sd.params)
    # The initial real Controls result is disabled because no authority wire exists.
    # This boot-neutral fixture establishes no Panda grant or active output.
    feed(controls, 10_000_000_000, 0, active=False, enabled=False)
    boot, boot_log = controls.state_control()
    assert not boot.latActive and not boot.longActive
    controls.publish(boot, boot_log)
    card = gm.TestGmAol.card(cp, sd.params)
    cleanup.callback(card.vehicle_startup.close)
    card.ci_initialized = True
    card.car_params_published = True
    card.sm.frame = 1
    assert card.CP.to_dict() == sd.CP.to_dict() == controls.CP.to_dict()
    original_pm = card.pm
    captured = {}
    intent_queue = []
    companion_queue = []
    original_optional = messaging.recv_one_or_none

    def deliver(sock):
      if sd.steering_companion is not None and sock is sd.steering_companion.sock:
        return companion_queue.pop(0) if companion_queue else None
      if sd.aol_intent_companion is not None and sock is sd.aol_intent_companion.sock:
        return intent_queue.pop(0) if intent_queue else None
      return original_optional(sock)

    cleanup.enter_context(
      patch(
        "openpilot.selfdrive.selfdrived.selfdrived.messaging.recv_one_or_none",
        side_effect=deliver,
      )
    )

    class Tee:
      def send(self, service, message):
        original_pm.send(service, message)
        event = messaging.log_from_bytes(message) if isinstance(message, bytes) else message.as_reader()
        captured[service] = event.as_builder()
        if service == "aolIntentWire":
          intent_queue.append(event)
        if service == "starpilotCarState":
          companion_queue.append(event)

    card.pm = Tee()
    sd_receive = sd.sm.update_msgs
    last_sd_control_stamp = 0

    def receive_sd(stamp, rows):
      nonlocal last_sd_control_stamp
      # Deliver the original last published Controls envelope without renewing its source.
      published = controls.pm.messages.get("carControl")
      rows = [r for r in rows if r.which() != "carControl"]
      if published is not None and published.logMonoTime != last_sd_control_stamp:
        rows.append(published)
        last_sd_control_stamp = int(published.logMonoTime)
      return sd_receive(stamp, rows)

    cleanup.enter_context(patch.object(sd.sm, "update_msgs", side_effect=receive_sd))
    safety = native.libsafety
    safety.init_tests()
    safety.set_alternative_experience(cp.alternativeExperience)
    assert safety.set_safety_hooks(4, cp.safetyConfigs[0].safetyParam) == 0
    # Establish actual neutral producer cadence before Card publishes carOutput.
    card_receive = card.sm.update_msgs
    card_receive(10.0, [controls.pm.messages["carControl"]])
    boot_rows = []
    with patch.object(
      controls.sm,
      "update_msgs",
      side_effect=lambda stamp, rows: boot_rows.extend(rows),
    ):
      feed(controls, 10_010_000_000, 1, active=False, enabled=False)
    controls.sm.update_msgs(10.01, boot_rows)
    clocked = messaging.new_message
    with patch(
      "openpilot.selfdrive.controls.controlsd.messaging.new_message",
      side_effect=lambda *args, **kwargs: clocked(*args, **dict(kwargs, logMonoTime=10_010_000_000)),
    ):
      neutral, neutral_log = controls.state_control()
      assert not neutral.latActive and not neutral.longActive
      controls.publish(neutral, neutral_log)
    card_receive(10.01, [controls.pm.messages["carControl"]])
    last_control_stamp = int(controls.pm.messages["carControl"].logMonoTime)
    delivered = {}

    if card.volt_cc_selected:
      # Establish the real clock floor before any fresh physical/control input.
      with (
        patch(
          "openpilot.selfdrive.car.card.time.monotonic_ns",
          return_value=10_010_000_000,
        ),
        patch(
          "openpilot.selfdrive.car.card.time.clock_gettime_ns",
          return_value=10_010_000_000,
        ),
      ):
        assert not card.volt_cc_control_current()

    def wire_batch(now):
      mask = safety.aol_get_permission_mask()
      request = safety.aol_get_request_mask()
      invalid = not safety.safety_config_valid()
      relay = safety.get_relay_malfunction()
      p = messaging.new_message("pandaStates", 1, valid=True, logMonoTime=now)
      slot = p.pandaStates[0]
      slot.safetyModel = 4
      slot.safetyParam = cp.safetyConfigs[0].safetyParam
      slot.alternativeExperience = cp.alternativeExperience
      slot.controlsAllowed = safety.get_controls_allowed()
      slot.safetyRxChecksInvalid = invalid
      slot.faults = ["relayMalfunction"] if relay else []
      w = messaging.new_message("aolSafetyWire", 0, valid=True, logMonoTime=now)
      w.aolSafetyWire = encode_safety(
        SafetyState(
          1,
          True,
          now,
          now + 200_000_000,
          4,
          cp.safetyConfigs[0].safetyParam,
          bool(mask & 1),
          bool(mask & 2),
          bool(request & 1),
          bool(request & 2),
          "authority-panda",
          sd.aol_session_id,
          now,
          (
            PandaSlot(
              0,
              "authority-panda",
              4,
              cp.safetyConfigs[0].safetyParam,
              cp.alternativeExperience,
              safety.get_controls_allowed(),
              invalid,
              False,
              1 if relay else 0,
            ),
          ),
        )
      )
      return p.as_reader(), w.as_reader()

    def receive_card(now, *, control=True):
      nonlocal last_control_stamp
      rows = list(wire_batch(now))
      if card.volt_cc_selected:
        device = messaging.new_message("deviceState", valid=True, logMonoTime=now)
        device.deviceState.started, device.deviceState.startedMonoTime = (
          True,
          9_000_000_000,
        )
        rows.append(device.as_reader())
      rows += [sd.pm.messages[k] for k in ("aolAxisState", "selfdriveState", "onroadEvents") if k in sd.pm.messages]
      published = controls.pm.messages["carControl"]
      if control and published.logMonoTime != last_control_stamp:
        rows.append(published)
        last_control_stamp = int(published.logMonoTime)
      unique = []
      for event in rows:
        name = event.which()
        key = (int(event.logMonoTime), event.as_builder().to_bytes())
        if delivered.get(name) != key:
          unique.append(event)
          delivered[name] = key
      card_receive(now / 1e9, unique)

    packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])

    def physical(tick, stamp, driving):
      speed = max(4.0, cp.minSteerSpeed + 2.0)
      if below_engage:
        assert speed < cp.minEnableSpeed
      else:
        speed = max(speed, cp.minEnableSpeed + 2.0)
      packets = packet_factory(cp, packer, stamp, tick - 100, speed if driving else 0.0, True, gesture)
      packets = [f for f in packets if f[0] not in (0x1F5, 0x12A, 0x184, 0x17D)]
      if cp.pcmCruise and cp.networkLocation == structs.CarParams.NetworkLocation.fwdCamera:
        address = packer.make_can_msg("ASCMActiveCruiseControlStatus", 2, {})[0]
        packets = [f for f in packets if not (f[0] == address and f[2] == 2)]
        # Factory camera reports adaptive standby, not non-adaptive cruise mode.
        packets.append(packer.make_can_msg("ASCMActiveCruiseControlStatus", 2, {"ACCCruiseState": 2}))
      packets += [
        packer.make_can_msg("ESPStatus", 0, {"TractionControlOn": 1}),
        packer.make_can_msg("ECMPRDNL2", 0, {"PRNDL2": 4 if driving else 1}),
        packer.make_can_msg("BCMDoorBeltStatus", 0, {"LeftSeatBelt": int(driving)}),
        packer.make_can_msg("PSCMStatus", 0, {"LKATorqueDeliveredStatus": int(driving)}),
      ]
      return packets

    for warm in range(100):
      stamp = 9_000_000_000 + warm * 10_000_000
      safety.set_timer(stamp // 1000)
      safety.set_aol_test_heartbeat(True)
      safety.aol_set_host_request(0)
      packets = physical(warm, stamp, False)
      for a, data, bus in packets:
        safety.safety_fwd_hook(bus, a)
        assert safety.safety_rx_hook(native.make_CANPacket(a, bus, data))
      card.CI.update([(stamp, packets)])
      safety.safety_tick()
    assert safety.safety_config_valid() and safety.aol_get_permission_mask() == 0
    tx = 0
    active = 0
    checkpoints = {}
    old_axis = None
    for tick in range(1, 391):
      now = 10_020_000_000 + (tick - 1) * 10_000_000
      if tick == 390:
        now += 20_271_875
      decision_ns = now + (6_268_334 if tick == 389 else 11_981_615 if tick == 390 else 0)
      driving = tick > 300
      safety.set_timer(now // 1000)
      safety.set_aol_test_heartbeat(True)
      packets = physical(tick + 100, now, driving)
      for a, data, bus in packets:
        safety.safety_fwd_hook(bus, a)
        assert safety.safety_rx_hook(native.make_CANPacket(a, bus, data))
      raw = messaging.new_message('can', len(packets), valid=True, logMonoTime=now)
      for dst, (address, data, bus) in zip(raw.can, packets, strict=True):
        dst.address, dst.dat, dst.src = address, data, bus
      with patch('openpilot.selfdrive.car.card.messaging.drain_sock_raw', return_value=[raw.to_bytes()]), \
           patch.object(card.sm, 'update', side_effect=lambda timeout, _now=now: receive_card(_now)), \
           patch('openpilot.selfdrive.car.card.time.monotonic_ns', return_value=now), \
           patch('openpilot.selfdrive.car.card.time.clock_gettime_ns', return_value=now):
        cs, _ = card.state_update(defer_radar=True)
      safety.safety_tick()
      assert cs.canValid and not cs.canTimeout
      assert card._aol_can_observation[0] is cs and card._aol_can_observation[1:3] == (now, now)
      allowed = card.aol_card_intent.output(cs)[0]
      request = int(allowed) | (2 if sd.aol_axis_decision.desired_longitudinal else 0)
      safety.set_timer(decision_ns // 1000)
      safety.aol_set_host_request(request)
      mask = safety.aol_get_permission_mask()
      message = messaging.new_message
      with patch(
        "openpilot.selfdrive.car.card.messaging.new_message",
        side_effect=lambda *args, _clock=message, _now=now, **kwargs: _clock(*args, **dict(kwargs, logMonoTime=_now)),
      ):
        card.state_publish(cs, None)
      negative = tick == 390 and condition != "healthy"
      if negative:
        # Typed receiver perturbations start after a complete actual CAN/native
        # baseline; they cannot manufacture native permission or renew old output.
        from dataclasses import replace
        from openpilot.starpilot.aol.wire import decode_intent, encode_intent
        current = captured["carState"].carState
        if condition == "new_brake":
          sd.CS_prev = sd.CS_prev.as_builder()
          sd.CS_prev.brakePressed = False
        elif condition == "new_paddle":
          current.regenBraking = True
        elif condition == "new_gas":
          current.gasPressed = True
        elif condition == "cancel":
          current.buttonEvents = [structs.CarState.ButtonEvent(type="cancel", pressed=True)]
        elif condition == "invalid_current":
          current.canValid, current.canTimeout = False, True
        elif condition == "permanent_eps":
          current.steerFaultPermanent = True
        elif condition == "main_off":
          current.cruiseState.available = False
        elif condition == "gear_unknown":
          current.gearShifter = "unknown"
        elif condition == "stock_disable_edge":
          sd.CS_prev = sd.CS_prev.as_builder()
          sd.CS_prev.cruiseState.enabled = True
        if condition in ("future_current", "old_current"):
          source = decision_ns + 1 if condition == "future_current" else old_axis.sourceCarStateMonoTime
          state = captured["carState"]
          assert state is not None
          state.logMonoTime = source
          wire = captured["aolIntentWire"]
          assert wire is not None
          wire.aolIntentWire = encode_intent(replace(decode_intent(wire.aolIntentWire), carStateLogMonoTime=source))
        changes = {"producer": {"producerSessionId": "other-card"}, "settings": {"settingsQualified": False},
                   "pause_lateral": {"pauseLateral": True}}
        if condition in changes:
          wire = captured["aolIntentWire"]
          assert wire is not None
          wire.aolIntentWire = encode_intent(replace(decode_intent(wire.aolIntentWire), **changes[condition]))
        if condition == "epoch":
          sd.aol_intent_companion.integrity_epoch += 1
      axis, _, cc = step(
        tick,
        state_stamp=now,
        offset_ns=decision_ns - 10_000_000_000 - tick * 10_000_000,
        car_state_message=captured["carState"],
        intent_message=captured["aolIntentWire"],
        car_output_message=captured["carOutput"],
        car_output_valid=captured["carOutput"].valid,
        allowed=bool(mask & 1) and not (negative and condition == "native_denied"),
        native_valid=not (negative and condition == "native_invalid"),
        panda_fault=negative and condition == "rx_fault",
        relay_fault=negative and condition == "relay_fault",
        long_allowed=bool(mask & 2),
        native_request_lat=bool(request & 1),
        native_request_long=bool(request & 2),
        panda_controls_allowed=safety.get_controls_allowed(),
        authority_message=captured.get("starpilotCarState"),
      )
      if negative:
        assert old_axis.validUntilMonoTime < decision_ns
        assert not axis.lateralActive and not cc.latActive, (condition, axis.to_dict(), sd.events.names)
        assert not axis.longitudinalActive and not cc.longActive
        return
      if tick == 389:
        old_axis = axis
      receive_card(decision_ns, control=False)
      if tick == 390:
        assert old_axis.validUntilMonoTime < decision_ns
        old_deadline = old_axis.validUntilMonoTime
        receive_card(decision_ns)
        assert old_axis.validUntilMonoTime == old_deadline
        assert axis.validUntilMonoTime == decision_ns + 30_000_000
      assert not cc.enabled and not cc.longActive and not safety.get_controls_allowed()
      if tick == 390:
        assert not sd.aol_authority_lost and str(axis.faultReason) == "none"
        assert axis.lateralActive and cc.latActive and not axis.longitudinalActive
        assert axis.sourceCarStateMonoTime == now and axis.observedMonoTime == decision_ns
      if tick in (360, 420, 480, 520):
        checkpoints[tick] = (
          bool(cc.enabled),
          bool(cc.longActive),
          bool(cc.latActive),
          card.v_cruise_helper.v_cruise_kph,
          bool(cs.brakePressed),
          bool(cs.regenBraking),
        )
      if not driving:
        assert str(cs.gearShifter) == "park" and cs.seatbeltUnlatched and cs.standstill
        assert not axis.lateralActive and not cc.latActive and safety.aol_get_request_mask() == 0 and mask == 0
        assert str(axis.faultReason) != "critical", (
          tick,
          axis.to_dict(),
          sd.events.names,
        )
      failed = card.aol_disarming_fault(cs, int(card.sm.logMonoTime["onroadEvents"]), decision_ns)
      if failed:
        print(
          "SDGM_FAILURE",
          tick,
          gesture,
          axis.to_dict(),
          [e.to_dict() for e in sd.events.to_msg()],
          card.aol_transport_feedback.__dict__,
          checkpoints,
        )
      assert not failed
      assert card.aol_card_intent.allowed_latch and not card.aol_card_intent._main_cycle_required
      command = card.sm["carControl"]
      captured.pop("sendcan", None)
      with (
        patch(
          "openpilot.selfdrive.car.card.time.monotonic_ns",
          return_value=decision_ns + 2,
        ),
        patch(
          "openpilot.selfdrive.car.card.time.monotonic",
          return_value=(decision_ns + 2) / 1e9,
        ),
        patch(
          "openpilot.selfdrive.car.card.time.clock_gettime_ns",
          return_value=decision_ns + 2,
        ),
      ):
        card.controls_update(cs, command)
      emitted = captured["sendcan"]
      if not emitted.valid:
        # The first command is exactly at the real source floor; it cannot emit.
        assert tick == 1 and not emitted.sendcan and not cc.enabled and not cc.latActive and not cc.longActive
        card.CS_prev = cs
        continue
      assert emitted.valid
      for frame in emitted.sendcan:
        assert safety.safety_tx_hook(native.make_CANPacket(frame.address, frame.src, frame.dat))
        tx += 1
        if frame.address == 0x180:
          if not driving:
            assert not frame.dat[0] & 8
          else:
            active += bool(frame.dat[0] & 8)
      card.CS_prev = cs
    assert active > 0 and tx > 0 and card.aol_card_intent.allowed_latch
    assert not sd.aol_authority_lost


@pytest.mark.parametrize("c9", [False, True])
@pytest.mark.parametrize("condition", ["new_brake", "new_paddle", "cancel", "invalid_current", "future_current",
  "old_current", "new_gas", "producer", "epoch", "settings", "pause_lateral", "native_denied", "native_invalid", "rx_fault",
  "relay_fault", "permanent_eps", "main_off", "gear_unknown", "stock_disable_edge"])
def test_fresh_held_successor_rejects_changed_identity_health_or_physical_input(c9, condition):
  exercise_sdgm(c9=c9, alpha=condition != "stock_disable_edge", condition=condition)
