"""Measured Card publication lapse through current factory/parser/SD/Controls.

Native RX earns controls; the extracted current pandad C++ writer requests the
actual host decision. No controls_allowed or native permission flag is forced.
Board heartbeat telemetry is an explicit transport fixture, not an execution of
the firmware heartbeat timer; its reported-loss case must veto retention.
This measures a current caller contract, not an unlogged original blocked call.
"""
from collections import deque
from contextlib import ExitStack
import os
import json
from pathlib import Path
from unittest import mock

import pytest
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.gm import gmcan
from opendbc.car.gm.tests.test_volt_sdgm_control import sdgm_params
from opendbc.car.gm.values import CruiseButtons, DBC
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car import card as card_module
from openpilot.selfdrive.selfdrived.events import EventName
from openpilot.starpilot.aol.tests import test_gm as gm_fixture
from openpilot.starpilot.aol.tests import test_ioniq_availability as caller_fixture
from openpilot.starpilot.aol.tests.test_gm_startup_main import parsed_packets
from openpilot.starpilot.aol.wire import PandaSlot, SafetyState, decode_intent, encode_safety
from openpilot.starpilot.car.gm.steering_companion import SteeringCompanion
from openpilot.starpilot.aol.tests.gm_gap_cpp_bridge import gm_host_heartbeat_bridge as gm_host_heartbeat_bridge

BASE = 10_000_000_000
LAST_SOURCE = BASE + 250_000_000
GAP_AGE = 36_415_981
SOURCE_ADVANCE = 73_592_025
NATIVE_BEFORE_SOURCE = 12_576_709


class Capture:
  def __init__(self):
    self.messages = {}

  def send(self, name, event):
    raw = event if isinstance(event, bytes) else event.to_bytes()
    self.messages[name] = messaging.log_from_bytes(raw)


def joined_case(cleanup, bridge, *, c9=False, pedal='none', engage=False, cp=None):
  saved = Params()
  cleanup.callback(saved._finalizer)
  for key in ('OpenpilotEnabledToggle', 'AlwaysOnLateral', 'AlphaLongitudinalEnabled'):
    saved.put_bool(key, True, block=True)
  default_sdgm = cp is None
  cp = sdgm_params(alpha=True, brake_c9=c9) if cp is None else cp
  card = gm_fixture.TestGmAol.card(cp, saved)
  cleanup.callback(card.vehicle_startup.close)
  assert card.CP.flags == cp.flags and card.CP.alternativeExperience == 32
  assert card.CP.safetyConfigs[0].safetyParam == cp.safetyConfigs[0].safetyParam
  assert card.CP.pcmCruise == cp.pcmCruise and card.CP.openpilotLongitudinalControl == cp.openpilotLongitudinalControl
  if default_sdgm and not c9:
    assert card.CP.flags == 16 and card.CP.safetyConfigs[0].safetyParam == 0x5007
  card.ci_initialized = True  # No ECU/device I/O; native permission is earned below.
  card.pm = Capture()
  case = caller_fixture.IoniqAvailabilityTests(methodName='runTest')
  cleanup.callback(case.doCleanups)
  producer_message = messaging.new_message  # Preserve Card's own publication clock before the SD fixture patches it.
  sd, controls, step = case._authority_callers(False, cp=card.CP.as_reader().as_builder(), genuine_events=True)
  # Restore the real constructor's settings snapshot removed by this generic fixture.
  from openpilot.starpilot.aol.intent import read_settings
  sd.aol_settings = read_settings(sd.params)
  assert sd.aol_settings.enabled and sd.aol_settings.pause_brake_mps == 0.
  cleanup.callback(sd.params._finalizer)
  cleanup.callback(controls.params._finalizer)
  native = libsafety_py.libsafety
  # GM snapshots AE during mode initialization; configure it before selecting hooks.
  native.set_alternative_experience(card.CP.alternativeExperience)
  assert native.set_safety_hooks(card.CP.safetyConfigs[0].safetyModel.raw,
                                 card.CP.safetyConfigs[0].safetyParam) == 0
  # Match the shared native harness: hook selection resets the relay transition
  # counter; initialize its established-mode fixture afterwards, then set AE.
  native.init_tests()
  native.set_alternative_experience(card.CP.alternativeExperience)
  assert not native.get_controls_allowed()
  bridge.reset_host()
  packer = CANPacker(DBC[card.CP.carFingerprint][Bus.pt])
  receipt_queue = deque()
  sd.steering_companion = SteeringCompanion(sock=object())
  original_receive = messaging.recv_one_or_none
  original_update = sd.sm.update_msgs
  original_controls_update = controls.sm.update_msgs
  card_delivered = {}
  controls_delivered = {}
  delivered = {}
  current = {'gap': False, 'raw_ahead': False, 'pedal': 'none', 'warmup_trace': [], 'last_cs': None, 'wire': None, 'pandas': None, 'last_command': None}

  def receive(sock):
    if sock is sd.steering_companion.sock:
      return receipt_queue.popleft() if receipt_queue else None
    return original_receive(sock)

  def update(now, rows):
    if current['gap'] and not current['raw_ahead']:
      rows = [row for row in rows if row.which() not in ('aolIntentWire', 'carOutput', 'starpilotCarState')]
    actual = []
    for row in rows:
      if row.which() in ('aolSafetyWire', 'pandaStates'):
        signature = row.as_builder().to_bytes()
        if delivered.get(row.which()) == signature:
          continue
        delivered[row.which()] = signature
      actual.append(row)
    return original_update(now, actual)

  def controls_update(now, rows):
    actual = []
    for row in rows:
      if current['gap'] and row.which() in ('carState', 'carOutput', 'aolIntentWire', 'starpilotCarState'):
        continue
      if row.which() in ('aolSafetyWire', 'pandaStates'):
        signature = row.as_builder().to_bytes()
        if controls_delivered.get(row.which()) == signature:
          continue
        controls_delivered[row.which()] = signature
      actual.append(row)
    return original_controls_update(now, actual)

  cleanup.enter_context(mock.patch.object(messaging, 'recv_one_or_none', side_effect=receive))
  cleanup.enter_context(mock.patch.object(controls.sm, 'update_msgs', side_effect=controls_update))
  cleanup.enter_context(mock.patch.object(sd.sm, 'update_msgs', side_effect=update))

  def physical(now, tick, *, main=True, button=CruiseButtons.UNPRESS, eps=1):
    rows = parsed_packets(card.CP, packer, now, tick, 20., main)
    if current['pedal'] != 'none':
      from opendbc.car.gm.tests.test_volt_camera_control import feed_camera
      from types import SimpleNamespace
      _, extra = feed_camera(SimpleNamespace(update=lambda _: None), packer, now,
        counter=tick % 4, speed=20., brake=current['pedal'] == 'brake',
        regen=current['pedal'] == 'regen', active=False, eps_status=eps)
      addresses = {0xBE, 0xC9, 0xF1, 0xBD}
      rows = [row for row in rows if row[0] not in addresses]
      rows += [row for row in extra if row[0] in addresses and row[0] != 0xBE]
      rows.append(packer.make_can_msg('ECMAcceleratorPos', 0,
        {'BrakePedalPos': 10 if current['pedal'] == 'brake' else 0}))
    rows = [row for row in rows if row[0] not in (0x1E1, 0x184)]
    rows += [gmcan.create_buttons(packer, 0, tick % 4, button),
             packer.make_can_msg('PSCMStatus', 0, {'LKATorqueDeliveredStatus': eps})]
    native.set_timer(now // 1000)
    for address, data, bus in rows:
      assert native.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data)), (hex(address), bus)
    native.safety_tick()
    return rows

  def sample_native(now, *, heartbeat_lost=False):
    axis = sd.pm.messages.get('aolAxisState')
    intent = card.pm.messages.get('aolIntentWire')
    axis_bytes = axis.as_builder().to_bytes() if axis is not None else b''
    intent_bytes = intent.as_builder().to_bytes() if intent is not None else b''
    native.set_timer(now // 1000)
    plan = bridge.plan_host(4, card.CP.safetyConfigs[0].safetyParam,
      native.aol_get_request_mask(), native.aol_get_permission_mask(), now,
      axis_bytes, len(axis_bytes), intent_bytes, len(intent_bytes), sd.enabled)
    native.aol_set_host_request(plan & 0xFF)
    native.set_aol_test_heartbeat(bool(plan & 0x100))
    bridge.complete_host(native.aol_get_request_mask(), native.aol_get_permission_mask())
    mask, request = native.aol_get_permission_mask(), native.aol_get_request_mask()
    bad_rx = not native.safety_config_valid()
    relay = bool(native.get_relay_malfunction())
    panda = messaging.new_message('pandaStates', 1, valid=True, logMonoTime=now)
    row = panda.pandaStates[0]
    row.safetyModel, row.safetyParam = card.CP.safetyConfigs[0].safetyModel, card.CP.safetyConfigs[0].safetyParam
    row.alternativeExperience, row.controlsAllowed = 32, native.get_controls_allowed()
    row.safetyRxChecksInvalid = bad_rx
    row.heartbeatLost = heartbeat_lost  # Received board-health fixture; safety DLL has no firmware timer.
    if relay:
      row.faults = ['relayMalfunction']
    slot = PandaSlot(0, 'gm-gap-native', 4, row.safetyParam, 32, row.controlsAllowed, bad_rx, heartbeat_lost, int(relay))
    wire = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=now)
    wire.aolSafetyWire = encode_safety(SafetyState(1, bool(plan & 0x400), now, now + 200_000_000,
      4, row.safetyParam, bool(mask & 1), bool(mask & 2), bool(request & 1), bool(request & 2),
      'gm-gap-native', sd.aol_session_id, now, (slot,)))
    current['wire'], current['pandas'] = wire, panda
    return mask

  def card_receive(now, rows, *, publish=True):
    peers = [current['wire'].as_reader(), current['pandas'].as_reader()]
    peers += [sd.pm.messages[name] for name in ('aolAxisState', 'onroadEvents', 'selfdriveState') if name in sd.pm.messages]
    if current['last_command'] is not None:
      peers.append(current['last_command'])
    device = messaging.new_message('deviceState', valid=sd.sm.valid['deviceState'],
                                   logMonoTime=sd.sm.logMonoTime['deviceState'])
    device.deviceState = sd.sm['deviceState']
    peers.append(device.as_reader())
    fresh = []
    for row in peers:
      signature = row.as_builder().to_bytes()
      if card_delivered.get(row.which()) != signature:
        card_delivered[row.which()] = signature
        fresh.append(row)
    card.sm.update_msgs(now / 1e9, fresh)
    def card_message(*args, **kwargs):
      kwargs.setdefault('logMonoTime', now)
      return producer_message(*args, **kwargs)

    with mock.patch.object(card_module.time, 'monotonic_ns', return_value=now), \
         mock.patch.object(card_module.time, 'monotonic', return_value=now / 1e9), \
         mock.patch.object(card_module.time, 'clock_gettime_ns', return_value=now), \
         mock.patch.object(card_module.messaging, 'new_message', side_effect=card_message), \
         mock.patch.object(card_module.messaging, 'drain_sock_raw', return_value=[b'physical-can']), \
         mock.patch.object(card_module, 'can_capnp_to_list', return_value=[(now, rows)]), \
         mock.patch.object(card_module, 'REPLAY', False), mock.patch.object(card.sm, 'update'), \
         mock.patch.object(card.RI, 'update', return_value=None):
      cs, rd = card.state_update()
      if publish:
        card.state_publish(cs, rd)
    current['last_cs'] = cs
    return cs

  def drive(now, tick, *, gap=False, missing_companion=False):
    current['gap'] = gap
    assert current['wire'] is not None and current['pandas'] is not None
    if (not gap or current['raw_ahead']) and not missing_companion:
      receipt_queue.append(card.pm.messages['starpilotCarState'])
    source = card.pm.messages['carState'].as_builder()
    intent = card.pm.messages['aolIntentWire'].as_builder()
    authority = None if (gap and not current['raw_ahead']) or missing_companion else card.pm.messages['starpilotCarState'].as_builder()
    axis, state, command = step(tick, offset_ns=now - (BASE + tick * 10_000_000), timeout=gap,
      car_state_message=None if gap else source, intent_message=intent, companion=False,
      authority_message=authority, car_output_message=card.pm.messages['carOutput'].as_builder(),
      safety_message=current['wire'], panda_state_message=current['pandas'])
    current['last_command'] = controls.pm.messages['carControl']
    return axis, state, command

  def apply_command(now, command):
    native.set_timer(now // 1000)
    card.pm.messages.pop('sendcan', None)
    # Card.step passes the decoded SubMaster reader, never Controls' builder.
    # Match the same explicit BOOTTIME/monotonic clock used in card_receive.
    received = current['last_command'].carControl
    assert received.to_dict() == command.to_dict()
    with mock.patch.object(card_module.time, 'monotonic_ns', return_value=now), \
         mock.patch.object(card_module.time, 'monotonic', return_value=now / 1e9), \
         mock.patch.object(card_module.time, 'clock_gettime_ns', return_value=now):
      card.controls_update(current['last_cs'], received)
    if 'sendcan' in card.pm.messages:
      for frame in card.pm.messages['sendcan'].sendcan:
        assert native.safety_tx_hook(libsafety_py.make_CANPacket(frame.address, frame.src, bytes(frame.dat)))

  native_due = BASE
  for tick in range(26):
    now = BASE + tick * 10_000_000
    if tick == 24:
      sample_native(LAST_SOURCE - NATIVE_BEFORE_SOURCE)
      native_due = LAST_SOURCE - NATIVE_BEFORE_SOURCE + 100_000_000
    rows = physical(now, tick, main=not engage or tick >= 5,
                    button=CruiseButtons.DECEL_SET if engage and tick == 12 else CruiseButtons.UNPRESS)
    if current['wire'] is None or now >= native_due:
      sample_native(now)
      native_due = now + 100_000_000
    card_receive(now, rows)
    axis, state, command = drive(now + 5_467_396, tick)
    apply_command(now + 5_467_397, command)
    current['warmup_trace'].append((tick, current['last_cs'].cruiseState.available,
      card.aol_card_intent.allowed_latch, card.aol_card_intent._main_cycle_required,
      card.aol_transport_feedback.failed, str(axis.faultReason), axis.lateralActive,
      axis.longitudinalActive, native.aol_get_request_mask(), native.aol_get_permission_mask()))
  if os.environ.get('GM_SDGM_TRACE'):
    with Path(os.environ['GM_SDGM_TRACE']).open('a') as out:
      out.write(json.dumps({'c9': c9, 'engage': engage, 'warmup': current['warmup_trace']}) + '\n')
  assert native.get_controls_allowed() == engage and native.aol_get_permission_mask() == (3 if engage else 1)
  assert axis.lateralActive and command.latActive
  assert axis.longitudinalActive == engage and command.longActive == engage
  assert state.enabled == engage and card.aol_card_intent.allowed_latch, current['warmup_trace']
  assert not card.CI.CS.steering_authority.latched
  assert decode_intent(card.pm.messages['aolIntentWire'].aolIntentWire).carStateLogMonoTime == LAST_SOURCE
  return card, sd, controls, native, current, physical, sample_native, card_receive, drive, apply_command




@pytest.mark.parametrize('c9', (False, True))
@pytest.mark.parametrize('pedal', ('brake', 'regen'))
@pytest.mark.parametrize('fault', ('none', 'eps_loss', 'heartbeat', 'main_off'))
def test_sdgm_main_and_held_pedal_receive_order(gm_host_heartbeat_bridge, c9, pedal, fault):
  with OpenpilotPrefix(), ExitStack() as cleanup, mock.patch.dict(os.environ, {'SIMULATION': '0', 'AOL_REPLAY_RUNTIME': '1'}):
    joined = joined_case(cleanup, gm_host_heartbeat_bridge, c9=c9, pedal=pedal, engage=True)
    card, sd, controls, native, current, physical, sample_native, card_receive, drive, apply_command = joined
    # Isolate the later brake gap from the independently reproduced cold MAIN
    # bug: a physical MAIN cycle after host readiness and ordinary SET earn
    # real native LONG first, then the pedal withdraws it.
    # The separate cold-start/Long test below uses MAIN already ON without SET.
    assert card.aol_card_intent.allowed_latch and sd.enabled and sd.active
    current['pedal'] = pedal
    for tick in range(26, 35):
      now = BASE + tick * 10_000_000
      rows = physical(now, tick)
      if tick in (26, 30):
        sample_native(now)
      card_receive(now, rows)
      axis, state, command = drive(now + 1, tick)
      assert axis.lateralActive and command.latActive and not command.longActive
      assert not state.enabled and card.aol_card_intent.allowed_latch
      assert current['last_cs'].brakePressed == (pedal == 'brake')
      assert card.pm.messages['carState'].carState.brakePressed == current['last_cs'].brakePressed
      assert current['last_cs'].regenBraking == (pedal == 'regen')
      assert card.pm.messages['carState'].carState.regenBraking == current['last_cs'].regenBraking
      apply_command(now + 2, command)
    source = BASE + 340_000_000
    gap_time = source + 35_320_469
    # Measured relative ages: original selected source35.320469ms;
    # new Card source/raw5.224167ms ahead of the selected receiver sample.
    newer = gap_time - 5_224_167
    # Earn a real EPS fault through confirmed active feedback before returning
    # to the held pedal. A neutral EPS response during a physical pedal pause
    # is deliberately accepted by the unchanged vehicle monitor.
    if fault == 'eps_loss':
      current['pedal'] = 'none'
    for tick in (35, 36):
      stamp = BASE + tick * 10_000_000
      rows = physical(stamp, tick, eps=0 if fault == 'eps_loss' and tick == 36 else 1)
      if fault == 'eps_loss':
        card_receive(stamp, rows, publish=False)
        if tick == 35:
          assert card.CI.CS.steering_authority.seen_active
        else:
          assert card.CI.CS.steering_authority.latched
    if fault == 'eps_loss':
      current['pedal'] = pedal
    if fault == 'heartbeat':
      sample_native(gap_time - 8_811_250, heartbeat_lost=True)
    else:
      sample_native(gap_time - 8_811_250)
    rows = physical(newer, 37, main=fault != 'main_off', eps=0 if fault == 'eps_loss' else 1)
    card_receive(newer, rows)
    if fault == 'eps_loss':
      assert card.CI.CS.steering_authority.latched
      assert card.pm.messages['starpilotCarState'].starpilotCarState.lateralAuthorityUnavailable
    current['raw_ahead'] = True
    axis, state, command = drive(gap_time, 38, gap=True)
    assert not command.latActive and not command.longActive
    assert not axis.lateralActive and not axis.longitudinalActive
    if fault != 'none':
      if fault != 'main_off':
        assert EventName.controlsMismatch in sd.events.names
      return
    assert not sd.steering_authority_unavailable and not sd.aol_authority_lost
    assert EventName.controlsMismatch not in sd.events.names
    assert str(axis.faultReason) == 'availabilityPause' and axis.sourceCarStateMonoTime == 0
    # Fresh matching input must recover independently, with LONG still OFF.
    fresh = gap_time + 10_689_334
    rows = physical(fresh, 39)
    card_receive(fresh, rows)
    assert card.aol_card_intent.allowed_latch
    recovered, state, command = drive(fresh + 1, 39)
    assert recovered.lateralActive and command.latActive and not command.longActive
    assert not state.enabled and not sd.aol_authority_lost
    apply_command(fresh + 2, command)
    assert sample_native(fresh + 10_000_000) == 1


@pytest.mark.parametrize('c9', (False, True))
@pytest.mark.parametrize('pedal', ('brake', 'regen'))
def test_sdgm_long_brake_withdrawal_keeps_main_lateral(gm_host_heartbeat_bridge, c9, pedal):
  with OpenpilotPrefix(), ExitStack() as cleanup, mock.patch.dict(os.environ, {'SIMULATION': '0', 'AOL_REPLAY_RUNTIME': '1'}):
    joined = joined_case(cleanup, gm_host_heartbeat_bridge, c9=c9)
    card, sd, controls, native, current, physical, sample_native, card_receive, drive, apply_command = joined
    engaged = False
    for tick in range(26, 47):
      now = BASE + tick * 10_000_000
      if tick == 38:
        current['pedal'] = pedal
      rows = physical(now, tick, button=CruiseButtons.DECEL_SET if tick == 27 else CruiseButtons.UNPRESS)
      if tick in (26, 30, 37, 39, 44):
        sample_native(now)
      card_receive(now, rows)
      axis, state, command = drive(now + 1, tick)
      if 32 <= tick < 38:
        assert state.enabled and axis.longitudinalActive and command.longActive
        engaged = True
      if tick >= 38:
        assert not state.enabled and not command.longActive and not axis.longitudinalActive
        assert axis.lateralActive and command.latActive and card.aol_card_intent.allowed_latch
        assert not sd.aol_authority_lost and EventName.controlsMismatch not in sd.events.names
      apply_command(now + 2, command)
    assert engaged


@pytest.mark.parametrize('c9', (False, True))
@pytest.mark.parametrize('pedal', ('brake', 'regen'))
@pytest.mark.parametrize('status', (2, 3))
def test_sdgm_factory_eps_withdrawal_and_recovery(gm_host_heartbeat_bridge, c9, pedal, status):
  """Factory EPS faults withdraw output; healthy MAIN recovers independent LAT."""
  from openpilot.starpilot.aol.wire import decode_safety
  with OpenpilotPrefix(), ExitStack() as cleanup, mock.patch.dict(os.environ, {'SIMULATION': '0', 'AOL_REPLAY_RUNTIME': '1'}):
    joined = joined_case(cleanup, gm_host_heartbeat_bridge, c9=c9, engage=True)
    card, sd, controls, native, current, physical, sample_native, card_receive, drive, apply_command = joined
    current['pedal'] = pedal

    def cycle(index, *, eps=1, main=True):
      now = BASE + index * 10_000_000
      rows = physical(now, index, main=main, eps=eps)
      card_receive(now, rows)
      axis, state, command = drive(now + 1, index)
      apply_command(now + 2, command)
      mask = sample_native(now + 3)
      snapshot = decode_safety(bytes(current['wire'].aolSafetyWire))
      assert snapshot is not None and snapshot.compatible
      assert snapshot.observedMonoTime == snapshot.sourcePandaStatesMonoTime == now + 3
      assert snapshot.safetyParam == card.CP.safetyConfigs[0].safetyParam
      assert not command.longActive and not axis.longitudinalActive and not state.enabled
      return axis, command, mask, snapshot

    # Withdraw an actually engaged LONG with the physical pedal, retaining LAT.
    for index in range(26, 31):
      axis, command, mask, _ = cycle(index)
      assert axis.lateralActive and command.latActive and mask == 1
      assert card.aol_card_intent.allowed_latch
    assert not native.get_controls_allowed()
    packer = CANPacker(DBC[card.CP.carFingerprint][Bus.pt])
    address, data, bus = gmcan.create_steering_control(packer, 0, 1, 0, True)

    def nonzero_native_request():
      return bool(native.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data)))

    assert nonzero_native_request()  # Establish this exact valid packet's granted baseline.
    for index in range(31, 35):
      axis, command, mask, snapshot = cycle(index, eps=status)
      parsed = current['last_cs']
      assert parsed.steerFaultTemporary == (status == 2)
      assert parsed.steerFaultPermanent == (status == 3)
      assert not axis.lateralActive and not command.latActive
      assert not card.last_actuators_output.torqueOutputCan
      # Actual host OFF reaches the current C++ owner after this physical RX;
      # GM firmware reads driver torque from PSCM, not its factory fault enum.
      assert mask == native.aol_get_request_mask() == native.aol_get_permission_mask() == 0
      assert not snapshot.requestedLateral and not snapshot.lateralAllowed
      assert not snapshot.requestedLongitudinal and not snapshot.longitudinalAllowed
      assert not nonzero_native_request()
      if status == 2:
        assert card.aol_card_intent.allowed_latch
        assert EventName.controlsMismatch not in sd.events.names
      else:
        assert not card.aol_card_intent.allowed_latch
    # Release the pedal and clear the fault. Neither is a new LONG engagement.
    current['pedal'] = 'none'
    recovered = False
    for index in range(35, 40):
      axis, command, mask, _ = cycle(index)
      assert not current['last_cs'].steerFaultTemporary and not current['last_cs'].steerFaultPermanent
      if not card.aol_card_intent.allowed_latch:
        assert not axis.lateralActive and not command.latActive and mask == 0
      recovered |= bool(card.aol_card_intent.allowed_latch and axis.lateralActive and command.latActive and mask == 1)
    # Fresh healthy EPS and native acknowledgment recover persistent MAIN intent.
    # Neither an active fault nor a missing native grant was accepted above.
    assert recovered and nonzero_native_request()
    for index in range(40, 44):
      axis, command, mask, _ = cycle(index, main=False)
      assert not card.aol_card_intent.allowed_latch and mask == 0
      assert not axis.lateralActive and not command.latActive
    recovered = False
    for index in range(44, 49):
      axis, command, mask, _ = cycle(index)
      recovered |= bool(axis.lateralActive and command.latActive and mask == 1)
    assert recovered and card.aol_card_intent.allowed_latch
    assert nonzero_native_request()
