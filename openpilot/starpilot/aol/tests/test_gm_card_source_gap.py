"""Measured Card publication lapse through current factory/parser/SD/Controls.

Native RX earns controls; the extracted current pandad C++ writer requests the
actual host decision. No controls_allowed or native permission flag is forced.
Board heartbeat telemetry is an explicit transport fixture, not an execution of
the firmware heartbeat timer; its reported-loss case must veto retention.
This measures a current caller contract, not an unlogged original blocked call.
"""
from collections import deque
from contextlib import ExitStack
import json
import os
from pathlib import Path
from unittest import mock

import pytest
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.gm import gmcan
from opendbc.car.gm.tests.test_ascm_intercept import params as ascm_params
from opendbc.car.gm.values import CAR, CruiseButtons, DBC
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car import card as card_module
from openpilot.selfdrive.selfdrived.events import EventName
from openpilot.starpilot.aol.runtime import current_native
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


def joined_case(cleanup, bridge, *, c9=False):
  saved = Params()
  cleanup.callback(saved._finalizer)
  for key in ('OpenpilotEnabledToggle', 'AlwaysOnLateral', 'AlphaLongitudinalEnabled',
              'VoltOnePedalMode', 'VoltSNG', 'GMAutoHold'):
    saved.put_bool(key, True, block=True)
  cp = ascm_params(CAR.CHEVROLET_VOLT_ASCM, sascm=True, alpha=True, accelerator=not c9, radar=True)
  card = gm_fixture.TestGmAol.card(cp, saved)
  cleanup.callback(card.vehicle_startup.close)
  assert card.CP.flags == 16 and card.CP.alternativeExperience == 32
  if not c9:
    assert card.CP.safetyConfigs[0].safetyParam == 0xD114
  card.ci_initialized = True  # No ECU/device I/O; native permission is earned below.
  card.pm = Capture()
  case = caller_fixture.IoniqAvailabilityTests(methodName='runTest')
  cleanup.callback(case.doCleanups)
  producer_message = messaging.new_message  # Preserve Card's own publication clock before the SD fixture patches it.
  sd, controls, step = case._authority_callers(False, cp=card.CP.as_reader().as_builder(), genuine_events=True)
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
  current = {'gap': False, 'last_cs': None, 'wire': None, 'pandas': None, 'last_command': None}

  def receive(sock):
    if sock is sd.steering_companion.sock:
      return receipt_queue.popleft() if receipt_queue else None
    return original_receive(sock)

  def update(now, rows):
    if current['gap']:
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
    if not gap and not missing_companion:
      receipt_queue.append(card.pm.messages['starpilotCarState'])
    source = card.pm.messages['carState'].as_builder()
    intent = card.pm.messages['aolIntentWire'].as_builder()
    authority = None if gap or missing_companion else card.pm.messages['starpilotCarState'].as_builder()
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
    rows = physical(now, tick, main=tick >= 5,
                    button=CruiseButtons.DECEL_SET if tick == 12 else CruiseButtons.UNPRESS)
    if current['wire'] is None or now >= native_due:
      sample_native(now)
      native_due = now + 100_000_000
    card_receive(now, rows)
    axis, state, command = drive(now + 5_467_396, tick)
    apply_command(now + 5_467_397, command)
  assert native.get_controls_allowed() and native.aol_get_permission_mask() == 3
  assert axis.lateralActive and axis.longitudinalActive and command.latActive and command.longActive
  assert state.enabled and card.aol_card_intent.allowed_latch
  assert not card.CI.CS.steering_authority.latched
  assert decode_intent(card.pm.messages['aolIntentWire'].aolIntentWire).carStateLogMonoTime == LAST_SOURCE
  return card, sd, controls, native, current, physical, sample_native, card_receive, drive, apply_command


@pytest.mark.parametrize('c9', [False, True])
@pytest.mark.parametrize('fault', ['none', 'missing_companion', 'eps_loss', 'relay', 'heartbeat', 'main_off'])
def test_measured_whole_card_gap_keeps_safe_state_and_current_native_guards(gm_host_heartbeat_bridge, c9, fault):
  with OpenpilotPrefix(), ExitStack() as cleanup, mock.patch.dict(os.environ, {'SIMULATION': '0', 'AOL_REPLAY_RUNTIME': '1'}):
    joined = joined_case(cleanup, gm_host_heartbeat_bridge, c9=c9)
    card, sd, controls, native, current, physical, sample_native, card_receive, drive, apply_command = joined
    source = LAST_SOURCE
    gap_time = source + GAP_AGE
    old = sd.pm.messages['aolAxisState']
    old_native = current['wire'].to_bytes()
    # CAN continues on Panda while Card's COMPLETE source publication stops.
    for tick, delta in enumerate((10_000_000, 20_000_000, 30_000_000), 26):
      physical(source + delta, tick)
    if fault in ('missing_companion', 'eps_loss', 'main_off'):
      rows = physical(gap_time, 30, main=fault != 'main_off', eps=0 if fault == 'eps_loss' else 1)
      card_receive(gap_time, rows)
      if fault == 'eps_loss':
        assert card.CI.CS.steering_authority.latched
      axis, state, command = drive(gap_time + 1, 30, missing_companion=fault == 'missing_companion')
    else:
      if fault == 'relay':
        native.set_timer(gap_time // 1000)
        native.safety_rx_hook(libsafety_py.make_CANPacket(0x315, 0, bytes(5)))
        assert native.get_relay_malfunction()
        sample_native(gap_time)
      if fault == 'heartbeat':
        sample_native(gap_time, heartbeat_lost=True)
      axis, state, command = drive(gap_time, 30, gap=True)
    assert not command.latActive and not command.longActive
    assert not axis.lateralActive and not axis.longitudinalActive
    trace = {'c9': c9, 'fault': fault, 'axis': axis.to_dict(), 'state': state.to_dict(),
             'command': {'latActive': command.latActive, 'longActive': command.longActive},
             'steeringUnavailable': sd.steering_authority_unavailable,
             'authorityLost': sd.aol_authority_lost, 'cardLatch': card.aol_card_intent.allowed_latch}
    if os.environ.get('GM_GAP_TRACE'):
      with Path(os.environ['GM_GAP_TRACE']).open('a') as out:
        out.write(json.dumps(trace) + '\n')
    if fault == 'heartbeat':
      assert current_native(sd.sm, sd.CP, now_ns=gap_time, axis_session_id=sd.aol_session_id) is None
    if fault != 'none':
      if fault != 'main_off':
        assert EventName.controlsMismatch in sd.events.names
      return
    assert current['wire'].to_bytes() == old_native
    admitted = current_native(sd.sm, sd.CP, now_ns=gap_time, axis_session_id=sd.aol_session_id)
    assert admitted is not None and admitted.lateralAllowed and admitted.longitudinalAllowed
    assert not sd.steering_authority_unavailable and not sd.aol_authority_lost
    assert EventName.controlsMismatch not in sd.events.names
    assert axis.sourceCarStateMonoTime == 0 and str(axis.faultReason) == 'availabilityPause'
    assert state.enabled and not sd.events.contains('enable')
    # A second normal SD iteration occurs before Card resumes. Retention must
    # keep the ORIGINAL source/native identities rather than invent a new lease.
    following = gap_time + 20_000_000
    axis, state, command = drive(following, 32, gap=True)
    assert not axis.lateralActive and not axis.longitudinalActive
    assert not command.latActive and not command.longActive
    assert state.enabled and not sd.aol_authority_lost
    assert EventName.controlsMismatch not in sd.events.names
    assert current['wire'].to_bytes() == old_native
    fresh = source + SOURCE_ADVANCE
    rows = physical(fresh, 34)
    card_receive(fresh, rows)
    assert card.aol_card_intent.allowed_latch
    recovered, state, command = drive(fresh + 1, 34)
    assert recovered.sourceCarStateMonoTime == fresh
    assert recovered.lateralActive and recovered.longitudinalActive
    assert command.latActive and command.longActive and state.enabled
    assert not sd.events.contains('enable')
    assert old.aolAxisState.sourceCarStateMonoTime == source
    assert old.aolAxisState.validUntilMonoTime < fresh

    apply_command(fresh + 2, command)
    # After the source returns, the NEXT actual C++/native observation must
    # continue both previously engaged axes; no stale positive wire is reused.
    intermediate = fresh + 10_000_000
    rows = physical(intermediate, 35)
    card_receive(intermediate, rows)
    _, _, command = drive(intermediate + 1, 35)
    apply_command(intermediate + 2, command)
    next_native = gap_time + 49_440_791
    assert sample_native(next_native) == 3
    final_source = next_native + 10_000_000
    rows = physical(final_source, 36)
    card_receive(final_source, rows)
    final, state, command = drive(final_source + 1, 36)
    assert state.enabled and final.lateralActive and final.longitudinalActive
    assert command.latActive and command.longActive and not sd.events.contains('enable')
    apply_command(final_source + 2, command)
