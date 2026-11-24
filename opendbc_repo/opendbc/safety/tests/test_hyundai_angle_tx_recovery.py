"""Real parser/controller/native rejection recovery and echo provenance guards."""
import unittest

import pytest

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.hyundai.angle_tx_recovery import AngleTxRecovery
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.tests.test_stock_angle_aol import source_profile
from opendbc.car.hyundai.values import CAR
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.starpilot.aol.tests.test_canfd_location_native_recovery import physical_frames


class TestAngleTxEcho(unittest.TestCase):
  @staticmethod
  def frame():
    body = bytearray(32)
    body[9] = 0x20
    return 0x110, bytes(body), 0

  def test_only_current_exact_local_rejected_command_requests_one_reset(self):
    address, data, bus = self.frame()
    for delta in (1, 2, 3):
      recovery = AngleTxRecovery(address, bus)
      recovery.record([(address, data, bus)], 10)
      recovery.observe([(100, [(address, data, bus + 0xC0)])], 10 + delta)
      self.assertTrue(recovery.take_rejection(10 + delta))
      self.assertFalse(recovery.take_rejection(10 + delta))
      recovery.observe([(101, [(address, data, bus + 0xC0)])], 11 + delta)
      self.assertFalse(recovery.take_rejection(11 + delta))

  def test_stale_wrong_returned_or_unmatched_echo_never_resets(self):
    address, data, bus = self.frame()
    for case in ('returned', 'wrong_bus', 'wrong_address', 'wrong_data', 'short', 'old_frame',
                 'same_frame', 'unknown', 'inactive', 'old_packet', 'late_consume'):
      recovery = AngleTxRecovery(address, bus)
      emitted = bytearray(data)
      if case == 'inactive':
        emitted[9] = 0x10
      if case != 'unknown':
        recovery.record([(address, bytes(emitted), bus)], 10)
      echo = (address + (case == 'wrong_address'), bytes([data[0] ^ 1]) + data[1:] if case == 'wrong_data' else
              data[:-1] if case == 'short' else data, bus + (0x80 if case == 'returned' else 0xC1 if case == 'wrong_bus' else 0xC0))
      frame = 14 if case == 'old_frame' else 10 if case == 'same_frame' else 11
      if case == 'old_packet':
        recovery.observe([(100, [])], frame)
      recovery.observe([(100, [echo])], frame)
      self.assertFalse(recovery.take_rejection(frame + (case == 'late_consume')), case)

  def test_feedback_uses_original_local_send_deadline_even_when_controller_is_paused(self):
    for case in ('expired_send', 'expired_consume', 'reversed'):
      clock = [100_000_000]
      recovery = AngleTxRecovery(0x110, 0, clock=lambda values=clock: values[0])
      address, data, bus = self.frame()
      recovery.record([(address, data, bus)], 10)
      clock[0] += 30_000_001 if case == 'expired_send' else -1 if case == 'reversed' else 10_000_000
      recovery.observe([(100, [(address, data, bus + 0xC0)])], 11)
      if case == 'expired_consume':
        clock[0] = 130_000_001
      self.assertFalse(recovery.take_rejection(11), case)


@pytest.mark.parametrize('identity', [CAR.HYUNDAI_IONIQ_5_PE, CAR.KIA_EV9])
@pytest.mark.parametrize('old,measured,speed', [(-19.5, -14.7, 5.336), (-4.7, -6.9, 9.212), (-6.5, -3.1, 20.822)])
def test_actual_stock_angle_controller_recovers_after_local_native_rejection(identity, old, measured, speed):
  cp = source_profile(identity)
  cp.alternativeExperience = 32
  ci = CarInterface(cp)
  native = libsafety_py.libsafety
  native.set_alternative_experience(32)
  assert native.set_safety_hooks(structs.CarParams.SafetyModel.hyundaiCanfd, cp.safetyConfigs[0].safetyParam) == 0
  native.init_tests()
  native.set_alternative_experience(32)
  packer = CANPacker('hyundai_canfd_generated')
  echo = []

  def receive(tick, lkas=0, button=0):
    now_ns = 1_000_000_000 + tick * 10_000_000
    native.set_timer(now_ns // 1000)
    frames = physical_frames(ci, tick, lkas=lkas, button=button)
    replaced = []
    for address, data, bus in frames:
      name = packer.dbc.addr_to_msg[address].name
      values = None
      if name == 'MDPS':
        values = {'MDPS_PaStrAnglVal': measured, 'MDPS_EstStrAnglVal': measured,
                  'MDPS_StrTqSnsrVal': 0, 'MDPS_OutTqVal': 0, 'COUNTER': tick % 256}
      elif name == 'STEERING_SENSORS':
        values = {'STEERING_ANGLE': measured, 'COUNTER': tick % 256}
      elif name == 'WHEEL_SPEEDS':
        values = {**dict.fromkeys(('WHL_SpdFLVal', 'WHL_SpdFRVal', 'WHL_SpdRLVal', 'WHL_SpdRRVal'), speed * 3.6),
                  'COUNTER': tick % 256}
      frame = packer.make_can_msg(name, bus, values) if values is not None else (address, data, bus)
      assert native.safety_rx_hook(libsafety_py.make_CANPacket(frame[0], frame[2], frame[1])), (hex(address), bus)
      replaced.append(frame)
    if tick % 10 == 0:
      native.aol_set_host_request(1)
      native.set_aol_test_heartbeat(True)
      native.safety_tick()
    state = ci.update([(now_ns, replaced + echo)])
    return now_ns, state

  for tick in range(1, 51):
    now_ns, state = receive(tick, lkas=int(tick in (40, 41)))
  assert state.canValid and not state.canTimeout
  assert native.aol_get_permission_mask() == 1
  assert abs(state.steeringAngleDeg - measured) < .11
  ci.CC.apply_angle_last = old
  command = structs.CarControl(latActive=True)
  command.actuators.steeringAngleDeg = old - 1 if old < measured else old + 1
  _, frames = ci.apply(command.as_reader(), now_ns)
  steering = [frame for frame in frames if frame[0] == 0x110]
  assert len(steering) == 1
  first = steering[0]
  assert not native.safety_tx_hook(libsafety_py.make_CANPacket(first[0], first[2], first[1]))
  echo[:] = [(first[0], first[1], first[2] + 0xC0)]
  accepted = False
  for tick in range(51, 55):
    now_ns, state = receive(tick)
    assert state.canValid
    _, frames = ci.apply(command.as_reader(), now_ns)
    steering = [frame for frame in frames if frame[0] == 0x110]
    assert len(steering) == 1
    frame = steering[0]
    accepted = bool(native.safety_tx_hook(libsafety_py.make_CANPacket(frame[0], frame[2], frame[1])))
    if accepted:
      break
    echo[:] = [(frame[0], frame[1], frame[2] + 0xC0)]
  assert accepted, 'Host command must recover from measured angle while every unchanged native limiter still runs'
  assert native.aol_get_permission_mask() == 1

  # The unchanged native physical CANCEL keeps already-authorized independent
  # lateral. Configured LKAS OFF belongs to the host, whose zero request
  # must still veto an active controller command.
  cancel_tick = tick + 2 - tick % 2
  now_ns, state = receive(cancel_tick, button=4)
  assert native.aol_get_permission_mask() == 1
  _, frames = ci.apply(command.as_reader(), now_ns)
  frame = next(frame for frame in frames if frame[0] == 0x110)
  assert native.safety_tx_hook(libsafety_py.make_CANPacket(frame[0], frame[2], frame[1]))
  now_ns, state = receive(cancel_tick + 2, lkas=1)
  native.aol_set_host_request(0)
  assert native.aol_get_permission_mask() == 0
  _, frames = ci.apply(command.as_reader(), now_ns)
  frame = next(frame for frame in frames if frame[0] == 0x110)
  assert not native.safety_tx_hook(libsafety_py.make_CANPacket(frame[0], frame[2], frame[1]))
