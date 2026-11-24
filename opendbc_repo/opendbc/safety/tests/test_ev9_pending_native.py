import unittest

from opendbc.car import structs
from opendbc.car.hyundai.tests.test_ev9_pending_startup import EV9StartupFixture as PEStartupFixture
from opendbc.car.hyundai.tests.test_ev9_long_controller import packets
from opendbc.safety.tests.test_hyundai_ev9_long import Ev9LongFixture as PEFixture
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.starpilot.longitudinal.extension import LongitudinalContext


class TestEV9PendingControllerNativeDebug(PEStartupFixture, PEFixture, unittest.TestCase):
  @staticmethod
  def can_packet(frame):
    return libsafety_py.make_CANPacket(frame[0], frame[2], frame[1])

  def setUp(self):
    PEStartupFixture.setUp(self)
    PEFixture.setUp(self)

    self.assertEqual(self.safety.set_safety_hooks(structs.CarParams.SafetyModel.elm327, 1), 0)

  def physical_tick(self, ci, packer, *, button=0, adaptive_main=False, native_hyundai=True):
    self.tick += 1
    self.boot += 10_000_000
    self.now += 10_000
    self.safety.set_timer(self.now)
    frames = packets(packer, self.tick, advance_counter=True, cruise_button=button, adaptive_main=adaptive_main)
    if native_hyundai:
      for frame in frames:
        accepted = self.safety.safety_rx_hook(libsafety_py.make_CANPacket(frame[0], frame[2], frame[1]))
        if frame[0] in (0x35, 0x175, 0xA0, 0xEA, 0x1CF):
          self.assertTrue(accepted, hex(frame[0]))
    ci.update([(self.boot, frames)])

  def test_deferred_owner_to_real_controller_wire_native_then_physical_cancel(self):
    owner, ci, packer = self.instance()
    for _ in range(12):
      self.physical_tick(ci, packer, native_hyundai=False)
    self.assertTrue(ci.CS.out.canValid)
    self.assertEqual(self.queries, [])
    self.commit(owner, ci)
    self.assertEqual(self.safety.set_safety_hooks(structs.CarParams.SafetyModel.hyundaiCanfd, 0x5c95), 0)
    for _ in range(12):
      self.physical_tick(ci, packer)
    self.assertFalse(self.safety.get_controls_allowed())
    self.acknowledge(owner, ci)
    cc = structs.CarControl(enabled=True, latActive=True, longActive=True)
    cc.actuators.accel = 1.
    cc.actuators.steeringAngleDeg = 1.
    cc.actuators.longControlState = structs.CarControl.Actuators.LongControlState.pid
    _, first = ci.apply(cc.as_reader(), self.boot)
    scc = next(frame[1] for frame in first if frame[0] == 0x1A0)
    cb = next(frame[1] for frame in first if frame[0] == 0xCB)
    self.assertEqual((scc[8] >> 4) & 7, 0)
    self.assertEqual((cb[3] >> 4) & 15, 1)
    for frame in first:
      self.assertTrue(self.safety.safety_tx_hook(self.can_packet(frame)), hex(frame[0]))
    owner.sent(first, valid=True)
    self.assertTrue(owner.handed_off)
    # EV9 retains its original tracked MAIN policy; physical MAIN precedes SET.
    while (self.tick + 1) % 2:
      self.physical_tick(ci, packer)
    self.physical_tick(ci, packer, adaptive_main=True)
    self.physical_tick(ci, packer, adaptive_main=True)
    self.physical_tick(ci, packer)
    self.assertFalse(self.safety.get_controls_allowed())
    positive = None
    loop = LongControl(ci.CP)
    context = LongitudinalContext(has_lead=False, traffic_mode=False, custom_acceleration=False,
                                  profile_max_accel=0., experimental_mode=False)
    press_tick = self.tick + (2 if self.tick % 2 == 0 else 1)
    for index in range(40):

      next_tick = self.tick + 1
      button = 2 if next_tick == press_tick else 0
      self.physical_tick(ci, packer, button=button)
      self.acknowledge(owner, ci)

      active = bool(self.safety.get_controls_allowed())
      cc.actuators.accel = float(loop.update(active, ci.CS.out, 1., False, (-3.5, 2.), context=context))
      cc.actuators.longControlState = loop.long_control_state
      command = cc.as_reader() if active else structs.CarControl.new_message().as_reader()
      _, emitted = ci.apply(command, self.boot)
      for frame in emitted:
        self.assertTrue(self.safety.safety_tx_hook(self.can_packet(frame)), (index, hex(frame[0])))
        if frame[0] == 0x1A0:
          data = frame[1]
          accel = ((((data[17] & 7) << 8) | data[16]) - 1023) / 100.
          self.assertGreaterEqual(accel, -3.5)
          self.assertLessEqual(accel, 2.)
          if accel > 0:
            positive = frame
    self.assertIsNotNone(positive)
    self.assertTrue(self.safety.get_controls_allowed())

    while (self.tick + 1) % 2:
      self.physical_tick(ci, packer)
    self.physical_tick(ci, packer, button=4)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.safety_tx_hook(self.can_packet(positive)))
