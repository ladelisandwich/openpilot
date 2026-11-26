#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import make_msg


class TestMazdaSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):

  DBC = "mazda_2017"
  SAFETY_MODEL = CarParams.SafetyModel.mazda

  TX_MSGS = [[0x243, 0], [0x09d, 0], [0x440, 0]]
  STANDSTILL_THRESHOLD = .1
  RELAY_MALFUNCTION_ADDRS = {0: (0x243, 0x440)}
  FWD_BLACKLISTED_ADDRS = {2: [0x243, 0x440]}

  MAX_RATE_UP = 10
  MAX_RATE_DOWN = 25
  MAX_TORQUE_LOOKUP = [0], [800]

  MAX_RT_DELTA = 300

  DRIVER_TORQUE_ALLOWANCE = 15
  DRIVER_TORQUE_FACTOR = 1

  # Mazda actually does not set any bit when requesting torque
  NO_STEER_REQ_BIT = True

  def _torque_meas_msg(self, torque):
    values = {"STEER_TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"STEER_TORQUE_SENSOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_REQUEST": torque}
    return self.packer.make_can_msg_safety("CAM_LKAS", 0, values)

  def _speed_msg(self, speed):
    values = {"SPEED": speed}
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_ON": brake}
    return self.packer.make_can_msg_safety("PEDALS", 0, values)

  def _user_gas_msg(self, gas):
    values = {"PEDAL_GAS": gas}
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRZ_ACTIVE": enable}
    return self.packer.make_can_msg_safety("CRZ_CTRL", 0, values)

  def _button_msg(self, resume=False, cancel=False):
    values = {
      "CAN_OFF": cancel,
      "CAN_OFF_INV": (cancel + 1) % 2,
      "RES": resume,
      "RES_INV": (resume + 1) % 2,
    }
    return self.packer.make_can_msg_safety("CRZ_BTNS", 0, values)

  def test_buttons(self):
    # only cancel allows while controls not allowed
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertFalse(self._tx(self._button_msg(resume=True)))

    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertTrue(self._tx(self._button_msg(resume=True)))


class TestMazdaTiRadarEmulationSafety(TestMazdaSafety):
  """The CX-9 configuration: GEN1 + Torque Interceptor on bus 1 + radar emulation (safety param 265).
  Engagement follows PEDALS (the stock CRZ_CTRL is silenced with the radar), driver torque comes from the TI."""

  SAFETY_PARAM = 1 | 8 | 256  # GEN1 | TORQUE_INTERCEPTOR | RADAR_EMULATION

  RADAR_EMU_ADDRS = [0x21b, 0x21c, 0x499, 0x361, 0x362, 0x363, 0x364, 0x365, 0x366]
  TX_MSGS = [[0x243, 0], [0x249, 1], [0x09d, 0], [0x440, 0], [0x764, 0],
             *[[addr, bus] for addr in RADAR_EMU_ADDRS for bus in (0, 2)]]
  TI_MAX_TORQUE = 800

  def setUp(self):
    self._acc_active = False
    self._brake = False
    super().setUp()

  def _pedals_msg(self):
    values = {"ACC_ACTIVE": self._acc_active, "ACC_OFF": self._acc_active, "BRAKE_ON": self._brake}
    return self.packer.make_can_msg_safety("PEDALS", 0, values)

  def _pcm_status_msg(self, enable):
    self._acc_active = enable
    return self._pedals_msg()

  def _user_brake_msg(self, brake):
    self._brake = brake
    return self._pedals_msg()

  def _torque_driver_msg(self, torque):
    values = {"TI_TORQUE_SENSOR": torque}
    return self.packer.make_can_msg_safety("TI_FEEDBACK", 1, values)

  def _ti_torque_cmd_msg(self, torque, copy=None):
    values = {"LKAS_REQUEST": torque, "CHKSUM": torque if copy is None else copy, "KEY": 3294744160}
    return self.packer.make_can_msg_safety("CAM_LKAS2", 1, values)

  def _radar_session_msg(self, session_type):
    return make_msg(0, 0x764, dat=bytes([0x02, 0x10, session_type, 0, 0, 0, 0, 0]))

  def test_ti_torque_requires_controls(self):
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._ti_torque_cmd_msg(0)))
    for torque in (1, -1, 100, -100, self.TI_MAX_TORQUE, -self.TI_MAX_TORQUE):
      self.assertFalse(self._tx(self._ti_torque_cmd_msg(torque)), torque)

  def test_ti_torque_max(self):
    self.safety.set_controls_allowed(1)
    for torque in range(-2048, 2048):
      allowed = abs(torque) <= self.TI_MAX_TORQUE
      self.assertEqual(allowed, self._tx(self._ti_torque_cmd_msg(torque)), torque)

  def test_ti_torque_copy_must_match(self):
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._ti_torque_cmd_msg(300)))
    self.assertFalse(self._tx(self._ti_torque_cmd_msg(300, copy=301)))
    self.assertFalse(self._tx(self._ti_torque_cmd_msg(0, copy=800)))
    self.safety.set_controls_allowed(0)
    self.assertFalse(self._tx(self._ti_torque_cmd_msg(0, copy=1)))

  def test_ti_torque_has_no_rate_limit(self):
    # The TI is sent 0 while LKAS steers (TI not in RUN) and resumes at the LKAS command in one step.
    self.safety.set_controls_allowed(1)
    for _ in range(6):
      self._rx(self._torque_driver_msg(0))
    for lkas in range(0, self.MAX_RT_DELTA - self.MAX_RATE_UP, self.MAX_RATE_UP):
      self.assertTrue(self._tx(self._torque_cmd_msg(lkas)))
      self.assertTrue(self._tx(self._ti_torque_cmd_msg(0)))
    self.assertTrue(self._tx(self._ti_torque_cmd_msg(lkas)))
    self.assertTrue(self._tx(self._ti_torque_cmd_msg(-self.TI_MAX_TORQUE)))
    self.assertTrue(self._tx(self._ti_torque_cmd_msg(self.TI_MAX_TORQUE)))
    # and the TI frames leave the LKAS rate state alone
    self.assertTrue(self._tx(self._torque_cmd_msg(lkas + self.MAX_RATE_UP)))

  def test_ti_torque_not_checked_without_ti(self):
    # without the TI flag 0x249 is not in the transmit table at all
    self.safety.set_safety_hooks(self.SAFETY_MODEL, 1 | 256)
    self.safety.set_controls_allowed(1)
    self.assertFalse(self._tx(self._ti_torque_cmd_msg(0)))

  def test_radar_programming_session_requires_controls(self):
    self.safety.set_controls_allowed(0)
    self.assertFalse(self._tx(self._radar_session_msg(0x02)))
    self.assertTrue(self._tx(self._radar_session_msg(0x01)))
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._radar_session_msg(0x02)))
    self.assertTrue(self._tx(self._radar_session_msg(0x01)))

  def test_radar_tester_present_always_allowed(self):
    tester_present = make_msg(0, 0x764, dat=bytes([0x02, 0x3E, 0x80, 0, 0, 0, 0, 0]))
    for allowed in (0, 1):
      self.safety.set_controls_allowed(allowed)
      self.assertTrue(self._tx(tester_present))

  def test_radar_uds_allowlist(self):
    self.safety.set_controls_allowed(1)
    for session_type in (0x00, 0x03, 0x04, 0x60):
      self.assertFalse(self._tx(self._radar_session_msg(session_type)), session_type)
    for dat in ([0x02, 0x11, 0x01], [0x02, 0x28, 0x03], [0x02, 0x3E, 0x00], [0x03, 0x10, 0x02]):
      self.assertFalse(self._tx(make_msg(0, 0x764, dat=bytes(dat + [0] * 5))), dat)


class TestMazdaIgnition(common.SafetyTestBase):
  SAFETY_MODEL = None

  TX_MSGS: list = []

  def _msg(self, byte0):
    return make_msg(0, 0x9E, dat=bytes([byte0]) + b"\x00" * 7)

  # 0x9E byte 0 high 3 bits == 6 (0xC0)
  def test_ignition_on(self):
    self.safety.ignition_can_hook(self._msg(0xC0))
    self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_off(self):
    self.safety.ignition_can_hook(self._msg(0xC0))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(0x20))
    self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()
