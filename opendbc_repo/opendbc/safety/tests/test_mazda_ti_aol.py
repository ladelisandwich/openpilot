"""Always-on-lateral for the fork's GEN1 with the Torque Interceptor and radar emulation (safety param 265).

MAIN and the cruise state come from PEDALS (the stock CRZ_CTRL is silenced with the radar), the stock EPS's LKAS_BLOCK
is not a source (it is set below about 45 km/h, where the interceptor steers), engaged steering and longitudinal need
no motion, and an AOL token still needs motion. Everything else is Dom's stock policy (mazda_stock_aol.h).
"""
import unittest

from opendbc.car import structs
from opendbc.can.dbc import DBC
from opendbc.safety.tests.common import CANPackerSafety

TI_RE = 1 | 8 | 256  # GEN1 | TORQUE_INTERCEPTOR | RADAR_EMULATION


class TestMazdaTiReAol(unittest.TestCase):
  def tearDown(self):
    self.safety.init_tests()
    self.safety.set_alternative_experience(0)
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)

  def configure(self, experience=32, word=TI_RE):
    from opendbc.safety.tests.libsafety import libsafety_py
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    self.safety.set_alternative_experience(experience)
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.mazda, word)
    self.safety.set_timer(1_000_000)
    self.safety.set_aol_test_heartbeat(True)
    self.packer = CANPackerSafety('mazda_2017')
    self.dbc = DBC('mazda_2017')

  def packet(self, name, fields, bus=0):
    self.assertFalse(fields.keys() - self.dbc.name_to_msg[name].sigs.keys())
    return self.packer.make_can_msg_safety(name, bus, fields)

  def feed(self, main=True, active=False, speed=80, brake=False, lkas_block=False, request=1, buttons=0):
    sources = (
      ('CRZ_BTNS', 0, {'RES': int(buttons == 1), 'SET_M': int(buttons == 2), 'CAN_OFF': int(buttons == 3)}),
      ('STEER_TORQUE', 0, {'STEER_TORQUE_SENSOR': 0}),
      ('ENGINE_DATA', 0, {'SPEED': speed, 'PEDAL_GAS': 0}),
      ('PEDALS', 0, {'ACC_OFF': int(main and not active), 'ACC_ACTIVE': int(active), 'BRAKE_ON': int(brake)}),
      ('GEAR', 0, {'GEAR': 4}),
      ('STEER_RATE', 0, {'LKAS_BLOCK': int(lkas_block)}),
      ('CAM_LKAS', 2, {'ERR_BIT_1': 0, 'LKAS_REQUEST': 0}),
      ('TI_FEEDBACK', 1, {'TI_TORQUE_SENSOR': 0}),
    )
    for name, bus, fields in sources:
      self.safety.safety_rx_hook(self.packet(name, fields, bus))
    self.safety.safety_tick()
    self.safety.aol_set_host_request(request)

  def arm(self, **kw):
    for _ in range(10):
      self.feed(main=True, **kw)
    self.feed(main=False, **kw)
    self.feed(main=True, **kw)

  def ti(self, torque):
    return self.safety.safety_tx_hook(self.packet('CAM_LKAS2', {'LKAS_REQUEST': torque, 'CHKSUM': torque, 'KEY': 3294744160}, 1))

  def test_main_edge_from_pedals_arms_lateral_despite_lkas_block(self):
    self.configure()
    self.feed(main=True, lkas_block=True)
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)   # no fresh physical edge yet
    self.assertFalse(self.ti(100))
    self.arm(lkas_block=True)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(self.safety.aol_get_permission_mask(), 1)
    self.assertTrue(self.ti(100))
    self.assertTrue(self.ti(-800))
    self.assertFalse(self.ti(801))

  def test_main_off_and_host_request_end_it(self):
    self.configure()
    self.arm()
    self.feed(main=False)
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)
    self.assertFalse(self.ti(100))
    self.arm()
    self.feed(request=0)
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)
    self.assertFalse(self.ti(100))

  def test_token_needs_motion_but_engaged_control_does_not(self):
    self.configure()
    self.arm()
    self.feed(speed=0)
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)   # AOL alone at a standstill: no
    self.assertFalse(self.ti(100))
    # engaged through PEDALS: both axes, also at a standstill and with the stock EPS's LKAS_BLOCK set
    self.feed(active=True, request=3)
    self.assertTrue(self.safety.get_controls_allowed())
    self.feed(active=True, speed=0, lkas_block=True, request=3)
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertEqual(self.safety.aol_get_permission_mask(), 3)
    self.assertTrue(self.ti(100))

  def test_brake_ends_longitudinal_and_keeps_lateral(self):
    self.configure()
    self.arm()
    self.feed(active=True, request=3)
    self.assertEqual(self.safety.aol_get_permission_mask(), 3)
    self.feed(active=False, brake=True, request=3)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(self.safety.aol_get_permission_mask(), 1)
    self.assertTrue(self.ti(100))

  def test_stock_crz_ctrl_is_not_a_main_source(self):
    self.configure()
    for _ in range(10):
      self.feed(main=False)
    for available in (0, 1, 0, 1):
      self.safety.safety_rx_hook(self.packet('CRZ_CTRL', {'CRZ_AVAILABLE': available, 'CRZ_ACTIVE': 0}))
      self.safety.safety_rx_hook(self.packet('CRZ_CTRL', {'CRZ_AVAILABLE': available, 'CRZ_ACTIVE': 1}, 1))
    self.feed(main=False)
    self.assertFalse(self.safety.get_controls_allowed())   # bus 1 CRZ_CTRL no longer arms controls either
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)

  def test_without_the_experience_it_stays_the_normal_mode(self):
    self.configure(experience=0)
    self.arm()
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)
    self.assertFalse(self.ti(100))
    self.safety.set_controls_allowed(1)
    self.assertTrue(self.ti(100))

  def test_other_flagged_words_stay_unarmed(self):
    for word in (1 | 8, 1 | 256, 1 | 8 | 16, 1 | 8 | 256 | 32):
      self.configure(word=word)
      self.arm()
      self.assertEqual(self.safety.aol_get_permission_mask(), 0, word)


if __name__ == "__main__":
  unittest.main()
