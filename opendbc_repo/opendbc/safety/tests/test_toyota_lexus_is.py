import unittest
from opendbc.safety.tests import common
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests import test_toyota as toyota_tests


class TestLexusIsNative(unittest.TestCase):
  init = toyota_tests.TestToyotaHighlanderAol.init
  rx = toyota_tests.TestToyotaHighlanderAol.rx
  tearDown = toyota_tests.TestToyotaHighlanderAol.tearDown

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.packer = common.CANPackerSafety('toyota_tnga_k_pt_generated')

  def physical(self, *, main=True, cruise=True, brake=False, gas=False, gear=0, eps=1, door=False):
    self.rx(0xAA, bytes([0x1A, 0x6F] * 4))
    self.rx(0x260, bytes(8), True)
    self.rx(0x1D2, bytes([(0 if gas else 16) | (32 if cruise else 0)]) + bytes(7), True)
    self.rx(0x226, bytes(4) + bytes([32 if brake else 0]) + bytes(3))
    self.rx(0x365, bytes([1 if main else 0]) + bytes(6))
    self.rx(0x3BC, bytes([0, gear]) + bytes(6))
    self.rx(0x620, bytes(5) + bytes([4 if door else 0, 0, 0]))
    self.assertTrue(self.safety.safety_rx_hook(self.packer.make_can_msg_safety('EPS_STATUS', 0, {'LKA_STATE': eps})))
    self.safety.aol_set_host_request(3)

  def test_real_sources_and_stock_longitudinal_withdrawal(self):
    self.init(0x424D, 32)
    self.physical()
    self.assertEqual(self.safety.aol_get_request_mask(), 1)
    self.assertEqual(self.safety.aol_get_permission_mask(), 1)
    self.assertTrue(self.safety.get_acc_main_on())
    for fields, allowed in (({'ACCEL_CMD': 0, 'CANCEL_REQ': 1}, True),
                            ({'ACCEL_CMD': 0, 'CANCEL_REQ': 0}, False),
                            ({'ACCEL_CMD': .1, 'CANCEL_REQ': 1}, False)):
      self.assertEqual(bool(self.safety.safety_tx_hook(self.packer.make_can_msg_safety('ACC_CONTROL', 0, fields))), allowed)
    self.physical(brake=True)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(self.safety.aol_get_permission_mask(), 1)
    self.physical(main=False)
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)
    # PCM_CRUISE_2 must never revive the DSU-owned MAIN source.
    self.rx(0x1D3, bytes([0, 128]) + bytes(6), True)
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)

  def test_reverse_fault_source_length_bus_and_stale_denials(self):
    for changes in ({'gear': 16}, {'eps': 9}, {'eps': 17}, {'door': True}):
      self.init(0x424D, 32)
      self.physical()
      self.assertEqual(self.safety.aol_get_permission_mask(), 1)
      self.physical(**changes)
      self.assertEqual(self.safety.aol_get_permission_mask(), 0)
      self.physical()
      self.assertEqual(self.safety.aol_get_permission_mask(), 1)
    for address, length in ((0x365, 8), (0x262, 8)):
      self.init(0x424D, 32)
      self.physical()
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(address, 0, bytes(length)))
      self.assertEqual(self.safety.aol_get_permission_mask(), 0)
    self.init(0x424D, 32)
    self.physical(main=False)
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x365, 2, bytes([1]) + bytes(6)))
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)
    self.physical()
    self.safety.set_timer(1_303_032)
    self.safety.aol_set_host_request(1)
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)

  def test_exact_profile_and_explicit_off(self):
    for word in (0x404D, 0x424C, 0x424E, 0x434D, 0x524D):
      self.init(word, 32)
      self.physical()
      self.assertEqual(self.safety.aol_get_permission_mask(), 0)
      self.assertFalse(self.safety.safety_tx_hook(self.packer.make_can_msg_safety('STEERING_LKA', 0, {'STEER_REQUEST': 1, 'STEER_TORQUE_CMD': 1})))
    self.init(0x424D, 0)
    self.physical()
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)
    self.safety.set_controls_allowed(False)
    self.assertFalse(self.safety.safety_tx_hook(self.packer.make_can_msg_safety('STEERING_LKA', 0, {'STEER_REQUEST': 1, 'STEER_TORQUE_CMD': 1})))

  def test_shared_workspace_cold_switch_and_positive_steering(self):
    for word in (0x424D, 73, 0x9049, 0x2049, 0x424D):
      self.init(word, 32)
      self.assertFalse(self.safety.safety_config_valid())
      self.assertEqual(self.safety.aol_get_permission_mask(), 0)
      if word == 0x424D:
        self.physical()
        self.assertTrue(self.safety.safety_config_valid())
        self.assertEqual(self.safety.aol_get_permission_mask(), 1)
        self.assertTrue(self.safety.safety_tx_hook(self.packer.make_can_msg_safety(
          'STEERING_LKA', 0, {'STEER_REQUEST': 1, 'STEER_TORQUE_CMD': 1})))
