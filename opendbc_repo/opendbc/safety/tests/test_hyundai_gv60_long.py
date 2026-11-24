"""GV60 heartbeat follows current native sources, integrity and exact profile admission."""
import unittest
from opendbc.can import CANPacker
from opendbc.car.structs import CarParams
from opendbc.car.hyundai.hyundaicanfd import create_gv60_radar_heartbeat, hkg_can_fd_checksum
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.test_hyundai_ev9_long import Ev9LongFixture
from opendbc.safety.tests.test_ioniq5pe_paired_native import release_safety


class GV60Fixture:
  RELEASE_NATIVE = False
  packet = Ev9LongFixture.packet
  feed = Ev9LongFixture.feed

  def setUp(self):
    self.safety = release_safety() if self.RELEASE_NATIVE else libsafety_py.libsafety
    self.packer = CANPacker('hyundai_canfd_generated')
    self.initialize(0x9015)

  def initialize(self, word, experience=0):
    self.safety.init_tests()
    self.safety.set_alternative_experience(experience)
    self.assertEqual(self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, word), 0)
    self.now = 1_000_000
    self.safety.set_timer(self.now)

  def heartbeat(self, counter, brake=False, gas=False):
    addr, data, bus = create_gv60_radar_heartbeat(counter, brake, gas)
    return libsafety_py.make_CANPacket(addr, bus, data)

  def warm(self):
    for _ in range(6):
      self.feed()


class TestGV60LongDebug(GV60Fixture, unittest.TestCase):
  def test_both_catalogs_require_current_sources_crc_counter_and_frozen_body(self):
    for word in (0x9015, 0x9095):
      with self.subTest(word=hex(word)):
        self.initialize(word)
        self.assertFalse(self.safety.safety_tx_hook(self.heartbeat(1)))
        self.warm()
        self.assertTrue(self.safety.safety_tx_hook(self.heartbeat(1)))
        self.assertFalse(self.safety.safety_tx_hook(self.heartbeat(1)))
        corrupt = self.heartbeat(2)
        corrupt[0].data[0] ^= 1
        self.assertFalse(self.safety.safety_tx_hook(corrupt))
        for offset in range(3, 24):
          addr, body, bus = create_gv60_radar_heartbeat(2, False, False)
          data = bytearray(body)
          data[offset] ^= 2
          data[:2] = hkg_can_fd_checksum(addr, None, data).to_bytes(2, 'little')
          self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, data)), offset)
        self.assertTrue(self.safety.safety_tx_hook(self.heartbeat(2)))
        self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x100, 1,
                         create_gv60_radar_heartbeat(3, False, False)[1])))
        self.now += 300_001
        self.safety.set_timer(self.now)
        self.assertFalse(self.safety.safety_tx_hook(self.heartbeat(3)))

  def test_actual_pedal_state_is_required_without_granting_control(self):
    for brake, gas in ((True, False), (False, True), (True, True)):
      with self.subTest(brake=brake, gas=gas):
        self.initialize(0x9015)
        self.warm()
        self.assertTrue(self.safety.safety_rx_hook(self.packet('TCS', 1, {'DriverBraking': int(brake)})))
        self.assertTrue(self.safety.safety_rx_hook(self.packet('ACCELERATOR', 1,
                        {'GEAR': 5, 'ACCELERATOR_PEDAL': int(gas)})))
        self.safety.set_controls_allowed(False)
        self.assertFalse(self.safety.safety_tx_hook(self.heartbeat(1, not brake, gas)))
        self.assertFalse(self.safety.safety_tx_hook(self.heartbeat(1, brake, not gas)))
        self.assertTrue(self.safety.safety_tx_hook(self.heartbeat(1, brake, gas)))
        self.assertFalse(self.safety.get_controls_allowed())

  def test_foreign_catalog_rejection_cannot_extend_another_profile(self):
    for alternate in (False, True):
      self.assertTrue(self.safety.safety_test_hyundai_gv60_catalog_rejection(alternate))

  def test_unqualified_experience_and_neighbor_profiles_cannot_send_heartbeat(self):
    for word, experience in ((0x9015, 32), (0x9095, 1), (0x9014, 0), (0x9094, 0), (0x15, 0), (0x95, 0)):
      with self.subTest(word=hex(word), experience=experience):
        self.initialize(word, experience)
        self.warm()
        self.safety.set_controls_allowed(True)
        self.assertFalse(self.safety.safety_tx_hook(self.heartbeat(1)))


class TestGV60LongReleaseDenied(GV60Fixture, unittest.TestCase):
  RELEASE_NATIVE = True

  def test_debug_profile_cannot_transmit_heartbeat_or_positive_acceleration(self):
    for word in (0x9015, 0x9095):
      self.initialize(word)
      self.warm()
      self.safety.set_controls_allowed(True)
      self.assertFalse(self.safety.safety_tx_hook(self.heartbeat(1)))
      self.assertFalse(self.safety.safety_tx_hook(self.packet('SCC_CONTROL', 1,
                       {'ACCMode': 1, 'aReqRaw': 1., 'aReqValue': 1.})))
      self.assertFalse(self.safety.safety_test_hyundai_gv60_catalog_rejection(False))
