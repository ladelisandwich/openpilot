import unittest
from opendbc.safety.tests import common, test_toyota
from opendbc.safety.tests.libsafety import libsafety_py


class TestToyotaCamera2aaIdentity(unittest.TestCase):
  init = test_toyota.TestToyotaHighlanderAol.init
  tearDown = test_toyota.TestToyotaHighlanderAol.tearDown

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.packer = common.CANPackerSafety("toyota_nodsu_pt_generated")
    self.base = test_toyota.TestToyotaHighlanderAol()
    self.base.safety = self.safety
    self.base.packer = self.packer

  def test_contradictory_identity_requires_reconfiguration(self):
    contradictions = ((0x201, 0, bytes(6)), (0x2FF, 0, bytes(4)),
                      (0x2AA, 0, bytes(7)), (0x2AA, 1, bytes(8)))
    for word, permission in ((0xC049, 3), (0xC249, 1)):
      for ae in (0, 32):
        for address, bus, data in contradictions:
          with self.subTest(word=hex(word), ae=ae, address=hex(address), bus=bus, length=len(data)):
            self.init(word, ae)
            steer = self.packer.make_can_msg_safety("STEERING_LKA", 0, {"STEER_REQUEST": 1, "STEER_TORQUE_CMD": 0})
            accel = self.packer.make_can_msg_safety("ACC_CONTROL", 0, {"ACCEL_CMD": .1})
            cancel = self.packer.make_can_msg_safety("ACC_CONTROL", 0, {"CANCEL_REQ": 1})

            def earn():
              # A correctly shaped discriminator on either allowed bus is not a lease.
              for allowed_bus in (0, 2):
                self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x2AA, allowed_bus, bytes(8))))
              self.base.physical(cruise=False)
              self.base.physical(cruise=True)
              self.safety.safety_tick()

            self.assertFalse(self.safety.safety_tx_hook(steer))
            self.assertFalse(self.safety.safety_tx_hook(accel))
            earn()
            self.assertEqual(self.safety.aol_get_permission_mask(), permission if ae else 0)
            self.assertTrue(self.safety.safety_tx_hook(steer))
            self.assertEqual(bool(self.safety.safety_tx_hook(accel)), word == 0xC049)
            self.assertEqual(self.safety.safety_fwd_hook(2, 0x343), 0)
            self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data)))
            # Valid physical sources and a cruise cycle cannot erase the contradiction.
            earn()
            self.assertTrue(self.safety.safety_config_valid())
            self.assertEqual(self.safety.aol_get_permission_mask(), 0)
            self.assertFalse(self.safety.safety_tx_hook(steer))
            self.assertFalse(self.safety.safety_tx_hook(accel))
            self.assertTrue(self.safety.safety_tx_hook(cancel))
            self.init(word, ae)
            self.assertFalse(self.safety.safety_tx_hook(steer))
            earn()
            self.assertEqual(self.safety.aol_get_permission_mask(), permission if ae else 0)
            self.assertTrue(self.safety.safety_tx_hook(steer))
            self.assertEqual(bool(self.safety.safety_tx_hook(accel)), word == 0xC049)


if __name__ == "__main__":
  unittest.main()
