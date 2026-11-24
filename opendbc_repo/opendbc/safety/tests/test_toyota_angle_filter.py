"""The existing complete LTA/longitudinal safety suite on exact filter words."""
from opendbc.safety.tests import test_toyota as base


class TestToyotaRav4AngleFilter(base.TestToyotaSafetyAngle):
  SAFETY_PARAM = 0x1449
  RELAY_MALFUNCTION_ADDRS = {0: (0x2E4, 0x191, 0x412)}
  FWD_BLACKLISTED_ADDRS = {2: [0x2E4, 0x412, 0x191]}

  def test_diagnostics(self):
    super().test_diagnostics(ecu_disabled=False)

  def test_native_filter_exception_is_exact(self):
    for word, relay in ((0x1449, False), (0x1649, False), (0x449, True),
                        (0x1448, True), (0x144A, True), (0x1549, True)):
      with self.subTest(word=word):
        self.safety.set_safety_hooks(self.SAFETY_MODEL, word)
        self.safety.init_tests()
        self.assertTrue(self._rx(self._accel_msg_343(0)))
        self.assertEqual(self.safety.get_relay_malfunction(), relay)


class TestToyotaRav4AngleFilterStock(base.TestToyotaStockLongitudinalAngle):
  SAFETY_PARAM = 0x1649
