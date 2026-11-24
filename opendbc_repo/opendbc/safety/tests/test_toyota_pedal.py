import unittest
from opendbc.can import CANPacker
from opendbc.car import structs, Bus
from opendbc.car.toyota import pedal
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, DBC
from opendbc.car.toyota.fingerprints import FW_VERSIONS
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests import test_toyota as toyota_tests


def firmware(candidate=CAR.TOYOTA_COROLLA_TSS2):
  return [
    structs.CarParams.CarFw(ecu=e, address=a, subAddress=s or 0, fwVersion=v[0], brand="toyota", bus=0)
    for (e, a, s), v in FW_VERSIONS[candidate].items()
  ]


def factory(candidate=CAR.TOYOTA_COROLLA_TSS2):
  return CarInterface.get_params(candidate, {0: {0x201: 6}, 1: {}, 2: {}}, firmware(candidate), False, False, False)


class TestPairedPedal(unittest.TestCase):
  candidate = CAR.TOYOTA_COROLLA_TSS2

  def setUp(self):
    self.cp = factory(self.candidate)
    self.packer = CANPacker(DBC[self.cp.carFingerprint][Bus.pt])
    self.safety = libsafety_py.libsafety
    self.base = toyota_tests.TestToyotaHighlanderAol()
    self.base.safety = self.safety
    self.base.packer = toyota_tests.common.CANPackerSafety(DBC[self.cp.carFingerprint][Bus.pt])
    self.counter = 0

  def sensor(self, gas=0, state=0, bad_crc=False):
    a, d, b = self.packer.make_can_msg("GAS_SENSOR", 0, {"INTERCEPTOR_GAS": gas, "INTERCEPTOR_GAS2": gas, "STATE": state, "COUNTER_PEDAL": self.counter})
    self.counter = (self.counter + 1) & 15
    d = bytearray(d)
    d[-1] = pedal.checksum(d) ^ int(bad_crc)
    return a, bytes(d), b

  def rx(self, frame):
    a, d, b = frame
    return self.safety.safety_rx_hook(libsafety_py.make_CANPacket(a, b, d))

  def tx(self, value, counter=0):
    a, d, b = pedal.command(self.packer, value, counter)
    return bool(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(a, b, d)))

  def init(self, word=0x2049):
    self.base.init(word, 32)
    self.base.physical(cruise=True)
    self.rx(self.sensor())

  def test_factory_current_words_and_exact_negatives(self):
    self.assertTrue(pedal.configured(self.cp))
    self.assertEqual(self.cp.safetyConfigs[0].safetyParam, 0x2049)
    for fp in ({0: {0x201: 7}, 1: {}, 2: {}}, {0: {}, 1: {0x201: 6}, 2: {}}, {0: {0x201: 5, 0x2FF: 8}, 1: {}, 2: {}}):
      cp = CarInterface.get_params(self.candidate, fp, firmware(self.candidate), False, False, False)
      self.assertFalse(pedal.configured(cp))
    self.assertTrue(pedal.prepare_stock(self.cp))
    self.assertEqual(self.cp.safetyConfigs[0].safetyParam, 0x2249)
    self.assertFalse(self.cp.openpilotLongitudinalControl)

  def test_parser_crc_counter_dlc_state_and_source(self):
    from opendbc.car.toyota.carstate import CarState

    state = CarState(self.cp)
    parser = state.get_can_parsers(self.cp)[Bus.pt]
    f = self.sensor()
    parser.update([(1_000_000_000, [f])])
    self.assertTrue(parser.pedal_valid)
    stamp = parser.ts_nanos["GAS_SENSOR"]["STATE"]
    a, d, b = self.sensor()
    parser.update([(1_020_000_000, [(a, d[:-1], b)])])
    self.assertFalse(parser.pedal_valid)
    self.assertEqual(parser.ts_nanos["GAS_SENSOR"]["STATE"], stamp)
    parser.update([(1_040_000_000, [self.sensor(bad_crc=True)])])
    self.assertFalse(parser.pedal_valid)

  def test_native_enabled_off_physical_gas_and_neutral(self):
    self.init()
    self.assertEqual(self.safety.aol_get_permission_mask(), 3)
    self.assertTrue(self.tx(0.5))
    self.assertTrue(self.tx(0))
    self.rx(self.sensor(gas=805))
    self.assertFalse(self.safety.get_gas_pressed_prev())
    self.rx(self.sensor(gas=806))
    self.assertTrue(self.safety.get_gas_pressed_prev())
    self.assertFalse(self.tx(0.1))
    self.assertTrue(self.tx(0))
    self.init(0x2249)
    self.assertEqual(self.safety.aol_get_permission_mask(), 1)
    self.assertFalse(self.tx(0.1))
    self.assertTrue(self.tx(0))

  def test_native_fault_crc_stale_reinit_and_unknown_words(self):
    for state in range(1, 16):
      self.init()
      self.assertFalse(self.rx(self.sensor(state=state)))
      self.assertFalse(self.tx(0.1))
      self.assertTrue(self.tx(0))
    self.init()
    self.assertFalse(self.rx(self.sensor(bad_crc=True)))
    self.assertFalse(self.tx(0.1))
    self.init()
    self.safety.set_timer(1_300_001)
    self.assertFalse(self.tx(0.1))
    self.assertTrue(self.tx(0))
    for word in (0x2149, 0x2849, 0x3048, 0xA048):
      self.init(word)
      self.assertFalse(self.tx(0), hex(word))
      self.assertFalse(self.tx(0.1))

  def test_native_command_pair_bounds_checksum_bus_and_neutral(self):
    self.init()
    a, d, b = pedal.command(self.packer, 0.1, 0)
    for change in ("pair", "crc", "disabled_nonzero", "max"):
      data = bytearray(d)
      if change == "pair":
        data[2:4] = (int.from_bytes(data[2:4], "big") + 8).to_bytes(2, "big")
      if change == "disabled_nonzero":
        data[4] &= 15
      if change == "max":
        data[:2] = (1500).to_bytes(2, "big")
      data[-1] = pedal.checksum(data) ^ int(change == "crc")
      self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(a, b, data)))
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(a, 2, d)))
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(a, b, d[:-1])))

  def test_counter_replay_fault_and_wrap(self):
    self.init()
    for _ in range(40):
      self.assertTrue(self.rx(self.sensor()))
    frame = self.sensor()
    self.rx(frame)
    for _ in range(6):
      self.rx(frame)
    self.assertFalse(self.tx(0.1))
    self.assertTrue(self.tx(0))

  def test_shared_source_reinit_does_not_carry_pedal_authority(self):
    for word in (0x2049, 73, 0x9049, 0x2249, 73, 0x2049):
      self.base.init(word, 32)
      self.assertFalse(self.safety.safety_config_valid())
      self.assertEqual(self.safety.aol_get_permission_mask(), 0)
      self.base.physical(cruise=True)
      if word in (0x2049, 0x2249):
        self.assertFalse(self.safety.safety_config_valid())
        self.assertFalse(self.tx(0.1))
        self.rx(self.sensor())
        self.assertTrue(self.safety.safety_config_valid())

  def test_exact_firmware_identity_and_unsupported_siblings(self):
    for ecu in (structs.CarParams.Ecu.eps, structs.CarParams.Ecu.fwdCamera):
      for field, value in (("fwVersion", b"unknown"), ("bus", 1), ("brand", ""), ("logging", True), ("address", 0x999)):
        fw = firmware(self.candidate)
        selected = next(item for item in fw if item.ecu == ecu)
        setattr(selected, field, value)
        cp = CarInterface.get_params(self.candidate, {0: {0x201: 6}, 1: {}, 2: {}}, fw, False, False, False)
        self.assertFalse(pedal.configured(cp), (ecu, field))
      missing = [item for item in firmware(self.candidate) if item.ecu != ecu]
      cp = CarInterface.get_params(self.candidate, {0: {0x201: 6}, 1: {}, 2: {}}, missing, False, False, False)
      self.assertFalse(pedal.configured(cp))
    for car in (CAR.TOYOTA_PRIUS, CAR.TOYOTA_RAV4H, CAR.TOYOTA_RAV4_TSS2_2022, CAR.TOYOTA_RAV4_TSS2_2023,
                CAR.TOYOTA_RAV4_PRIME, CAR.TOYOTA_CHR_TSS2, CAR.TOYOTA_SIENNA_4TH_GEN):
      cp = CarInterface.get_params(car, {0: {0x201: 6}, 1: {}, 2: {}}, [], True, False, False)
      self.assertFalse(pedal.configured(cp))

  def test_source_health_wrong_dlc_bus_and_parser_state(self):
    from opendbc.car.toyota.carstate import CarState

    state = CarState(self.cp)
    parsers = state.get_can_parsers(self.cp)
    parser = parsers[Bus.pt]
    f = self.sensor()
    parser.update([(1_000_000_000, [f])])
    state.update(parsers)
    self.assertTrue(state.pedal_ready)
    parser.update([(1_200_000_001, [])])
    state.update(parsers)
    self.assertFalse(state.pedal_ready)
    f = self.sensor(state=4)
    parser.update([(1_220_000_000, [f])])
    self.assertTrue(state.update(parsers).accFaulted)
    self.init()
    for step in range(1, 22):
      self.safety.set_timer(1_000_000 + step * 10_000)
      self.base.physical(cruise=True)
      a, d, b = self.sensor()
      self.rx((a, d, 1))
      self.rx((a, d[:-1], b))
    self.assertFalse(self.tx(0.1))
    self.assertTrue(self.tx(0))

  def test_main_source_lease_checksum_and_length(self):
    from opendbc.car.toyota.carstate import CarState
    from opendbc.car import CanData

    state = CarState(self.cp)
    parsers = state.get_can_parsers(self.cp)
    parser = parsers[Bus.pt]
    state.update(parsers)
    frame = self.packer.make_can_msg('PCM_CRUISE_2', 0, {'MAIN_ON': 1})
    parser.update([(1_000_000_000, [frame])])
    self.assertTrue(state.update(parsers).cruiseState.available)
    parser.update([(1_303_031_000, [])])
    self.assertTrue(state.update(parsers).cruiseState.available)
    parser.update([(1_303_031_001, [])])
    self.assertFalse(state.update(parsers).cruiseState.available)
    a, d, b = frame
    for bad in (CanData(a, d[:-1], b), CanData(a, d[:-1] + bytes([d[-1] ^ 1]), b)):
      parser.update([(1_400_000_000, [frame])])
      self.assertTrue(state.update(parsers).cruiseState.available)
      stamp = parser.ts_nanos['PCM_CRUISE_2']['MAIN_ON']
      parser.update([(1_410_000_000, [bad])])
      self.assertFalse(state.update(parsers).cruiseState.available)
      self.assertEqual(parser.ts_nanos['PCM_CRUISE_2']['MAIN_ON'], stamp)


class TestRav4CameraPairedPedal(TestPairedPedal):
  candidate = CAR.TOYOTA_RAV4_TSS2

  def test_existing_tunes_and_nonpedal_configuration_preserved(self):
    from opendbc.car.toyota.values import ToyotaFlags

    ordinary = CarInterface.get_params(self.candidate, {0: {}, 1: {}, 2: {}}, firmware(self.candidate), False, False, False)
    paired = self.cp.as_reader().as_builder()
    paired.flags &= ~int(ToyotaFlags.GAS_INTERCEPTOR)
    paired.safetyConfigs[0].safetyParam = ordinary.safetyConfigs[0].safetyParam
    # Dom pedal defaults are intentional; all other configuration stays identical.
    self.assertAlmostEqual(paired.longitudinalActuatorDelay, .2)
    self.assertAlmostEqual(paired.stopAccel, -1.5)
    self.assertAlmostEqual(ordinary.longitudinalActuatorDelay, .15)
    self.assertAlmostEqual(ordinary.stopAccel, -2.)
    paired.longitudinalActuatorDelay = ordinary.longitudinalActuatorDelay
    paired.stopAccel = ordinary.stopAccel
    self.assertEqual(paired.to_dict(), ordinary.to_dict())

  def test_known_firmware_is_candidate_specific(self):
    cp = CarInterface.get_params(self.candidate, {0: {0x201: 6}, 1: {}, 2: {}}, firmware(CAR.TOYOTA_COROLLA_TSS2), False, False, False)
    self.assertFalse(pedal.configured(cp))
