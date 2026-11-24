import unittest
from opendbc.car import Bus, structs, CanData
from opendbc.can import CANPacker
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, DBC
from openpilot.starpilot.car.toyota.aol import qualified, policy_for


class TestLexusIsFactory(unittest.TestCase):
  def test_exact_factory_stock_owner_and_sibling_boundary(self):
    for alpha in (False, True):
      cp = CarInterface.get_params(CAR.LEXUS_IS, {0: {}, 1: {}, 2: {}}, [], alpha, False, False)
      self.assertEqual(cp.safetyConfigs[0].safetyParam, 0x424D)
      self.assertFalse(cp.openpilotLongitudinalControl)
      self.assertTrue(cp.pcmCruise)
      self.assertTrue(qualified(cp))
      self.assertEqual(policy_for(cp).alternative_experience_addition, 32)
      cp.alternativeExperience = 32
      self.assertTrue(qualified(cp, marked_only=True))
      for mutation in ('carFingerprint', 'safetyParam', 'long', 'angle', 'passive'):
        bad = cp.as_reader().as_builder()
        if mutation == 'carFingerprint':
          bad.carFingerprint = CAR.LEXUS_RC
        elif mutation == 'safetyParam':
          bad.safetyConfigs[0].safetyParam = 0x404D
        elif mutation == 'long':
          bad.openpilotLongitudinalControl = True
        elif mutation == 'angle':
          bad.steerControlType = structs.CarParams.SteerControlType.angle
        else:
          bad.passive = True
        self.assertFalse(qualified(bad), mutation)
    rc = CarInterface.get_params(CAR.LEXUS_RC, {0: {}, 1: {}, 2: {}}, [], False, False, False)
    self.assertEqual(rc.safetyConfigs[0].safetyParam, 0x564D)
    self.assertTrue(qualified(rc))

  def test_real_parser_dsu_main_and_pcm_cruise(self):
    cp = CarInterface.get_params(CAR.LEXUS_IS, {0: {}, 1: {}, 2: {}}, [], False, False, False)
    ci = CarInterface(cp)
    packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
    ci.update([])  # Register signals used by the real CarState before the first CAN sample.
    for tick in range(50):
      frames = [packer.make_can_msg('DSU_CRUISE', 0, {'MAIN_ON': tick < 25, 'SET_SPEED': 80}),
                packer.make_can_msg('PCM_CRUISE', 0, {'GAS_RELEASED': 1, 'CRUISE_ACTIVE': 1}),
                packer.make_can_msg('PCM_CRUISE_2', 0, {'MAIN_ON': tick >= 25})]
      state = ci.update([(1_000_000_000 + tick * 100_000_000, frames)])
      self.assertEqual(state.cruiseState.available, tick < 25)
      self.assertTrue(state.cruiseState.enabled)
      self.assertAlmostEqual(state.cruiseState.speed, 80/3.6, places=5)

  def test_dsu_main_lease_and_malformed_frames_cannot_refresh_it(self):
    cp = CarInterface.get_params(CAR.LEXUS_IS, {0: {}, 1: {}, 2: {}}, [], False, False, False)
    ci = CarInterface(cp)
    packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
    ci.update([])
    frame = packer.make_can_msg('DSU_CRUISE', 0, {'MAIN_ON': 1})
    self.assertTrue(ci.update([(1_000_000_000, [frame])]).cruiseState.available)
    self.assertTrue(ci.update([(1_303_031_000, [])]).cruiseState.available)
    self.assertFalse(ci.update([(1_303_031_001, [])]).cruiseState.available)
    self.assertTrue(ci.update([(1_400_000_000, [frame])]).cruiseState.available)
    a, d, b = frame
    self.assertFalse(ci.update([(1_410_000_000, [CanData(a, d + bytes(1), b)])]).cruiseState.available)
    self.assertTrue(ci.update([(1_420_000_000, [frame])]).cruiseState.available)
