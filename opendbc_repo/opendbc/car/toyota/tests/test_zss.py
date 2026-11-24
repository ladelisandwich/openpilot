"""Prius secondary-angle selection through the real CAN parser and controller."""
import unittest
from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, DBC, ToyotaFlags
from opendbc.car.toyota.prius_longitudinal import configured


def factory(car=CAR.TOYOTA_PRIUS, bus=0, length=8):
  fp = {0: {}, 1: {}, 2: {}}
  fp[bus][0x23] = length
  return CarInterface.get_params(car, fp, [], False, False, False)


class TestZssCaller(unittest.TestCase):
  def test_factory_identity_and_exact_presence(self):
    for car in (CAR.TOYOTA_PRIUS, CAR.TOYOTA_PRIUS_RETROFIT):
      self.assertTrue(factory(car).flags & (1 << 16), "Detected Prius ZSS has no selected sensor owner")
      for bus, length in ((1, 8), (2, 8), (0, 7)):
        self.assertFalse(factory(car, bus, length).flags & ToyotaFlags.ZSS)
    for car in (CAR.TOYOTA_AVALON, CAR.TOYOTA_COROLLA_TSS2, CAR.TOYOTA_RAV4_TSS2):
      self.assertFalse(factory(car).flags & ToyotaFlags.ZSS)

  def test_real_interface_parser_apply_alignment_and_stale_fallback(self):
    cp = factory()
    ci = CarInterface(cp)
    packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
    command = structs.CarControl()

    def update(tick, zorro):
      frames = []
      if zorro is not None:
        frames.append(packer.make_can_msg('SECONDARY_STEER_ANGLE', 0, {'ZORRO_STEER': zorro}))
      frames += [packer.make_can_msg('STEER_TORQUE_SENSOR', 0, {'STEER_ANGLE': 10.}),
                 packer.make_can_msg('STEER_ANGLE_SENSOR', 0, {'STEER_ANGLE': 10.})]
      for name in ('VSC1S07', 'BODY_CONTROL_STATE', 'BODY_CONTROL_STATE_2', 'BRAKE_MODULE', 'ESP_CONTROL',
                   'GEAR_PACKET', 'PCM_CRUISE_SM', 'BLINKERS_STATE', 'PRE_COLLISION', 'WHEEL_SPEEDS', 'LIGHT_STALK'):
        frames.append(packer.make_can_msg(name, 0, {}))
      frames.extend([packer.make_can_msg('EPS_STATUS', 0, {'LKA_STATE': 1}),
                     packer.make_can_msg('PCM_CRUISE', 0, {'GAS_RELEASED': 1, 'CRUISE_ACTIVE': 1}),
                     packer.make_can_msg('PCM_CRUISE_2', 0, {'MAIN_ON': 1}),
                     packer.make_can_msg('LKAS_HUD', 2, {}), packer.make_can_msg('RSA1', 2, {}),
                     packer.make_can_msg('ACC_CONTROL', 0, {})])
      return ci.update([(2_000_000_000 + tick * 10_000_000, frames)])
    for tick in range(-100, 1):
      state = update(tick, 20.)
    self.assertTrue(state.canValid)
    stock_angle = state.steeringAngleDeg
    command.latActive = True
    ci.apply(command.as_reader(), 2_000_000_001)
    self.assertTrue(ci.CS.zss_lateral_active)
    update(1, 20.)
    self.assertFalse(ci.CS.zss_needs_offset)
    aligned = update(2, 20.5)
    self.assertAlmostEqual(aligned.steeringAngleDeg, stock_angle + .5, places=1)
    stale = update(3, None)
    self.assertAlmostEqual(stale.steeringAngleDeg, stock_angle, places=1)
    jumped = update(4, 25.)
    self.assertAlmostEqual(jumped.steeringAngleDeg, stock_angle, places=1)
    command.latActive = False
    ci.apply(command.as_reader(), 2_050_000_000)
    update(5, 25.)
    command.latActive = True
    ci.apply(command.as_reader(), 2_060_000_000)
    update(6, 25.)
    self.assertAlmostEqual(update(7, 25.5).steeringAngleDeg, stock_angle + .5, places=1)

  def test_filter_ownership_accepts_sensor_without_changing_native_word(self):
    from opendbc.car.toyota.prius_longitudinal import prepare_stock
    rack = structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.eps,
                                  fwVersion=b'8965B47070' + bytes(6))
    cp = CarInterface.get_params(CAR.TOYOTA_PRIUS, {0: {0x23: 8, 0x2FF: 8}, 1: {}, 2: {}}, [rack], False, False, False)
    self.assertTrue(configured(cp))
    self.assertEqual(cp.safetyConfigs[0].safetyParam, 0x8049)
    sibling = cp.as_reader().as_builder()
    sibling.flags |= 1 << 17
    self.assertFalse(configured(sibling))
    self.assertTrue(prepare_stock(cp))
    self.assertEqual(cp.safetyConfigs[0].safetyParam, 0x8249)
    self.assertTrue(cp.flags & ToyotaFlags.ZSS)
