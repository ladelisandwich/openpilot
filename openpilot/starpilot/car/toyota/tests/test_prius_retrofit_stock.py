import pytest
import struct
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, ToyotaFlags
from openpilot.starpilot.car.toyota.aol import policy_for, qualified, stock_prius_retrofit_qualified


def factory(*, alpha=False, zss=False):
  fp = gen_empty_fingerprint()
  if zss:
    fp[0][0x23] = 8
  fw = [structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.eps, address=0x7A1,
                              fwVersion=b'8965B47070' + bytes(6), brand='toyota', bus=0)]
  return CarInterface.get_params(CAR.TOYOTA_PRIUS_RETROFIT, fp, fw, alpha, False, False)


@pytest.mark.parametrize('alpha', (False, True))
@pytest.mark.parametrize('zss', (False, True))
def test_manual_stock_prius_retrofit_factory_and_policy(alpha, zss):
  cp = factory(alpha=alpha, zss=zss)
  before = cp.to_dict()
  assert not cp.openpilotLongitudinalControl and cp.pcmCruise and not cp.dashcamOnly
  assert cp.safetyConfigs[0].safetyParam == 585
  assert cp.flags == int(ToyotaFlags.HYBRID) | (int(ToyotaFlags.ZSS) if zss else 0)
  assert cp.steerActuatorDelay == struct.unpack('<f', struct.pack('<f', .14))[0]
  assert cp.lateralTuning.torque.steeringAngleDeadzoneDeg == struct.unpack('<f', struct.pack('<f', .3))[0]
  assert stock_prius_retrofit_qualified(cp) and qualified(cp)
  assert policy_for(cp).runtime_supported
  assert cp.to_dict() == before, 'Admission must not change any existing tune or configuration'
  assert not stock_prius_retrofit_qualified(cp, marked_only=True)
  cp.alternativeExperience = 32
  assert stock_prius_retrofit_qualified(cp, marked_only=True) and qualified(cp, marked_only=True)


@pytest.mark.parametrize('field,value', (
  ('notCar', True), ('passive', True), ('dashcamOnly', True), ('secOcRequired', True),
  ('pcmCruise', False), ('openpilotLongitudinalControl', True), ('flags', 0),
  ('flags', int(ToyotaFlags.HYBRID | ToyotaFlags.LONG_FILTER)),
  ('flags', int(ToyotaFlags.HYBRID | ToyotaFlags.GAS_INTERCEPTOR)),
  ('flags', int(ToyotaFlags.HYBRID | ToyotaFlags.AUTO_BRAKE_HOLD)),
  ('steerControlType', structs.CarParams.SteerControlType.angle),
  ('carFingerprint', CAR.TOYOTA_PRIUS), ('carFingerprint', CAR.TOYOTA_MATRIX_RETROFIT),
  ('brand', 'gm'), ('alternativeExperience', 128), ('alternativeExperience', 160),
))
def test_stock_prius_retrofit_admission_refuses_foreign_configuration(field, value):
  cp = factory()
  setattr(cp, field, value)
  assert not stock_prius_retrofit_qualified(cp)


@pytest.mark.parametrize('word', (73, 584, 586, 4169, 4681, 0x8042, 0x8242, 0xC042, 0xC242))
def test_stock_prius_retrofit_rejects_other_native_owners(word):
  cp = factory()
  cp.safetyConfigs[0].safetyParam = word
  assert not stock_prius_retrofit_qualified(cp)


def test_stock_prius_retrofit_refuses_invalid_native_inventory():
  cp = factory()
  cp.safetyConfigs = []
  assert not stock_prius_retrofit_qualified(cp)
  cp = factory()
  cp.safetyConfigs = [cp.safetyConfigs[0], cp.safetyConfigs[0]]
  assert not stock_prius_retrofit_qualified(cp)
  cp = factory()
  cp.safetyConfigs[0].safetyModel = structs.CarParams.SafetyModel.gm
  assert not stock_prius_retrofit_qualified(cp)
  assert not stock_prius_retrofit_qualified(None)
