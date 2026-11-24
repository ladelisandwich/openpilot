"""Real factory enrollment and exact refusals for standalone radar/LTA pedals."""

import pytest
from opendbc.car import structs
from opendbc.car.toyota import pedal
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, ToyotaFlags
from openpilot.starpilot.car.toyota.tests.test_pedal_caller_native import firmware
from openpilot.starpilot.car.toyota.aol import qualified

CARS = (CAR.TOYOTA_CHR_TSS2, CAR.TOYOTA_RAV4_TSS2_2022, CAR.TOYOTA_RAV4_TSS2_2023)


def factory(candidate, alpha=True, fp=None, fw=None):
  return CarInterface.get_params(candidate, {0: {0x201: 6}, 1: {}, 2: {}} if fp is None else fp, firmware(candidate) if fw is None else fw, alpha, False, False)


@pytest.mark.parametrize('candidate', CARS)
@pytest.mark.parametrize('alpha', (False, True))
@pytest.mark.parametrize('stock', (False, True))
def test_real_factory_parent_tunes_stock_pair_and_alpha(candidate, alpha, stock):
  cp = factory(candidate, alpha)
  parent = factory(candidate, alpha, fp={0: {}, 1: {}, 2: {}})
  assert cp.steerActuatorDelay == parent.steerActuatorDelay and cp.steerLimitTimer == parent.steerLimitTimer
  assert cp.lateralTuning.to_dict() == parent.lateralTuning.to_dict()
  assert pedal.configured(cp) == alpha
  assert bool(cp.flags & ToyotaFlags.GAS_INTERCEPTOR) == alpha
  if alpha:
    assert cp.safetyConfigs[0].safetyParam == (0x2449 if candidate == CAR.TOYOTA_RAV4_TSS2_2023 else 0x2049)
    assert qualified(cp)
    if stock:
      assert pedal.prepare_stock(cp)
      assert pedal.configured(cp) and qualified(cp) and not cp.openpilotLongitudinalControl
      assert not cp.flags & ToyotaFlags.DISABLE_RADAR
      assert cp.safetyConfigs[0].safetyParam == (0x2649 if candidate == CAR.TOYOTA_RAV4_TSS2_2023 else 0x2249)


@pytest.mark.parametrize('candidate', CARS)
@pytest.mark.parametrize(
  'fp',
  ({0: {0x201: 5}}, {0: {}, 2: {0x201: 6}}, {0: {0x201: 6}, 1: {0x201: 6}}, {0: {0x201: 5, 0x2FF: 4}}, {0: {0x201: 5, 0x2FF: 8}}, {0: {0x201: 5, 0x2AA: 8}}),
)
def test_conflicting_hardware_is_not_enrolled(candidate, fp):
  assert not pedal.configured(factory(candidate, fp=fp))


@pytest.mark.parametrize('candidate', CARS)
@pytest.mark.parametrize('field,value', (('brand', ''), ('logging', True), ('bus', 2), ('address', 0x999), ('fwVersion', b'unknown')))
def test_wrong_firmware_metadata_is_not_enrolled(candidate, field, value):
  fw = firmware(candidate)
  item = next(f for f in fw if f.ecu == structs.CarParams.Ecu.eps)
  setattr(item, field, value)
  assert not pedal.configured(factory(candidate, fw=fw))
