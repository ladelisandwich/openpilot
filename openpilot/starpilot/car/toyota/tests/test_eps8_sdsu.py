import pytest
from opendbc.car import gen_empty_fingerprint
from opendbc.car.toyota import eps8_sdsu
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, ToyotaFlags
from openpilot.starpilot.car.toyota.tests.test_pedal_caller_native import firmware
from openpilot.starpilot.car.toyota.aol import qualified


@pytest.mark.parametrize('candidate', sorted(eps8_sdsu.CARS))
@pytest.mark.parametrize('alpha', [False, True])
def test_eps8_sdsu_factory_and_saved_off(candidate, alpha):
  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = 8
  cp = CarInterface.get_params(candidate, fp, firmware(candidate), alpha, False, False)
  assert eps8_sdsu.configured(cp) and qualified(cp)
  long = (
    alpha
    or not candidate.config.flags & (ToyotaFlags.NO_DSU | ToyotaFlags.RADAR_ACC)
    or bool(candidate.config.flags & ToyotaFlags.TSS2 and not candidate.config.flags & ToyotaFlags.RADAR_ACC)
  )
  assert cp.openpilotLongitudinalControl == long
  assert not cp.flags & ToyotaFlags.DISABLE_RADAR
  assert cp.safetyConfigs[0].safetyParam == eps8_sdsu.words(candidate)[not long]
  assert eps8_sdsu.prepare_stock(cp)
  assert eps8_sdsu.configured(cp) and qualified(cp) and not cp.openpilotLongitudinalControl


@pytest.mark.parametrize('candidate', sorted(eps8_sdsu.CARS))
@pytest.mark.parametrize('bad', ['length', 'mirror', 'pedal', 'mixed', 'firmware'])
def test_eps8_hardware_boundaries(candidate, bad):
  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = 8
  fw = firmware(candidate)
  if bad == 'length':
    fp[0][0x2FF] = 4
  if bad == 'mirror':
    fp[2][0x2FF] = 7
  if bad == 'pedal':
    fp[0][0x201] = 6
  if bad == 'mixed':
    fp[0][0x2AA] = 8
  if bad == 'firmware':
    fw = []
  assert not eps8_sdsu.hardware_supported(candidate, fp, fw)


@pytest.mark.parametrize('candidate', [CAR.TOYOTA_RAV4_PRIME, CAR.TOYOTA_SIENNA_4TH_GEN, CAR.TOYOTA_RAV4H, CAR.TOYOTA_PRIUS])
def test_other_native_topologies_not_claimed(candidate):
  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = 8
  assert not eps8_sdsu.hardware_supported(candidate, fp, firmware(candidate))


@pytest.mark.parametrize("candidate", sorted(eps8_sdsu.CARS))
@pytest.mark.parametrize("alpha", [False, True])
@pytest.mark.parametrize("hybrid", [False, True])
def test_sdsu_keeps_existing_vehicle_configuration_and_tune(candidate, alpha, hybrid):
  from opendbc.car import structs
  from opendbc.car.toyota.values import CarControllerParams
  from opendbc.car.toyota.radar_filter import PROFILES

  fw = firmware(candidate)
  if hybrid:
    fw.append(
      structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.hybrid, address=0x7E2, fwVersion=b"synthetic hybrid ECU response", brand="toyota", bus=0, logging=True)
    )
  existing = gen_empty_fingerprint()
  if candidate in PROFILES:
    existing[0][0x2AA] = 8
  new = gen_empty_fingerprint()
  new[0][0x2FF] = 8
  reference = CarInterface.get_params(candidate, existing, fw, alpha, False, False)
  actual = CarInterface.get_params(candidate, new, fw, alpha, False, False)
  before, after = reference.to_dict(), actual.to_dict()
  for field in ("flags", "safetyConfigs", "openpilotLongitudinalControl", "alphaLongitudinalAvailable", "autoResumeSng"):
    before.pop(field, None)
    after.pop(field, None)
  assert before == after
  baseline_limits, new_limits = CarControllerParams(reference), CarControllerParams(actual)
  assert (baseline_limits.ACCEL_MIN, baseline_limits.ACCEL_MAX) == (new_limits.ACCEL_MIN, new_limits.ACCEL_MAX)
