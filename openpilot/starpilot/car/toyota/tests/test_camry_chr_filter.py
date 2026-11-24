import pytest
from opendbc.car import gen_empty_fingerprint
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, ToyotaFlags
from openpilot.starpilot.car.toyota.aol import radar_filter_qualified

@pytest.mark.parametrize('candidate', [CAR.TOYOTA_CAMRY, CAR.TOYOTA_CHR, CAR.TOYOTA_CHR_TSS2])
@pytest.mark.parametrize('alpha', [False, True])
def test_exact_filter_factory_and_existing_tunes(candidate, alpha):
  fp = gen_empty_fingerprint()
  plain = CarInterface.get_params(candidate, fp, [], alpha, False, False)
  fp[0][0x2AA] = 8
  cp = CarInterface.get_params(candidate, fp, [], alpha, False, False)
  expected = plain.to_dict()
  expected['flags'] = (expected['flags'] & ~int(ToyotaFlags.DISABLE_RADAR)) | int(ToyotaFlags.RADAR_CAN_FILTER)
  expected['alphaLongitudinalAvailable'] = True
  expected['openpilotLongitudinalControl'] = alpha
  expected['safetyConfigs'][0]['safetyParam'] = 0x1049 if alpha else 0x1249
  assert cp.to_dict() == expected
  assert radar_filter_qualified(cp)
