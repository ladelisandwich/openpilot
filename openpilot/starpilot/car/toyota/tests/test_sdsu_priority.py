import pytest
from opendbc.car import gen_empty_fingerprint
from opendbc.car.toyota import eps8_sdsu, rav4h_filter, legacy_sdsu
from opendbc.car.toyota.interface import CarInterface
from openpilot.starpilot.car.toyota.tests.test_rav4h_caller_native import firmware

CARS = sorted(eps8_sdsu.CARS | rav4h_filter.EPS5_SDSU_CARS | legacy_sdsu.PROFILES.keys())


def detector(candidate):
  if candidate in legacy_sdsu.PROFILES:
    return legacy_sdsu.hardware_supported
  return eps8_sdsu.hardware_supported if candidate in eps8_sdsu.CARS else rav4h_filter.hardware_supported


@pytest.mark.parametrize('candidate', CARS)
@pytest.mark.parametrize('alpha', [False, True])
@pytest.mark.parametrize('addresses', [(0x343,), (0x4CB,), (0x343, 0x4CB)])
def test_smartdsu_priority_preserves_full_configuration(candidate, alpha, addresses):
  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = 8
  fw = firmware(candidate)
  baseline = CarInterface.get_params(candidate, fp, fw, alpha, False, False)
  fp[2].update(dict.fromkeys(addresses, 8))
  actual = CarInterface.get_params(candidate, fp, fw, alpha, False, False)
  assert detector(candidate)(candidate, fp, fw)
  assert actual.to_dict() == baseline.to_dict()
  assert rav4h_filter.prepare_stock(actual) and rav4h_filter.prepare_stock(baseline)
  assert actual.to_dict() == baseline.to_dict()


@pytest.mark.parametrize('candidate', CARS)
@pytest.mark.parametrize('address', [0x343, 0x4CB])
def test_camera_traffic_cannot_replace_bus0_status(candidate, address):
  fp = gen_empty_fingerprint()
  fp[2].update({address: 8, 0x2FF: 8})
  assert not detector(candidate)(candidate, fp, firmware(candidate))


@pytest.mark.parametrize('candidate', sorted(rav4h_filter.EPS5_SDSU_CARS | legacy_sdsu.PROFILES.keys()))
@pytest.mark.parametrize('address', [0x343, 0x4CB])
def test_malformed_camera_copy_remains_rejected(candidate, address):
  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = 8
  fp[2][address] = 7
  assert not detector(candidate)(candidate, fp, firmware(candidate))
