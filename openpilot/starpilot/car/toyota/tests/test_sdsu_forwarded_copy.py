import pytest
from opendbc.car import gen_empty_fingerprint
from opendbc.car.toyota import eps8_sdsu, rav4h_filter
from opendbc.car.toyota.interface import CarInterface
from openpilot.starpilot.car.toyota.tests.test_pedal_caller_native import firmware

CARS = sorted(eps8_sdsu.CARS | rav4h_filter.EPS5_SDSU_CARS)


def detector(candidate):
  return eps8_sdsu.hardware_supported if candidate in eps8_sdsu.CARS else rav4h_filter.hardware_supported


@pytest.mark.parametrize('candidate', CARS)
@pytest.mark.parametrize('alpha', [False, True])
def test_forwarded_copy_preserves_full_factory_configuration(candidate, alpha):
  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = 8
  fw = firmware(candidate)
  original = CarInterface.get_params(candidate, fp, fw, alpha, False, False)
  fp[2][0x2FF] = 8
  mirrored = CarInterface.get_params(candidate, fp, fw, alpha, False, False)
  assert detector(candidate)(candidate, fp, fw)
  assert rav4h_filter.configured(mirrored)
  assert mirrored.to_dict() == original.to_dict()


@pytest.mark.parametrize('candidate', CARS)
@pytest.mark.parametrize('bad', ['mirror_only', 'bus1', 'bus3', 'mirror0', 'mirror4', 'mirror7', 'source7'])
def test_copy_cannot_enroll_wrong_source_or_length(candidate, bad):
  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = 8
  fp[2][0x2FF] = 8
  if bad == 'mirror_only':
    del fp[0][0x2FF]
  elif bad in ('bus1', 'bus3'):
    fp[int(bad[-1])] = {0x2FF: 8}
  elif bad.startswith('mirror'):
    fp[2][0x2FF] = int(bad[-1])
  else:
    fp[0][0x2FF] = 7
  assert not detector(candidate)(candidate, fp, firmware(candidate))
