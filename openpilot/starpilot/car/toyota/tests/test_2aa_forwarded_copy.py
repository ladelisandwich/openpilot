import pytest
from opendbc.car import gen_empty_fingerprint
from opendbc.car.toyota import radar_filter
from opendbc.car.toyota.interface import CarInterface
from openpilot.starpilot.car.toyota.tests.test_pedal_caller_native import firmware


@pytest.mark.parametrize('candidate', sorted(radar_filter.PROFILES))
@pytest.mark.parametrize('alpha', [False, True])
def test_same_length_advertisement_copy_preserves_factory(candidate, alpha):
  fp = gen_empty_fingerprint()
  fp[0][0x2AA] = 8
  fw = firmware(candidate)
  before = CarInterface.get_params(candidate, fp, fw, alpha, False, False)
  fp[2][0x2AA] = 8
  after = CarInterface.get_params(candidate, fp, fw, alpha, False, False)
  assert radar_filter.hardware_supported(candidate, fp)
  assert radar_filter.configured(after)
  assert after.to_dict() == before.to_dict()


@pytest.mark.parametrize('candidate', sorted(radar_filter.PROFILES))
@pytest.mark.parametrize('bad', ['only_copy', 'source7', 'copy4', 'copy7', 'bus1', 'echo', 'pedal', 'mixed'])
def test_unsupported_advertisement_copies_do_not_enroll(candidate, bad):
  fp = gen_empty_fingerprint()
  fp[0][0x2AA] = fp[2][0x2AA] = 8
  if bad == 'only_copy':
    del fp[0][0x2AA]
  elif bad == 'source7':
    fp[0][0x2AA] = 7
  elif bad.startswith('copy'):
    fp[2][0x2AA] = int(bad[-1])
  elif bad in ('bus1', 'echo'):
    fp[1 if bad == 'bus1' else 128] = {0x2AA: 8}
  else:
    fp[0][0x201 if bad == 'pedal' else 0x2FF] = 6 if bad == 'pedal' else 4
  assert not radar_filter.hardware_supported(candidate, fp)
