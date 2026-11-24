import pytest
from opendbc.car import gen_empty_fingerprint
from opendbc.car.toyota import advert_filter
from opendbc.car.toyota.interface import CarInterface
from openpilot.starpilot.car.toyota.tests.test_rav4h_caller_native import firmware


@pytest.mark.parametrize('candidate', sorted(advert_filter.CARS))
@pytest.mark.parametrize('bad', ['missing', 'source8', 'source7', 'copy8', 'copy7', 'bus1', 'pedal', 'mixed', 'firmware'])
def test_hardware_identity_boundaries(candidate, bad):
  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = fp[2][0x2FF] = 4
  fw = firmware(candidate)
  if bad == 'missing':
    del fp[0][0x2FF]
  elif bad.startswith('source'):
    fp[0][0x2FF] = int(bad[-1])
  elif bad.startswith('copy'):
    fp[2][0x2FF] = int(bad[-1])
  elif bad == 'bus1':
    fp[1][0x2FF] = 4
  elif bad == 'pedal':
    fp[0][0x201] = 6
  elif bad == 'mixed':
    fp[0][0x2AA] = 8
  else:
    fw = []
  assert not advert_filter.hardware_supported(candidate, fp, fw)


@pytest.mark.parametrize('candidate', sorted(advert_filter.CARS))
@pytest.mark.parametrize('field,value', [('fwVersion', b'unknown'), ('bus', 1), ('brand', 'ford'), ('logging', True), ('address', 0x123), ('subAddress', 1)])
def test_unknown_observed_eps_cannot_enroll(candidate, field, value):
  from opendbc.car import structs

  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = 4
  fw = firmware(candidate)
  eps = next(f for f in fw if f.ecu == structs.CarParams.Ecu.eps)
  setattr(eps, field, value)
  assert not advert_filter.hardware_supported(candidate, fp, fw)


@pytest.mark.parametrize('candidate', sorted(advert_filter.CARS))
@pytest.mark.parametrize('alpha', [False, True])
def test_forwarded_advertisement_and_saved_stock_are_exact(candidate, alpha):
  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = 4
  fw = firmware(candidate)
  original = CarInterface.get_params(candidate, fp, fw, alpha, False, False)
  fp[2][0x2FF] = 4
  copied = CarInterface.get_params(candidate, fp, fw, alpha, False, False)
  assert original.to_dict() == copied.to_dict()
  assert advert_filter.prepare_stock(original) and advert_filter.prepare_stock(copied)
  assert original.to_dict() == copied.to_dict()


def test_saved_setting_and_clock_withdraw_without_changing_frozen_cp():
  from pathlib import Path
  from openpilot.common.params import Params
  from openpilot.common.prefix import OpenpilotPrefix
  from openpilot.starpilot.car.toyota.advert_preferences import AdvertFilterPreference

  candidate = next(iter(advert_filter.CARS))
  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = 4
  cp = CarInterface.get_params(candidate, fp, firmware(candidate), False, False, False)
  original = cp.to_dict()
  with OpenpilotPrefix():
    params = Params()
    for key, value in [('OpenpilotEnabledToggle', b'1'), ('SafeMode', b'0'), ('DisableOpenpilotLongitudinal', b'0')]:
      params.put_bool(key, value == b'1', block=True)
    preference = AdvertFilterPreference(cp, params)
    assert preference.update(1_000_000_000)
    Path(params.get_param_path('DisableOpenpilotLongitudinal')).write_bytes(b'invalid')
    assert not preference.update(1_250_000_000)
    params.put_bool('DisableOpenpilotLongitudinal', False, block=True)
    assert preference.update(1_500_000_000)
    assert not preference.update(1_499_999_999)
    params.put_bool('SafeMode', True, block=True)
    assert not preference.update(2_000_000_000)
    assert cp.to_dict() == original
