import pytest
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.toyota import legacy_advert_filter, advert_filter, rav4h_filter
from opendbc.car.toyota.interface import CarInterface, toyota_auto_hold_supported
from opendbc.car.toyota.values import CAR, ToyotaFlags
from openpilot.starpilot.car.toyota.tests.test_rav4h_caller_native import firmware


@pytest.mark.parametrize('candidate', sorted(legacy_advert_filter.PROFILES))
@pytest.mark.parametrize('stock', (False, True))
def test_complete_sdsu_cp_tunes_retained_for_advertisement(candidate, stock):
  fw = firmware(candidate)
  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = 8
  old = CarInterface.get_params(candidate, fp, fw, False, False, False)
  fp[0][0x2FF] = 4
  new = CarInterface.get_params(candidate, fp, fw, False, False, False)
  assert legacy_advert_filter.enabled(new) and advert_filter.enabled(new)
  assert toyota_auto_hold_supported(new) == toyota_auto_hold_supported(old)
  if stock:
    assert rav4h_filter.prepare_stock(old) and advert_filter.prepare_stock(new)
  assert legacy_advert_filter.configured(new)
  expected = old.to_dict()
  expected['flags'] = (expected['flags'] & ~int(ToyotaFlags.LONG_FILTER)) | int(ToyotaFlags.RADAR_CAN_FILTER)
  expected['safetyConfigs'][0]['safetyParam'] = legacy_advert_filter.PROFILES[candidate][stock]
  assert new.to_dict() == expected
  fp[2][0x2FF] = 4
  copied = CarInterface.get_params(candidate, fp, fw, False, False, False)
  if stock:
    assert advert_filter.prepare_stock(copied)
  assert copied.to_dict() == new.to_dict()


@pytest.mark.parametrize('candidate', sorted(legacy_advert_filter.PROFILES))
@pytest.mark.parametrize('bad', ('missing', 'source7', 'source8', 'copy7', 'copy8', 'bus1', 'pedal', 'mixed', 'firmware'))
def test_advertisement_identity_and_topology_fail_closed(candidate, bad):
  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = 4
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
    if candidate == CAR.TOYOTA_MATRIX_RETROFIT:
      # Manual selection admits empty acquisition, not invalid critical-ECU metadata.
      assert legacy_advert_filter.hardware_supported(candidate, fp, fw)
      fw = [structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.eps, address=0x7A1,
                                   fwVersion=b'', brand='toyota', bus=0)]
  assert not legacy_advert_filter.hardware_supported(candidate, fp, fw)


@pytest.mark.parametrize('candidate', sorted(legacy_advert_filter.PROFILES))
@pytest.mark.parametrize('stock', (False, True))
def test_wheel_button_scope_matches_existing_sdsu_owner(candidate, stock):
  from openpilot.starpilot.controllers.wheel_actions import button_reported, KEYS
  from openpilot.starpilot.car.toyota.aol import policy_for
  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = 8
  old = CarInterface.get_params(candidate, fp, firmware(candidate), False, False, False)
  fp[0][0x2FF] = 4
  new = CarInterface.get_params(candidate, fp, firmware(candidate), False, False, False)
  if stock:
    assert rav4h_filter.prepare_stock(old) and advert_filter.prepare_stock(new)
  assert [button_reported(old, key) for key in KEYS[:8]] == [button_reported(new, key) for key in KEYS[:8]]
  assert policy_for(new).fixed_cruise_buttons and policy_for(new).auxiliary_lkas_supported
  assert not button_reported(new, 'MainCruiseButtonControl')
  assert not any(button_reported(new, key) for key in KEYS[5:8])
