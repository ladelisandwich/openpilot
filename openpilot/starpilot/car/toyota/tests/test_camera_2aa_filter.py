"""Factory-owned camera filters preserve full existing CP/tune fields."""

import json
from copy import deepcopy
from pathlib import Path
import pytest
from opendbc.car import gen_empty_fingerprint
from opendbc.car.toyota import camera_2aa_filter, advert_filter
from opendbc.car.toyota.interface import CarInterface
from openpilot.starpilot.car.toyota.tests.test_rav4h_caller_native import firmware

BASE = json.loads((Path(__file__).parent / 'data/camera-2aa-factory-before.json').read_text())


def factory(candidate, *, mirror=False):
  fp = gen_empty_fingerprint()
  fp[0][0x2AA] = 8
  if mirror:
    fp[2][0x2AA] = 8
  return CarInterface.get_params(candidate, fp, firmware(candidate), False, False, False)


@pytest.mark.parametrize('candidate', sorted(camera_2aa_filter.CARS))
def test_complete_factory_cp_tunes_and_stock(candidate):
  cp = factory(candidate)
  assert camera_2aa_filter.configured(cp) and advert_filter.configured(cp)
  data = cp.to_dict()
  data['safetyConfigs'][0]['safetyParam'] = advert_filter.LONG_WORD
  assert data == BASE[candidate.name]['2FF4']
  original = cp.to_dict()
  assert advert_filter.prepare_stock(cp) and camera_2aa_filter.configured(cp)
  expected = deepcopy(original)
  expected['openpilotLongitudinalControl'] = False
  expected['safetyConfigs'][0]['safetyParam'] = camera_2aa_filter.STOCK_WORD
  expected['autoResumeSng'] = False
  assert cp.to_dict() == expected
  assert factory(candidate, mirror=True).to_dict() == original


@pytest.mark.parametrize('candidate', sorted(camera_2aa_filter.CARS))
@pytest.mark.parametrize('size', (4, 8))
def test_previous_filter_factory_cp_exact(candidate, size):
  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = size
  assert CarInterface.get_params(candidate, fp, firmware(candidate), False, False, False).to_dict() == BASE[candidate.name]['2FF' + str(size)]


@pytest.mark.parametrize('candidate', sorted(camera_2aa_filter.CARS))
@pytest.mark.parametrize(
  'bad', ('missing', 'copy_only', 'primary4', 'primary7', 'copy7', 'bus1', 'pedal', 'status8', 'advert4', 'camera343', 'camera4cb', 'firmware')
)
def test_twoaa_identity_topology_rejects_unqualified_owner(candidate, bad):
  fp = gen_empty_fingerprint()
  fp[0][0x2AA] = 8
  fw = firmware(candidate)
  if bad == 'missing':
    del fp[0][0x2AA]
  elif bad == 'copy_only':
    del fp[0][0x2AA]
    fp[2][0x2AA] = 8
  elif bad.startswith('primary'):
    fp[0][0x2AA] = int(bad[-1])
  elif bad == 'copy7':
    fp[2][0x2AA] = 7
  elif bad == 'bus1':
    fp[1][0x2AA] = 8
  elif bad == 'pedal':
    fp[0][0x201] = 5
  elif bad in ('status8', 'advert4'):
    fp[0][0x2FF] = 8 if bad == 'status8' else 4
  elif bad.startswith('camera'):
    fp[2][0x343 if bad == 'camera343' else 0x4CB] = 7
  else:
    fw = []
  assert not camera_2aa_filter.hardware_supported(candidate, fp, fw)
  assert not camera_2aa_filter.configured(CarInterface.get_params(candidate, fp, fw, False, False, False))


@pytest.mark.parametrize('candidate', sorted(camera_2aa_filter.CARS))
@pytest.mark.parametrize('stock', (False, True))
def test_twoaa_wheel_scope_uses_only_actual_lkas_receiver(candidate, stock):
  from openpilot.starpilot.controllers.wheel_actions import button_reported, KEYS
  from openpilot.starpilot.car.toyota.aol import policy_for

  cp = factory(candidate)
  if stock:
    assert camera_2aa_filter.prepare_stock(cp)
  assert button_reported(cp, 'LKASButtonControl')
  assert not any(button_reported(cp, key) for key in KEYS[2:8])
  assert not button_reported(cp, 'MainCruiseButtonControl')
  assert policy_for(cp).fixed_cruise_buttons and policy_for(cp).auxiliary_lkas_supported
