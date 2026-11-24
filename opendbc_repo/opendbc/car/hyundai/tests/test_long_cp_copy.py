"""Direct configuration copies retain admission and independent nested state."""
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from opendbc.car import structs
from opendbc.car.hyundai import ev9_longitudinal, ioniq5pe_long_profile
from opendbc.car.hyundai.tests.test_ioniq5pe_stock import params
from opendbc.car.hyundai.values import CAR


def legacy_copy(cp):
  with structs.CarParams.from_bytes(structs.CarParams.new_message(**cp.to_dict()).to_bytes()) as reader:
    return reader.as_builder()


@pytest.mark.parametrize('car,profile', [(CAR.KIA_EV9, ev9_longitudinal), (CAR.HYUNDAI_IONIQ_5_PE, ioniq5pe_long_profile)])
@pytest.mark.parametrize('experience', [0, 32])
def test_independent_copy_and_live_admission(car, profile, experience):
  cp = profile.candidate(params(candidate=car), enabled=True, is_release=False)
  assert cp is not None
  cp.alternativeExperience = experience
  original = cp.to_dict()
  clone = ev9_longitudinal.copy_cp(cp)
  assert clone.to_dict() == legacy_copy(cp).to_dict() == original
  clone.safetyConfigs[0].safetyParam = 0
  clone.deprecated.startAccel = -9
  clone.wheelbase += 1
  assert cp.to_dict() == original
  assert profile.qualified(cp)
  for field, value in [('brand', 'wrong'), ('passive', True), ('dashcamOnly', True), ('notCar', True),
                       ('alternativeExperience', 1), ('flags', 0), ('pcmCruise', True),
                       ('openpilotLongitudinalControl', False), ('alphaLongitudinalAvailable', False)]:
    changed = legacy_copy(cp)
    setattr(changed, field, value)
    actual = profile.qualified(changed)
    with patch.object(profile, 'copy_cp', legacy_copy):
      assert profile.qualified(changed) == actual
    assert not actual, field
  assert cp.to_dict() == original


@pytest.mark.parametrize('car', [CAR.KIA_EV9, CAR.HYUNDAI_IONIQ_5_PE])
def test_reader_copy_outlives_reader_and_retains_nested_fields(car):
  cp = params(candidate=car)
  original = cp.to_dict()
  with structs.CarParams.from_bytes(cp.to_bytes()) as reader:
    clone = ev9_longitudinal.copy_cp(reader)
    assert clone.to_dict() == legacy_copy(reader).to_dict() == original
    clone.safetyConfigs[0].safetyParam = 0
    assert reader.to_dict() == original
  assert clone.safetyConfigs[0].safetyParam == 0
  assert clone.carFingerprint == car


def test_unknown_input_retains_dictionary_fallback():
  cp = params(candidate=CAR.KIA_EV9)
  value = cp.to_dict()
  custom = SimpleNamespace(to_dict=lambda: value)
  assert ev9_longitudinal.copy_cp(custom).to_dict() == legacy_copy(custom).to_dict()
