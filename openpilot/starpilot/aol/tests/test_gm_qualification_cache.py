"""Static GM qualification reuses exact immutable CP values, never live authority."""
from types import SimpleNamespace
from collections.abc import Callable
from typing import NoReturn, cast
from unittest.mock import patch
import warnings

import pytest
from opendbc.car.gm import aol
from opendbc.car.gm.tests.test_volt_sdgm_control import sdgm_params
from openpilot.starpilot.aol.tests.test_gm import _configurations


@pytest.fixture(autouse=True)
def clear_cache():
  aol._qualified_gm_bytes.cache_clear()
  yield
  aol._qualified_gm_bytes.cache_clear()


def test_all_existing_factory_configurations_match_uncached():
  count = 0
  for cp in _configurations():
    expected = aol._qualified_gm_uncached(cp)
    assert aol.qualified_gm(cp) == expected
    assert aol.qualified_gm(cp.as_reader()) == expected
    count += 1
  assert count > 100
  assert aol._qualified_gm_bytes.cache_info().currsize <= 8


@pytest.mark.parametrize('field,value', [
  ('brand', 'hyundai'), ('carFingerprint', 'unknown'), ('flags', 1), ('passive', True),
  ('dashcamOnly', True), ('notCar', True), ('pcmCruise', True), ('openpilotLongitudinalControl', False),
  ('alternativeExperience', 1), ('alphaLongitudinalAvailable', False), ('networkLocation', 'gateway'),
  ('transmissionType', 'manual'), ('steerControlType', 'angle'), ('carVin', 'changed'),
])
def test_mutable_builder_and_reader_alias_requalify(field, value):
  cp = sdgm_params(alpha=True)
  alias = cp.as_reader()
  assert aol.qualified_gm(cp)
  setattr(cp, field, value)
  expected = aol._qualified_gm_uncached(cp)
  assert aol.qualified_gm(cp) == aol.qualified_gm(alias) == expected


@pytest.mark.parametrize('value', [0., float('nan'), float('inf'), 2.5])
def test_nested_tuning_mutations_requalify(value):
  cp = sdgm_params(alpha=True)
  assert aol.qualified_gm(cp)
  cp.lateralTuning.torque.latAccelFactor = value
  assert aol.qualified_gm(cp) == aol._qualified_gm_uncached(cp)


@pytest.mark.parametrize('field,value', [('safetyParam', 0), ('safetyModel', 'noOutput')])
def test_nested_native_owner_mutations_requalify(field, value):
  cp = sdgm_params(alpha=True)
  assert aol.qualified_gm(cp)
  setattr(cp.safetyConfigs[0], field, value)
  assert not aol.qualified_gm(cp)


def test_original_builder_write_state_and_large_cp_fallback():
  cp = sdgm_params(alpha=True)
  with warnings.catch_warnings():
    warnings.simplefilter('error')
    assert aol.qualified_gm(cp)
    cp.to_bytes()
  cp.carVin = 'x' * (64 * 1024)
  previous = aol._qualified_gm_bytes.cache_info()
  assert aol.qualified_gm(cp) == aol._qualified_gm_uncached(cp)
  assert aol._qualified_gm_bytes.cache_info() == previous


def test_ducktype_does_not_attempt_serialization():
  cp = SimpleNamespace(brand='gm', copy=lambda: cast(Callable[[str], NoReturn], pytest.fail)('Unknown type serialized'))
  with patch.object(aol, '_qualified_gm_uncached', return_value=False) as original:
    assert not aol.qualified_gm(cp)
    original.assert_called_once_with(cp)
  assert aol._qualified_gm_bytes.cache_info().currsize == 0


def test_other_brands_keep_fast_rejection_without_cp_serialization():
  cp = sdgm_params(alpha=True)
  cp.brand = 'hyundai'
  assert not aol.qualified_gm(cp)
  assert aol._qualified_gm_bytes.cache_info().currsize == 0


def test_f1_owner_checks_topology_before_full_gm_qualification():
  from openpilot.starpilot.car.gm import aol as host
  from opendbc.car.gm.values import VOLT_CC_F1_WORD, is_volt_cc_profile
  count = 0
  for cp in _configurations():
    expected = bool(aol.qualified_gm(cp) and host.aol_enabled(cp) and is_volt_cc_profile(cp) and
                    int(cp.safetyConfigs[0].safetyParam) == VOLT_CC_F1_WORD)
    with patch.object(host, 'qualified_gm', wraps=aol.qualified_gm) as qualify:
      assert host.f1_conventional_lateral_owner(cp) == expected
      assert host.f1_conventional_lateral_owner(cp.as_reader()) == expected
      if not is_volt_cc_profile(cp):
        assert qualify.call_count == 0
    count += 1
  assert count > 100
