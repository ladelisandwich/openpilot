"""Cache complete static HKG eligibility, never a live permission or lease."""
from unittest.mock import patch

import pytest

from openpilot.starpilot.aol import vehicle
from openpilot.starpilot.car.hyundai import aol
from openpilot.starpilot.aol.tests.test_hkg_source_policy_cache import cp as cp, original, outcome


def test_repeated_static_eligibility_is_evaluated_once_for_exact_cp(cp):
  before = cp.to_dict()
  aol._source_deadline_for_bytes.cache_clear()
  vehicle._policy_for_bytes.cache_clear()
  uncached = aol._source_deadline_supported_uncached
  with patch.object(aol, '_source_deadline_supported_uncached', wraps=uncached) as admission:
    for _ in range(20):
      assert aol.source_deadline_supported(cp)
      assert aol.source_deadline_supported(cp.as_reader())
    assert admission.call_count == 1
  assert cp.to_dict() == before and original(cp)


@pytest.mark.parametrize('field', ('mass', 'carVin', 'carFw', 'safetyParam', 'alternativeExperience', 'flags',
                                  'passive', 'pcmCruise', 'openpilotLongitudinalControl'))
def test_mutating_same_cp_invalidates_static_admission_for_every_field(cp, field):
  aol._source_deadline_for_bytes.cache_clear()
  uncached = aol._source_deadline_supported_uncached
  with patch.object(aol, '_source_deadline_supported_uncached', wraps=uncached) as admission:
    before = outcome(aol.source_deadline_supported, cp)
    assert admission.call_count == 1
    if field == 'mass':
      cp.mass += 1.
    elif field == 'carVin':
      cp.carVin = 'CHANGED'
    elif field == 'carFw':
      cp.carFw = [{'ecu': 'eps', 'fwVersion': b'changed firmware'}]
    elif field == 'safetyParam':
      cp.safetyConfigs[-1].safetyParam ^= 1
    elif field in ('passive', 'pcmCruise', 'openpilotLongitudinalControl'):
      setattr(cp, field, not getattr(cp, field))
    else:
      setattr(cp, field, int(getattr(cp, field)) ^ 1)
    after = outcome(aol.source_deadline_supported, cp)
    assert admission.call_count == 2
  assert before[0] == 'value' and after == outcome(original, cp)
  assert outcome(aol.source_deadline_supported, cp.as_reader()) == after


def test_unknown_wrapper_uses_original_dispatch_without_admission_cache(cp):
  class Proxy:
    def __getattr__(self, name):
      return getattr(cp, name)
  proxy = Proxy()
  with patch.object(aol, '_source_deadline_for_bytes', side_effect=AssertionError('unknown CP reached cache')):
    assert outcome(aol.source_deadline_supported, proxy) == outcome(original, proxy)


def test_cache_failure_keeps_original_admission_and_cp_untouched(cp):
  before = cp.to_dict()
  with patch.object(aol, '_source_deadline_for_bytes', side_effect=RuntimeError('cache unavailable')):
    assert outcome(aol.source_deadline_supported, cp) == outcome(original, cp)
  assert cp.to_dict() == before


def test_snapshot_failure_keeps_original_admission(cp):
  with patch.object(vehicle, '_car_params_bytes', return_value=None), \
       patch.object(aol, '_source_deadline_for_bytes', side_effect=AssertionError('snapshot unavailable')):
    assert outcome(aol.source_deadline_supported, cp) == outcome(original, cp)


@pytest.mark.parametrize('brand', ('honda', 'gm', 'ford', 'mazda', 'tesla', 'toyota'))
def test_other_brands_reject_before_serializing_deadline_cache_key(brand):
  from openpilot.starpilot.aol.tests.test_vehicle_policy_cache import profile
  cp = profile(brand)
  expected = outcome(original, cp)
  with patch.object(vehicle, '_car_params_bytes', side_effect=AssertionError('unrelated owner serialized')):
    assert outcome(aol.source_deadline_supported, cp) == expected
    assert outcome(aol.source_deadline_supported, cp.as_reader()) == expected
