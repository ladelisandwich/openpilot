"""The recent deadline eligibility path reuses only exact static policy."""
from unittest.mock import patch

import pytest

from openpilot.starpilot.aol import vehicle
from openpilot.starpilot.car.hyundai import aol
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD


def original(cp):
  config = aol.aol_safety_config(cp)
  policy = aol.policy_for(cp)
  return bool(policy.runtime_supported and policy.full_axis_runtime_required and config is not None and
              aol.native_accepts_cp(cp, int(config.safetyModel.raw), int(config.safetyParam)))


def outcome(function, cp):
  try:
    return ('value', function(cp))
  except (AttributeError, TypeError, ValueError, OverflowError, IndexError) as error:
    return ('error', type(error))


@pytest.fixture(params=('ev6', 'ev6_two_pandas', 'ev9'))
def cp(request):
  if request.param == 'ev9':
    from openpilot.starpilot.tests.test_ev9_lead_inputs import long_params
    result = long_params()
  else:
    from openpilot.starpilot.tests.test_ev6_startup import params
    result = params()
    result.openpilotLongitudinalControl, result.pcmCruise = True, False
    result.safetyConfigs = ([{'safetyModel': 'noOutput', 'safetyParam': 0}] if request.param == 'ev6_two_pandas' else []) + [
      {'safetyModel': 'hyundaiCanfd', 'safetyParam': 0x815}]
  result.alternativeExperience = 32
  return result


def test_actual_selfdrive_eligibility_reuses_policy_without_changing_cp(cp):
  before = cp.to_dict()
  vehicle._policy_for_bytes.cache_clear()
  aol._source_deadline_for_bytes.cache_clear()
  sd = SelfdriveD.__new__(SelfdriveD)
  sd.CP = cp
  with patch.object(aol, 'policy_for', wraps=aol.policy_for) as local:
    for _ in range(10):
      assert aol.source_deadline_supported(cp)
      assert sd.source_deadline_deferral_supported()
    assert local.call_count == 1
  assert cp.to_dict() == before
  assert original(cp)


@pytest.mark.parametrize('field', ('brand', 'flags', 'radarUnavailable', 'alphaLongitudinalAvailable', 'steerControlType',
                                  'passive', 'notCar', 'dashcamOnly', 'openpilotLongitudinalControl', 'pcmCruise',
                                  'alternativeExperience', 'safetyParam', 'safetyModel', 'safetyConfigs', 'carVin'))
def test_hot_policy_mutations_match_original_deadline_eligibility(cp, field):
  assert aol.source_deadline_supported(cp) == original(cp)
  if field == 'safetyParam':
    cp.safetyConfigs[-1].safetyParam ^= 1
  elif field == 'safetyModel':
    cp.safetyConfigs[-1].safetyModel = 'noOutput'
  elif field == 'safetyConfigs':
    cp.safetyConfigs = []
  elif field == 'steerControlType':
    cp.steerControlType = 'torque' if str(cp.steerControlType) == 'angle' else 'angle'
  elif field == 'brand':
    cp.brand = 'ford'
  elif field in ('radarUnavailable', 'alphaLongitudinalAvailable', 'passive', 'notCar', 'dashcamOnly',
                 'openpilotLongitudinalControl', 'pcmCruise'):
    setattr(cp, field, not getattr(cp, field))
  elif field == 'carVin':
    cp.carVin = 'MUTATED'
  else:
    setattr(cp, field, int(getattr(cp, field)) ^ 1)
  assert outcome(aol.source_deadline_supported, cp) == outcome(original, cp)
  assert outcome(aol.source_deadline_supported, cp.as_reader()) == outcome(original, cp)


@pytest.mark.parametrize('brand', ('honda', 'gm', 'ford', 'mazda', 'tesla', 'toyota'))
def test_non_hkg_profiles_keep_original_result(brand):
  from openpilot.starpilot.aol.tests.test_vehicle_policy_cache import profile
  cp = profile(brand)
  assert outcome(aol.source_deadline_supported, cp) == outcome(original, cp)
