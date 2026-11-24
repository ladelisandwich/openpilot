import pytest
from opendbc.car import gen_empty_fingerprint
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import ToyotaFlags
from openpilot.starpilot.car.toyota.tests.test_camera_pedal_extension_callers import NEW_MODELS
from openpilot.starpilot.car.toyota.tests.test_pedal_caller_native import firmware
from openpilot.starpilot.car.toyota.aol import qualified


@pytest.mark.parametrize('candidate', NEW_MODELS)
def test_pedal_admission_changes_only_hardware_flags_and_native_owner(candidate):
  fp = gen_empty_fingerprint()
  fw = firmware(candidate)
  baseline = CarInterface.get_params(candidate, fp, fw, False, False, False)
  fp[0][0x201] = 6
  cp = CarInterface.get_params(candidate, fp, fw, False, False, False)
  expected = baseline.to_dict()
  expected['flags'] |= int(ToyotaFlags.GAS_INTERCEPTOR)
  expected['safetyConfigs'][0]['safetyParam'] = 0x2049
  expected['stopAccel'] = -1.5
  expected['longitudinalActuatorDelay'] = pytest.approx(max(baseline.longitudinalActuatorDelay, .2))
  assert cp.to_dict() == expected
  assert qualified(cp)
