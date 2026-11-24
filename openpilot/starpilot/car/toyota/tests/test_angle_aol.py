from pathlib import Path
import pytest
from opendbc.car import gen_empty_fingerprint
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, ToyotaFlags
from opendbc.car.toyota.angle_control import configured
from openpilot.starpilot.car.toyota.aol import angle_qualified
from openpilot.starpilot.car.toyota.tests.test_radar_preferences import FilesystemParams
from openpilot.starpilot.vehicle_preferences import VehicleStartupPreferences


def factory(alpha, filtered):
  fp = gen_empty_fingerprint()
  if filtered:
    fp[0][0x2AA] = 8
  return CarInterface.get_params(CAR.TOYOTA_RAV4_TSS2_2023, fp, [], alpha, False, False)


@pytest.mark.parametrize('alpha', [False, True])
@pytest.mark.parametrize('filtered', [False, True])
def test_angle_exact_factory_owner_and_startup_off(alpha, filtered, tmp_path):
  cp = factory(alpha, filtered)
  assert configured(cp) and angle_qualified(cp)
  assert not angle_qualified(cp, marked_only=True)
  params = FilesystemParams(tmp_path)
  Path(params.get_param_path('DisableOpenpilotLongitudinal')).write_bytes(b'1')
  prior_tune = cp.lateralTuning.to_dict()
  owner = VehicleStartupPreferences.read(params, enabled=True)
  owner.prepare(cp)
  owner.finalize(cp)
  assert configured(cp) and not cp.openpilotLongitudinalControl
  assert cp.safetyConfigs[0].safetyParam == (0x1649 if filtered else 0x649)
  assert not cp.flags & ToyotaFlags.DISABLE_RADAR
  assert cp.lateralTuning.to_dict() == prior_tune


@pytest.mark.parametrize('field,value', [('passive', True), ('dashcamOnly', True), ('notCar', True),
                                        ('secOcRequired', True), ('pcmCruise', False),
                                        ('flags', 0), ('alternativeExperience', 128)])
def test_angle_aol_rejects_wrong_configuration(field, value):
  cp = factory(True, True)
  setattr(cp, field, value)
  assert not angle_qualified(cp)


@pytest.mark.parametrize('word', [73, 585, 0x1049, 0x1249, 0x1448, 0x144A, 0x2449])
def test_angle_aol_rejects_unrelated_native_words(word):
  cp = factory(True, True)
  cp.safetyConfigs[0].safetyParam = word
  assert not angle_qualified(cp)
