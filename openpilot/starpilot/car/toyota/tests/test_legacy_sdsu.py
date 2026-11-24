import pytest
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.toyota import legacy_sdsu
from opendbc.car.toyota.carstate import CarState
from opendbc.car.toyota.interface import CarInterface, apply_toyota_auto_hold
from opendbc.car.toyota.values import CAR, CarControllerParams, EPS_SCALE, TOYOTA_AUTO_HOLD_CARS
from openpilot.starpilot.car.toyota.tests.test_pedal_caller_native import firmware
from openpilot.starpilot.car.toyota.aol import qualified


@pytest.mark.parametrize('candidate', sorted(legacy_sdsu.PROFILES))
@pytest.mark.parametrize('alpha', [False, True])
@pytest.mark.parametrize('hybrid', [False, True])
def test_full_configuration_and_tune_preserved(candidate, alpha, hybrid):
  fw = firmware(candidate)
  if hybrid:
    fw.append(
      structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.hybrid, address=0x7E2, fwVersion=b'synthetic hybrid ECU response', brand='toyota', bus=0, logging=True)
    )
  fp = gen_empty_fingerprint()
  baseline = CarInterface.get_params(candidate, fp, fw, alpha, False, False)
  fp[0][0x2FF] = 8
  actual = CarInterface.get_params(candidate, fp, fw, alpha, False, False)
  if candidate == CAR.TOYOTA_MATRIX_RETROFIT and hybrid:
    # A logged hybrid donor is not the admitted manual non-hybrid Matrix.
    assert not legacy_sdsu.configured(actual) and not qualified(actual)
    assert not apply_toyota_auto_hold(actual, True)
    return
  assert legacy_sdsu.configured(actual) and qualified(actual)
  assert actual.openpilotLongitudinalControl and not actual.alphaLongitudinalAvailable
  assert actual.autoResumeSng == (candidate == CAR.TOYOTA_PRIUS_V)
  assert CarState(actual).eps_torque_scale == EPS_SCALE[candidate] / 100.0
  before, after = baseline.to_dict(), actual.to_dict()
  for name in ('flags', 'safetyConfigs', 'openpilotLongitudinalControl', 'alphaLongitudinalAvailable', 'autoResumeSng'):
    before.pop(name)
    after.pop(name)
  assert before == after
  old_limits, new_limits = CarControllerParams(baseline), CarControllerParams(actual)
  assert (old_limits.ACCEL_MIN, old_limits.ACCEL_MAX) == (new_limits.ACCEL_MIN, new_limits.ACCEL_MAX)
  hold = actual.as_reader().as_builder()
  assert apply_toyota_auto_hold(hold, True) == (candidate in TOYOTA_AUTO_HOLD_CARS)
  assert actual.to_dict() == CarInterface.get_params(candidate, fp, fw, alpha, False, False).to_dict()
  assert legacy_sdsu.configured(actual)
  assert legacy_sdsu.prepare_stock(actual) and legacy_sdsu.configured(actual) and qualified(actual)
  assert actual.safetyConfigs[0].safetyParam == legacy_sdsu.PROFILES[candidate][1]


@pytest.mark.parametrize('candidate', sorted(legacy_sdsu.PROFILES))
@pytest.mark.parametrize('bad', ['source7', 'copy7', 'only_copy', 'bus1', 'pedal', 'mixed', 'malformed_camera', 'firmware'])
def test_hardware_boundaries(candidate, bad):
  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = fp[2][0x2FF] = 8
  fw = firmware(candidate)
  if bad == 'source7':
    fp[0][0x2FF] = 7
  elif bad == 'copy7':
    fp[2][0x2FF] = 7
  elif bad == 'only_copy':
    del fp[0][0x2FF]
  elif bad == 'bus1':
    fp[1][0x2FF] = 8
  elif bad == 'pedal':
    fp[0][0x201] = 6
  elif bad == 'mixed':
    fp[0][0x2AA] = 8
  elif bad == 'malformed_camera':
    fp[2][0x343] = 7
  else:
    fw = []
  if candidate == CAR.TOYOTA_MATRIX_RETROFIT and bad == 'firmware':
    # Manual enrollment deliberately permits an empty acquisition; this is not
    # evidence of a catalog fingerprint or permission supplied by the filter.
    assert not fw and legacy_sdsu.hardware_supported(candidate, fp, fw)
  else:
    assert not legacy_sdsu.hardware_supported(candidate, fp, fw)


@pytest.mark.parametrize('candidate', sorted(legacy_sdsu.PROFILES))
@pytest.mark.parametrize('field,value', [('fwVersion', b'unknown'), ('bus', 1), ('brand', 'ford'), ('logging', True), ('address', 0x123), ('subAddress', 1)])
def test_catalog_identity_is_required(candidate, field, value):
  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = 8
  fw = firmware(candidate)
  if candidate == CAR.TOYOTA_MATRIX_RETROFIT:
    # Synthetic acquired observation, not a fabricated firmware-catalog entry.
    fw = [structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.fwdCamera, address=0x750,
      fwVersion=b'manual metadata test observation', brand='toyota', bus=0)]
  camera = next(item for item in fw if item.ecu == structs.CarParams.Ecu.fwdCamera)
  setattr(camera, field, value)
  if candidate == CAR.TOYOTA_MATRIX_RETROFIT:
    # Its fixed-selection contract rejects invalid transport metadata and
    # accepts nonempty observations without claiming catalog addresses/bytes.
    assert legacy_sdsu.hardware_supported(candidate, fp, fw) == (field in ('fwVersion', 'address', 'subAddress'))
  else:
    assert not legacy_sdsu.hardware_supported(candidate, fp, fw)


@pytest.mark.parametrize('candidate', [CAR.TOYOTA_PRIUS_V, CAR.LEXUS_CTH])
def test_missing_eps_catalog_never_fabricates_a_response(candidate):
  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = 8
  fw = firmware(candidate)
  assert not any(item.ecu == structs.CarParams.Ecu.eps for item in fw)
  assert legacy_sdsu.hardware_supported(candidate, fp, fw)
  assert not legacy_sdsu.hardware_supported(candidate, fp, [item for item in fw if item.ecu != structs.CarParams.Ecu.engine])
  fw.append(structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.eps, address=0x7A1, fwVersion=b'unknown EPS response', brand='toyota', bus=0))
  assert not legacy_sdsu.hardware_supported(candidate, fp, fw)
