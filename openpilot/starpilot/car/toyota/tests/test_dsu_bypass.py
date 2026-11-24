"""Dom bypass topology, unchanged tune projection and exact stock pairing."""

import pytest
from opendbc.car import structs
from opendbc.car.toyota import dsu_bypass
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, ToyotaFlags
from openpilot.starpilot.car.toyota.aol import qualified
from openpilot.starpilot.car.toyota.tests.test_rav4h_caller_native import firmware


def acquired(candidate):
  if candidate == CAR.TOYOTA_MATRIX_RETROFIT:
    return []
  fw = firmware(CAR.TOYOTA_PRIUS if candidate == CAR.TOYOTA_PRIUS_RETROFIT else candidate)
  if candidate == CAR.TOYOTA_PRIUS_RETROFIT:
    for row in fw:
      if row.ecu == structs.CarParams.Ecu.eps:
        row.fwVersion = b'8965B47070' + bytes(6)
  return fw


def fingerprint(candidate, address=0x343):
  pt = {0x260: 8, 0x262: 8 if candidate in dsu_bypass.EPS8_CARS or candidate == CAR.TOYOTA_PRIUS else 5, 0x224: 8, 0x226: 8, 0x1D2: 8, 0x1D3: 8}
  return {0: pt, 1: {}, 2: {address: 8}}


def factory(candidate, address=0x343):
  return CarInterface.get_params(candidate, fingerprint(candidate, address), acquired(candidate), False, False, False)


@pytest.mark.parametrize('candidate', sorted(dsu_bypass.PROFILES))
@pytest.mark.parametrize('address', (0x343, 0x4CB))
@pytest.mark.parametrize('stock', (False, True))
def test_exact_identity_current_tunes_and_stock_pair(candidate, address, stock):
  fp = fingerprint(candidate, address)
  cp = factory(candidate, address)
  baseline = CarInterface.get_params(candidate, {**fp, 2: {}}, acquired(candidate), False, False, False)
  assert dsu_bypass.configured(cp) and qualified(cp) and not cp.dashcamOnly and cp.pcmCruise
  assert cp.flags & ToyotaFlags.DSU_BYPASS and not cp.flags & ToyotaFlags.LONG_FILTER
  assert cp.safetyConfigs[0].safetyParam == dsu_bypass.PROFILES[candidate]
  for field in (
    'lateralTuning',
    'longitudinalTuning',
    'steerActuatorDelay',
    'steerLimitTimer',
    'longitudinalActuatorDelay',
    'minEnableSpeed',
    'minSteerSpeed',
    'wheelSpeedFactor',
    'mass',
    'wheelbase',
    'steerRatio',
    'centerToFront',
  ):
    if field == 'longitudinalActuatorDelay':
      expected_delay = (.05 if candidate in (CAR.TOYOTA_PRIUS, CAR.TOYOTA_PRIUS_RETROFIT) or
                        candidate == CAR.TOYOTA_CAMRY and cp.flags & ToyotaFlags.HYBRID else
                        .4 if candidate == CAR.TOYOTA_HIGHLANDER and not cp.flags & ToyotaFlags.HYBRID else
                        .5 if candidate == CAR.TOYOTA_SIENNA else baseline.longitudinalActuatorDelay)
      assert cp.longitudinalActuatorDelay == pytest.approx(expected_delay)
    else:
      assert cp.to_dict()[field] == baseline.to_dict()[field], field
  if stock:
    assert dsu_bypass.prepare_stock(cp) and dsu_bypass.configured(cp) and qualified(cp)
    assert not cp.openpilotLongitudinalControl and cp.safetyConfigs[0].safetyParam == dsu_bypass.PROFILES[candidate] | 0x200


@pytest.mark.parametrize('candidate', (CAR.TOYOTA_CAMRY, CAR.LEXUS_IS, CAR.TOYOTA_PRIUS, CAR.TOYOTA_PRIUS_RETROFIT))
@pytest.mark.parametrize('address', (0x343, 0x4CB))
def test_native_bus_mirror_is_not_adapter_identity(candidate, address):
  fp = fingerprint(candidate, address)
  fw = acquired(candidate)
  if candidate in (CAR.TOYOTA_PRIUS, CAR.TOYOTA_PRIUS_RETROFIT):
    for row in fw:
      if row.ecu == structs.CarParams.Ecu.fwdCamera:
        row.fwVersion = b'8646F47050'
  fp[0][address] = 8
  assert not dsu_bypass.hardware_supported(candidate, fp, fw)


@pytest.mark.parametrize('kind', ('foreign-pedal', 'filter', 'sdsu', 'camera-dlc', 'unknown-eps', 'logging', 'wrong-fw-bus'))
def test_mixed_topology_or_unknown_firmware_does_not_enroll(kind):
  fp = fingerprint(CAR.TOYOTA_RAV4H)
  fw = acquired(CAR.TOYOTA_RAV4H)
  if kind in ('foreign-pedal', 'filter', 'sdsu'):
    fp[0][{'foreign-pedal': 0x201, 'filter': 0x2AA, 'sdsu': 0x2FF}[kind]] = 6 if kind == 'foreign-pedal' else 8
  elif kind == 'camera-dlc':
    fp[2][0x343] = 7
  else:
    row = next(f for f in fw if f.ecu == structs.CarParams.Ecu.eps)
    if kind == 'unknown-eps':
      row.fwVersion = b'unknown'
    elif kind == 'logging':
      row.logging = True
    else:
      row.bus = 2
  assert not dsu_bypass.hardware_supported(CAR.TOYOTA_RAV4H, fp, fw)


@pytest.mark.parametrize('candidate', sorted(dsu_bypass.PROFILES))
def test_native_eps_and_brake_basis_matches_current_car(candidate):
  fp = fingerprint(candidate)
  baseline = CarInterface.get_params(candidate, {**fp, 2: {}}, acquired(candidate), False, False, False)
  cp = factory(candidate)
  assert cp.safetyConfigs[0].safetyParam & 0x1FF == baseline.safetyConfigs[0].safetyParam & 0x1FF


@pytest.mark.parametrize('candidate', sorted(dsu_bypass.PROFILES))
def test_dom_auto_resume_and_stop_timer_are_model_specific(candidate):
  from opendbc.car.toyota import legacy_stop_timer
  cp = factory(candidate)
  assert cp.autoResumeSng == (candidate in dsu_bypass.AUTO_RESUME_CARS)
  assert legacy_stop_timer.enabled(cp) == (candidate not in dsu_bypass.AUTO_RESUME_CARS)
  assert dsu_bypass.prepare_stock(cp)
  assert not cp.autoResumeSng and not legacy_stop_timer.enabled(cp)
