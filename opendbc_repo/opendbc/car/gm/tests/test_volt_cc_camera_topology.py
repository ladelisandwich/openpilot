"""Volt CC pedal camera ownership comes from complete observed contracts."""
import copy
import json

import pytest
from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.gm import gmcan
from opendbc.car.gm.aol import qualified_gm
from opendbc.car.gm.interface import CarInterface
from opendbc.car.gm.startup_preferences import prepare_disable_longitudinal
from opendbc.car.gm.tests.test_cc_gateway_stock import pt_frames
from opendbc.car.gm.values import CAR, DBC, GMFlags, volt_cc_pedal_profile
from opendbc.safety.tests.libsafety import libsafety_py

BAD_CAMERAS = (
  {0x180: 4}, {0x180: 0}, {0x180: 8}, {0x320: 6}, {0x320: 0}, {0x320: 8},
  {0x370: 6}, {0x370: 0}, {0x370: 8}, {0x180: 4, 0x370: 6},
  {0x180: 4, 0x320: 8}, {0x180: 8, 0x320: 6}, {0x180: 4, 0x320: 6, 0x370: 8},
)


def observed(camera, *, be=False, radar=False):
  fingerprint = gen_empty_fingerprint()
  fingerprint[0].update({0x184: 8, 0x34A: 5, 0x348: 5, 0x1E1: 7, 0x1C4: 8, 0xC9: 8,
                         0xBD: 7, 0x232: 8, 0x3D1: 8, 0x1F5: 8, 0x201: 6,
                         0xBE if be else 0xF1: 6})
  fingerprint[2].update(camera)
  if radar:
    from opendbc.car.gm.radar_interface import RADAR_HEADER_MSG
    fingerprint[1][RADAR_HEADER_MSG] = 8
  return fingerprint


def params(fingerprint, *, release=False, off=False):
  cp = CarInterface.get_params(CAR.CHEVROLET_VOLT_CC, fingerprint, [], False, release, False)
  prepare_disable_longitudinal(cp, off)
  return cp


@pytest.mark.parametrize('camera', BAD_CAMERAS)
@pytest.mark.parametrize('be', (False, True))
@pytest.mark.parametrize('release', (False, True))
def test_partial_or_malformed_camera_never_selects_removed_native_owner(camera, be, release, record_property):
  cp = params(observed(camera, be=be), release=release)
  record_property('topology_witness', json.dumps({'camera': sorted(camera.items()), 'be': be, 'release': release,
                  'dashcam': cp.dashcamOnly, 'param': cp.safetyConfigs[0].safetyParam,
                  'longitudinal': cp.openpilotLongitudinalControl}))
  # BE inputs also satisfy the older nonpedal word20 branch: denial must replace it.
  for off in (False, True):
    prepare_disable_longitudinal(cp, off)
    assert cp.dashcamOnly and not cp.openpilotLongitudinalControl
    assert not cp.alphaLongitudinalAvailable and not cp.autoResumeSng
    assert cp.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.noOutput
    assert cp.safetyConfigs[0].safetyParam == 0
    assert not cp.flags & GMFlags.NO_CAMERA
    assert volt_cc_pedal_profile(cp) is None and not qualified_gm(cp)
  ci = CarInterface(cp)
  assert ci.CC.volt_cc_pedal_profile is None
  packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
  frames = pt_frames(packer, counter=0, acc_cruise=6)
  ci.update([(1_000_000_000, frames)])
  control = structs.CarControl(enabled=True, latActive=True, longActive=True)
  control.actuators.accel = .5
  _, commands = ci.apply(control.as_reader(), 1_000_000_001)
  assert not any(address == 0x200 for address, _, _ in commands)
  safety = libsafety_py.libsafety
  safety.init_tests()
  assert safety.set_safety_hooks(int(cp.safetyConfigs[0].safetyModel.raw), cp.safetyConfigs[0].safetyParam) == 0
  address, data, bus = gmcan.create_pedal_command(packer, .2, 0)
  assert not safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data))


@pytest.mark.parametrize('camera', ({}, {0x180: 4, 0x320: 6}, {0x180: 4, 0x320: 6, 0x370: 6}))
@pytest.mark.parametrize('be', (False, True))
@pytest.mark.parametrize('radar', (False, True))
@pytest.mark.parametrize('release', (False, True))
def test_complete_present_and_removed_preserve_original_owners_and_off(camera, be, radar, release):
  cp = params(observed(camera, be=be, radar=radar), release=release)
  index = 4 * int(radar) + 2 * int(not camera) + int(not be)
  assert cp.safetyConfigs[0].safetyParam == (0xE610 if release else 0xE600) + index
  assert cp.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.gm
  assert not cp.dashcamOnly and bool(cp.flags & GMFlags.NO_CAMERA) == (not camera)
  assert cp.openpilotLongitudinalControl == (not release)
  assert volt_cc_pedal_profile(cp) is not None
  assert CarInterface(cp).CC.volt_cc_pedal_profile is not None
  native = libsafety_py.libsafety
  native.init_tests()
  assert native.set_safety_hooks(int(cp.safetyConfigs[0].safetyModel.raw), cp.safetyConfigs[0].safetyParam) == 0
  prepare_disable_longitudinal(cp, True)
  assert cp.safetyConfigs[0].safetyParam == 0xE610 + index
  assert not cp.openpilotLongitudinalControl and not cp.autoResumeSng
  assert not cp.dashcamOnly and volt_cc_pedal_profile(cp) is not None


@pytest.mark.parametrize('camera', BAD_CAMERAS)
def test_recovery_requires_new_valid_observed_cp_without_mutating_denied_owner(camera):
  original = observed(camera)
  denied = params(original)
  snapshot = denied.to_dict()
  corrected = copy.deepcopy(original)
  corrected[2] = {0x180: 4, 0x320: 6}
  fresh = params(corrected)
  assert fresh.openpilotLongitudinalControl and fresh.safetyConfigs[0].safetyParam == 0xE601
  assert volt_cc_pedal_profile(fresh) is not None
  assert denied.to_dict() == snapshot
  assert denied.dashcamOnly and denied.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.noOutput


@pytest.mark.parametrize('mutation', ('bus1_only', 'missing_pedal', 'bad_pedal'))
def test_unqualified_pt_or_pedal_cannot_enter_volt_cc_pedal_owner(mutation):
  fingerprint = observed({})
  if mutation == 'bus1_only':
    fingerprint[1], fingerprint[0] = fingerprint[0], {}
  elif mutation == 'missing_pedal':
    del fingerprint[0][0x201]
  else:
    fingerprint[0][0x201] = 5
  cp = params(fingerprint)
  assert not cp.openpilotLongitudinalControl
  assert volt_cc_pedal_profile(cp) is None
  assert cp.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.noOutput


def test_missing_gear_keeps_valid_topology_but_denies_longitudinal():
  fingerprint = observed({})
  del fingerprint[0][0x1F5]
  cp = params(fingerprint)
  assert cp.safetyConfigs[0].safetyParam == 0xE613
  assert not cp.openpilotLongitudinalControl and not cp.autoResumeSng
  assert volt_cc_pedal_profile(cp) is not None


@pytest.mark.parametrize('camera', ({}, {0x180: 4, 0x320: 6}, {0x180: 4}, {0x370: 6}, {0x180: 8, 0x320: 6}))
@pytest.mark.parametrize('release', (False, True))
def test_automatic_pedal_literal_reaches_topology_gate(camera, release, monkeypatch):
  from opendbc.car import car_helpers
  from opendbc.car.can_definitions import CanData
  from opendbc.car.gm.fingerprints import FINGERPRINTS

  physical = FINGERPRINTS[CAR.CHEVROLET_VOLT_CC][1]
  packets = [CanData(address, bytes(length), 0) for address, length in physical.items()]
  packets += [CanData(address, bytes(length), 2) for address, length in camera.items()]

  def receive(wait_for_one=False):
    return [packets] if wait_for_one else []

  monkeypatch.setenv('SKIP_FW_QUERY', '1')
  monkeypatch.setenv('FINGERPRINT', '')
  ci = car_helpers.get_car(receive, lambda *_args, **_kwargs: None,
                           lambda *_args, **_kwargs: None, False, release)
  assert ci.CP.carFingerprint == CAR.CHEVROLET_VOLT_CC
  assert ci.CP.fingerprintSource == structs.CarParams.FingerprintSource.can
  valid = not camera or camera == {0x180: 4, 0x320: 6}
  if valid:
    expected = (0xE610 if release else 0xE600) + (3 if not camera else 1)
    assert ci.CP.safetyConfigs[0].safetyParam == expected
    assert ci.CC.volt_cc_pedal_profile is not None
  else:
    assert ci.CP.dashcamOnly and not ci.CP.openpilotLongitudinalControl
    assert ci.CP.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.noOutput
    assert ci.CP.safetyConfigs[0].safetyParam == 0
    assert ci.CC.volt_cc_pedal_profile is None
