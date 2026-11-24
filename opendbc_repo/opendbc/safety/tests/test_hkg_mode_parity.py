"""HKG mode parity through real callers; transport and clocks are synthetic."""
from contextlib import ExitStack
from unittest import mock

import pytest
from opendbc.car import CanData
from opendbc.car.hyundai.values import CAR
from opendbc.car.hyundai import discovery
from opendbc.car.hyundai.canfd_angle_aol import MODELS
from opendbc.car.hyundai.tests.test_stock_angle_aol import source_profile
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.tests import test_hkg_cancel_joined as joined
from openpilot.starpilot.aol.tests import test_ioniq_availability as callers
from openpilot.starpilot.aol.tests.test_canfd_stock_profiles import params
from openpilot.starpilot.aol.tests.test_canfd_location_native_recovery import params as topology_params
from openpilot.starpilot.aol.tests.test_ordinary_source_availability import settings


@pytest.mark.parametrize('alpha', [False, True])
def test_gv70_physical_lkas_and_cancel_preserve_independent_lateral(alpha, monkeypatch):
  if joined._native_variant() == 'release' and alpha:
    joined._release_long_denial(alternate=False)
    return
  joined._exercise_join(CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN, False, 'cancel', 'toggle-off',
                        monkeypatch, shared_long=alpha)


@pytest.mark.parametrize('family', ['gv70', 'lka', 'lka_alt', 'camera', 'radar', 'camera_buttons_alt'] +
                         ['angle:' + str(identity) for identity in sorted(MODELS)])
@pytest.mark.parametrize('socket_timeout', [False, True])
@pytest.mark.parametrize('age_ns,published_offset_ns', [(31_428_438, 5_014_375), (31_627_188, 5_542_344),
                                                        (33_086_408, 5_833_377)])
def test_stock_hkg_recorded_cutoff_keeps_original_proof(family, socket_timeout, age_ns, published_offset_ns):
  case = callers.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    cleanup.enter_context(OpenpilotPrefix())
    cp = (source_profile(CAR(family.removeprefix('angle:'))) if family.startswith('angle:') else
          params(CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN) if family == 'gv70' else topology_params(family))
    sd, controls, drive = case._authority_callers(False, cp=cp, genuine_events=True)
    source = 10_020_000_000
    for tick in (0, 1):
      stamp = 10_000_000_000 + tick * 10_000_000
      drive(tick, gas=True, **settings('lateral_only', stamp, tick + 1))
    original, _, command = drive(2, gas=True, state_stamp=source, offset_ns=published_offset_ns,
                                 **settings('lateral_only', source, 3))
    assert original.lateralActive and command.latActive and not original.longitudinalActive
    before = original.to_dict()
    deadline = original.validUntilMonoTime
    sd.car_state_poller = mock.Mock()
    sd.car_state_poller.poll.return_value = []
    axis, _, command = drive(5, gas=True, timeout=socket_timeout, state_stamp=source,
                             offset_ns=age_ns - 30_000_000, **settings('lateral_only', source, 3))
    assert sd.aol_axis_publication_deferred
    assert axis.to_dict() == before and axis.validUntilMonoTime == deadline
    assert not command.longActive
    if socket_timeout:
      assert sd.aol_car_state_log_ns == 0 and not sd.conditional_car_state_valid
    with mock.patch('openpilot.selfdrive.controls.controlsd.time.monotonic_ns', return_value=deadline + 1):
      expired, _ = controls.state_control()
    assert not expired.latActive and not expired.longActive
    axis, _, command = drive(5, gas=True, timeout=True, state_stamp=source,
                             offset_ns=age_ns - 30_000_000 + 100_000, **settings('lateral_only', source, 3))
    assert not sd.aol_axis_publication_deferred
    assert not axis.lateralActive and not command.latActive and not command.longActive


# Actual startup frames, restricted to the diagnostic-layout signature. These
# select normal wiring, not a fingerprint or permission to control the car.
CLASSIC_FRAMES = (
  (0, 593, '700361a6008cc77f'),
  (0, 902, '4cc749475a475f47'),
  (0, 916, '71a4000071041d83'),
  (0, 1056, '401460cadc916b00'),
  (0, 1057, '000000ffe37f2126'),
  (0, 1265, 'e04666d0'),
  (2, 832, '000000246400a018'),
)


def classic_frames() -> list[CanData]:
  return [CanData(address, bytes.fromhex(data), bus) for bus, address, data in CLASSIC_FRAMES]


def test_recorded_classic_layout_avoids_obd_mux_and_preserves_firmware_identity():
  from opendbc.car import car_helpers
  from opendbc.car.vin import VIN_UNKNOWN

  frames = classic_frames()

  def receive(wait_for_one: bool = False) -> list[list[CanData]]:
    return [frames]
  assert discovery.normal_bus_topology(lambda **kw: [frames])
  switches = []
  with mock.patch.object(car_helpers, 'get_vin', return_value=(-1, -1, VIN_UNKNOWN)), \
       mock.patch.object(car_helpers, 'get_fw_versions', return_value=[]) as query, \
       mock.patch.object(car_helpers, 'get_present_ecus') as present, \
       mock.patch.object(car_helpers, 'match_fw_to_car', return_value=(True, set())), \
       mock.patch.object(car_helpers, 'can_fingerprint', return_value=(None, {})):
    actual = car_helpers.fingerprint(receive, lambda frames: None, switches.append, None)
  assert actual[0] is None  # A layout never invents a matched vehicle.
  assert switches and not any(switches)
  assert query.call_args.kwargs == {'query_brand': 'hyundai', 'normal_bus_only': True}
  present.assert_not_called()


@pytest.mark.parametrize('defect', ['missing', 'length', 'bus', 'echo'])
@pytest.mark.parametrize('index', range(7))
def test_classic_query_signature_rejects_incomplete_or_wrong_layout(defect, index):
  frames = classic_frames()
  frame = frames.pop(index)
  if defect != 'missing':
    frames.append(CanData(frame.address, frame.dat[:-1] if defect == 'length' else frame.dat,
                          frame.src + (128 if defect == 'echo' else 1) if defect != 'length' else frame.src))
  assert not discovery.normal_bus_topology(lambda **kw: [frames])


def test_gv70_long_profile_admits_independent_aol():
  from opendbc.car.hyundai.tests.test_gv70_longitudinal import params as gv70_params
  from openpilot.starpilot.car.hyundai.aol import policy_for
  cp = gv70_params()
  assert cp.openpilotLongitudinalControl and cp.safetyConfigs[0].safetyParam == 0x2015
  policy = policy_for(cp)
  assert policy.intent_supported and policy.runtime_supported and policy.explicit_latch

def test_gv70_recorded_cutoff_baseline():
  test_stock_hkg_recorded_cutoff_keeps_original_proof(
    'gv70', True, 31_428_438, 5_014_375)

@pytest.mark.parametrize('fault', ['can', 'epsTemporary', 'epsPermanent', 'acc', 'brake',
                                 'pandaRx', 'relay', 'nativeInvalid'])
@pytest.mark.parametrize('identity', [CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN, CAR.HYUNDAI_IONIQ_5_PE, CAR.KIA_EV9])
def test_stock_hkg_original_proof_wait_never_defers_new_fault(fault, identity):
  case = callers.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    cleanup.enter_context(OpenpilotPrefix())
    cp = params(identity) if identity == CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN else source_profile(identity)
    sd, _, drive = case._authority_callers(False, cp=cp, genuine_events=True)
    source = 10_020_000_000
    for tick in (0, 1, 2):
      stamp = 10_000_000_000 + tick * 10_000_000
      axis, _, command = drive(tick, state_stamp=stamp, offset_ns=5_014_375 if tick == 2 else 0,
                               **settings('lateral_only', stamp, tick + 1))
    assert axis.lateralActive and command.latActive
    state = callers.car_state()
    changes = {}
    if fault == 'can':
      state.canValid = False
    elif fault == 'epsTemporary':
      state.steerFaultTemporary = True
    elif fault == 'epsPermanent':
      state.steerFaultPermanent = True
    elif fault == 'acc':
      state.accFaulted = True
    elif fault == 'brake':
      changes['brake'] = True
    elif fault == 'pandaRx':
      changes['panda_fault'] = True
    elif fault == 'relay':
      changes['relay_fault'] = True
    elif fault == 'nativeInvalid':
      changes['native_valid'] = False
    if fault in ('can', 'epsTemporary', 'epsPermanent', 'acc'):
      changes['observed_state'] = state
    axis, _, command = drive(5, state_stamp=source, offset_ns=1_428_438,
                             **settings('lateral_only', source, 3), **changes)
    assert not sd.aol_axis_publication_deferred
    assert not axis.lateralActive and not command.latActive and not command.longActive


@pytest.mark.parametrize('defect', ['pcm', 'long', 'angle', 'altLka', 'altButtons', 'camera', 'passive', 'dashcam', 'notCar', 'experience'])
def test_gv70_long_aol_rejects_unsupported_profile(defect):
  from opendbc.car.hyundai.tests.test_gv70_longitudinal import params as gv70_params
  from opendbc.car.hyundai.canfd_stock_aol import qualified_long
  from opendbc.car.hyundai.values import HyundaiFlags
  from openpilot.starpilot.car.hyundai.aol import policy_for, native_accepts_cp

  cp = gv70_params()
  policy = policy_for(cp)
  cp.safetyConfigs[0].safetyParam |= policy.safety_param_addition
  cp.alternativeExperience |= policy.alternative_experience_addition
  assert qualified_long(cp, marked_only=True)
  assert cp.safetyConfigs[0].safetyParam == 0x2815
  assert native_accepts_cp(cp, int(cp.safetyConfigs[0].safetyModel.raw), 0x2815)
  if defect == 'pcm':
    cp.pcmCruise = True
  elif defect == 'long':
    cp.openpilotLongitudinalControl = False
  elif defect == 'passive':
    cp.passive = True
  elif defect == 'dashcam':
    cp.dashcamOnly = True
  elif defect == 'notCar':
    cp.notCar = True
  elif defect == 'experience':
    cp.alternativeExperience = 64
  else:
    flag = {'angle': HyundaiFlags.CANFD_ANGLE_STEERING, 'altLka': HyundaiFlags.CANFD_LKA_STEER_MSG_ALT,
            'altButtons': HyundaiFlags.CANFD_ALT_BUTTONS, 'camera': HyundaiFlags.CANFD_CAMERA_SCC}[defect]
    cp.flags = int(cp.flags) | int(flag)
  assert not qualified_long(cp, marked_only=True)
  assert not native_accepts_cp(cp, int(cp.safetyConfigs[0].safetyModel.raw), cp.safetyConfigs[0].safetyParam)
