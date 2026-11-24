"""Saved MEB choices, exact CP/native profiles and independent longitudinal output."""
from types import SimpleNamespace
from pathlib import Path
import pytest
from opendbc.car import structs
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.meb_modes import MEB_CARS, base_word, profile_word, qualified, prepare_stock
from opendbc.car.volkswagen.mebcan import MebLongStateMachine
from opendbc.car.volkswagen.values import CarControllerParams
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.car.volkswagen.preferences import stock_long_requested
from openpilot.starpilot.aol.vehicle import native_matches_cp, policy_for
from openpilot.starpilot.vehicle_preferences import VehicleStartupPreferences


def params(identity, *, gateway=True, offset=0, fixed=False, alt=False):
  fp = {bus: {} for bus in range(8)}
  fp[offset][0x25D] = 8
  if gateway:
    fp[offset + 1][0x13D] = 32
  if alt:
    fp[offset][0x3DC] = 8
  cp = CarInterface.get_params(identity, fp, [], False, False, False)
  cp.fingerprintSource = structs.CarParams.FingerprintSource.fixed if fixed else structs.CarParams.FingerprintSource.fw
  return cp


@pytest.mark.parametrize('identity', sorted(MEB_CARS))
@pytest.mark.parametrize('offset,fixed', ((0, False), (0, True), (4, False), (4, True)))
@pytest.mark.parametrize('stock', (False, True))
@pytest.mark.parametrize('alt', (False, True))
def test_exact_gateway_configuration_and_camera_boundary(identity, offset, fixed, stock, alt):
  cp = params(identity, offset=offset, fixed=fixed, alt=alt)
  assert cp.openpilotLongitudinalControl and not cp.pcmCruise
  prepare_stock(cp, stock)
  assert cp.pcmCruise == stock and cp.openpilotLongitudinalControl != stock
  assert qualified(cp) and cp.safetyConfigs[-1].safetyParam == base_word(cp)
  policy = policy_for(cp)
  assert policy.intent_supported
  cp.safetyConfigs[-1].safetyParam |= policy.safety_param_addition
  cp.alternativeExperience |= policy.alternative_experience_addition
  assert cp.safetyConfigs[-1].safetyParam == profile_word(cp)
  assert native_matches_cp(cp, 34, profile_word(cp))
  assert not native_matches_cp(cp, 15, profile_word(cp))
  if offset:
    assert cp.safetyConfigs[0].safetyParam == 0
  camera = params(identity, gateway=False, offset=offset, fixed=fixed, alt=alt)
  before = camera.to_dict()
  prepare_stock(camera, True)
  assert camera.to_dict() == before and camera.dashcamOnly
  assert not qualified(camera) and not policy_for(camera).intent_supported


@pytest.mark.parametrize('saved,expected', ((None, False), (b'1', False), (b'0', True), (b'', True), (b'broken', True)))
@pytest.mark.parametrize('disable', (False, True))
def test_actual_saved_alpha_off_absence_and_disable_precedence(saved, expected, disable):
  with OpenpilotPrefix():
    store = Params()
    store.remove('AlphaLongitudinalEnabled')
    if saved is not None:
      Path(store.get_param_path('AlphaLongitudinalEnabled')).write_bytes(saved)
    store.put_bool('DisableOpenpilotLongitudinal', disable, block=True)
    assert stock_long_requested(store) == expected
    snapshot = VehicleStartupPreferences.read(store, enabled=True)
    cp = params(sorted(MEB_CARS)[0])
    snapshot.prepare(cp)
    snapshot.finalize(cp)
    assert cp.pcmCruise == (disable or expected)
    assert cp.openpilotLongitudinalControl != (disable or expected)
    path = Path(store.get_param_path('AlphaLongitudinalEnabled'))
    assert (path.read_bytes() if path.exists() else None) == saved


def test_unreadable_alpha_is_stock_and_does_not_rewrite():
  class Unreadable:
    def get(self, *args, **kwargs):
      raise OSError('unreadable fixture')

    def get_param_path(self, *args):
      raise OSError('unreadable fixture')
  assert stock_long_requested(Unreadable())


def test_marked_lateral_only_uses_inactive_accel_and_standby_status():
  cp = params(sorted(MEB_CARS)[0])
  policy = policy_for(cp)
  cp.alternativeExperience = 32
  cp.safetyConfigs[-1].safetyParam |= policy.safety_param_addition
  machine = MebLongStateMachine(cp, CarControllerParams(cp))
  state = SimpleNamespace(out=structs.CarState(canValid=True, vEgo=20.),
                          esp_standstill=False, esp_hold_confirmation=False)
  state.out.cruiseState.available = True
  command = structs.CarControl(enabled=True, latActive=True, longActive=False)
  command.actuators.longControlState = structs.CarControl.Actuators.LongControlState.pid
  accel, status, hold, braking, launching = machine.update(state, command, .5)
  assert accel == machine.CCP.ACCEL_INACTIVE
  assert status == machine.acc_status_vals['ACC_STANDBY']
  assert hold == machine.acc_hold_type_vals['KEINE_ANFORDERUNG'] and not braking and not launching


@pytest.mark.parametrize('pedal', ('gasPressed', 'brakePressed'))
def test_marked_long_withdraws_immediately_on_current_pedal_before_native_status(pedal):
  cp = params(sorted(MEB_CARS)[0])
  policy = policy_for(cp)
  cp.alternativeExperience = 32
  cp.safetyConfigs[-1].safetyParam |= policy.safety_param_addition
  machine = MebLongStateMachine(cp, CarControllerParams(cp))
  state = SimpleNamespace(out=structs.CarState(canValid=True, vEgo=20.),
                          esp_standstill=False, esp_hold_confirmation=False)
  state.out.cruiseState.available = True
  setattr(state.out, pedal, True)
  command = structs.CarControl(enabled=True, latActive=True, longActive=True)
  command.actuators.longControlState = structs.CarControl.Actuators.LongControlState.pid
  accel, status, hold, braking, launching = machine.update(state, command, .5)
  assert not machine._long_active(state, command)
  assert accel == machine.CCP.ACCEL_INACTIVE and status == machine.acc_status_vals['ACC_STANDBY']
  assert hold == machine.acc_hold_type_vals['KEINE_ANFORDERUNG'] and not braking and not launching


def test_gateway_ui_saved_off_and_parked_cas_guards():
  from openpilot.starpilot.ui.feature_settings_owner import FeatureSettingsOwner
  from openpilot.starpilot.ui.feature_settings_state import FeatureSettingsRequest
  with OpenpilotPrefix():
    store = Params()
    store.put_bool('SafeMode', False, block=True)
    cp = params(sorted(MEB_CARS)[0])
    allowed = [True]
    owner = FeatureSettingsOwner(store, lambda group: allowed[0] and group == 'parked_preferences',
                                 vehicle_fingerprint=lambda: cp.carFingerprint, vehicle_params=lambda: cp)
    capability = owner._bolt_disable_capability()
    assert capability is not None
    request = FeatureSettingsRequest('DisableOpenpilotLongitudinal', None, 'On', confirmation=True,
                                     vehicle_fingerprint=cp.carFingerprint, capability=capability)
    assert owner.apply(request)
    assert not owner.apply(request)  # Original source bytes are stale after the successful commit.
    request = FeatureSettingsRequest('DisableOpenpilotLongitudinal', b'1', 'Off', confirmation=True,
                                     vehicle_fingerprint=cp.carFingerprint, capability=capability)
    allowed[0] = False
    assert not owner.apply(request)
    allowed[0] = True
    store.put_bool('AlphaLongitudinalEnabled', False, block=True)
    assert not owner.apply(request)  # Cannot override explicit speed-control OFF from the other driving setting.
    store.put_bool('AlphaLongitudinalEnabled', True, block=True)
    assert owner.apply(request)


@pytest.mark.parametrize('identity', ('VOLKSWAGEN_ID4_MK1', 'VOLKSWAGEN_ID4_MK2'))
def test_temporary_eps_admission_keeps_exact_profile_and_physical_disarms(identity):
  from openpilot.starpilot.car.volkswagen.aol import temporary_eps_restriction
  cp = params(identity)
  policy = policy_for(cp)
  cp.alternativeExperience = 32
  cp.safetyConfigs[-1].safetyParam |= policy.safety_param_addition
  state = structs.CarState(canValid=True, steerFaultTemporary=True, gearShifter=structs.CarState.GearShifter.drive)
  state.cruiseState.available = True
  assert temporary_eps_restriction(cp, state)
  for field in ('steerFaultPermanent', 'brakePressed', 'gasPressed', 'regenBraking', 'doorOpen', 'seatbeltUnlatched',
                'accFaulted', 'vehicleSensorsInvalid', 'steeringDisengage', 'canTimeout'):
    setattr(state, field, True)
    assert not temporary_eps_restriction(cp, state), field
    setattr(state, field, False)
  for brand in ('honda', 'hyundai', 'gm', 'toyota', 'ford', 'subaru', 'chrysler', 'nissan', 'mazda', 'rivian', 'tesla'):
    cp.brand = brand
    assert not temporary_eps_restriction(cp, state), brand
  cp.brand = 'volkswagen'
  cp.alternativeExperience = 0
  assert not temporary_eps_restriction(cp, state)


@pytest.mark.parametrize('identity', ('VOLKSWAGEN_ID4_MK1', 'VOLKSWAGEN_ID4_MK2'))
@pytest.mark.parametrize('disarm', ('current_native', 'permanent', 'host_fault', 'main_off', 'can_invalid'))
def test_recovered_eps_ignores_only_prior_denial_and_never_a_current_disarm(identity, disarm):
  from openpilot.starpilot.aol.intent import AolSettings
  from openpilot.starpilot.car.volkswagen.aol import VolkswagenStockCardIntent
  cp = params(identity)
  policy = policy_for(cp)
  cp.alternativeExperience = 32
  cp.safetyConfigs[-1].safetyParam |= policy.safety_param_addition
  owner = VolkswagenStockCardIntent(cp, AolSettings(True, 5., 0, 0, (0, 0, 0), (0, 0, 0)))
  state = structs.CarState(canValid=True, vEgo=20., gearShifter=structs.CarState.GearShifter.drive)
  state.cruiseState.available = True
  owner.update(state, now_ns=1_000_000_000, fault_active=False)
  assert owner.allowed_latch
  state.steerFaultTemporary = True
  owner.update(state, now_ns=1_200_000_000, fault_active=False)
  assert owner.allowed_latch
  state.steerFaultTemporary = False
  owner.update(state, now_ns=1_250_000_000, native_rejection_ns=1_200_000_000, fault_active=False)
  assert owner.allowed_latch and owner._native_rejection_ns == 0
  rejection = 1_200_000_000
  if disarm == 'current_native':
    rejection = 1_260_000_000
  elif disarm == 'permanent':
    state.steerFaultPermanent = True
  elif disarm == 'main_off':
    state.cruiseState.available = False
  elif disarm == 'can_invalid':
    state.canValid = False
  owner.update(state, now_ns=1_260_000_000, native_rejection_ns=rejection, fault_active=disarm == 'host_fault')
  assert not owner.allowed_latch
