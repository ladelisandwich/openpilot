"""Nissan stock-cruise AOL; optional transport never owns longitudinal actuation."""

from opendbc.car import structs
from opendbc.car.nissan.values import CAR
from openpilot.starpilot.aol.intent import AolCardIntent, AOL_TOGGLE, ButtonType
from openpilot.starpilot.aol.policy import AolVehiclePolicy

# Bit 1 belonged to Dom's disabled longitudinal experiment; never reuse it.
NISSAN_AOL_LEAF = 4


def qualified(cp, *, marked_only=False):
  try:
    if (
      cp.brand != 'nissan'
      or cp.carFingerprint not in tuple(CAR)
      or cp.flags != 0
      or cp.passive
      or cp.notCar
      or cp.dashcamOnly
      or not cp.pcmCruise
      or cp.openpilotLongitudinalControl
      or cp.steerControlType != structs.CarParams.SteerControlType.angle
      or len(cp.safetyConfigs) != 1
      or cp.alternativeExperience not in ((32,) if marked_only else (0, 32))
    ):
      return False
    base = 1 if cp.carFingerprint == CAR.NISSAN_ALTIMA else 0
    leaf = cp.carFingerprint in (CAR.NISSAN_LEAF, CAR.NISSAN_LEAF_IC)
    expected = base | (NISSAN_AOL_LEAF if leaf and cp.alternativeExperience == 32 else 0)
    config = cp.safetyConfigs[0]
    return config.safetyModel == structs.CarParams.SafetyModel.nissan and config.safetyParam == expected
  except (AttributeError, TypeError, ValueError, IndexError):
    return False


def policy_for(cp):
  admitted = qualified(cp)
  leaf = admitted and cp.carFingerprint in (CAR.NISSAN_LEAF, CAR.NISSAN_LEAF_IC)
  return AolVehiclePolicy(
    intent_supported=admitted,
    settings_supported=admitted,
    runtime_supported=admitted,
    normal_runtime_supported=admitted,
    ordinary_axis_ack_required=qualified(cp, marked_only=True),
    explicit_latch=admitted,
    alternative_experience_addition=32 if admitted else 0,
    safety_param_addition=NISSAN_AOL_LEAF if leaf else 0,
  )


def native_profile_supported(model, param):
  return model == int(structs.CarParams.SafetyModel.nissan) and param in (0, 1, NISSAN_AOL_LEAF)


def native_accepts_cp(cp, model, param):
  return qualified(cp, marked_only=True) and native_profile_supported(model, param) and param == cp.safetyConfigs[0].safetyParam


def ordinary_axis_request_allowed(cp, state):
  return qualified(cp, marked_only=True) and state.canValid and not state.canTimeout and not state.steerFaultPermanent


def temporary_restriction(cp, state):
  gears = structs.CarState.GearShifter
  return bool(
    state is not None
    and qualified(cp, marked_only=True)
    and state.canValid
    and not state.canTimeout
    and not state.steerFaultPermanent
    and (state.gearShifter not in (gears.drive, gears.low, gears.brake) or state.steerFaultTemporary or state.vehicleSensorsInvalid or state.invalidLkasSetting)
  )


def native_latch_rejected(cp, native):
  return qualified(cp, marked_only=True) and native.requestedLateral and not native.lateralAllowed


def retain_on_native_denial(cp, native, state):
  return temporary_restriction(cp, state)


def native_observation(native, *, latched, panda_ready, restricted, now_ns, pending_since_ns):
  lost = not panda_ready or native is None
  if not latched or lost or restricted:
    return 0, lost, False
  if native.requestedLateral and native.lateralAllowed:
    return 0, False, False
  pending_since_ns = pending_since_ns or now_ns
  return pending_since_ns, False, now_ns - pending_since_ns >= 200_000_000


class NissanCardIntent(AolCardIntent):
  observe_stock_engagement = True

  def __init__(self, cp, settings):
    self.main_seen = False
    self.main_available = False
    self.main_pulse = False
    self.needs_rearm = False
    super().__init__(settings, explicit_latch=True)

  def _perform(self, action):
    if action == AOL_TOGGLE and self.settings.enabled:
      self.allowed_latch = not self.allowed_latch
      self.needs_rearm = not self.allowed_latch
      if self.allowed_latch:
        self.pause_lateral = False
      else:
        self._record_latch_clear('configured_toggle')
        self._withdraw_full_engagement()
    else:
      super()._perform(action)

  def _arm_from_full_engagement(self, CS, *, source_ns):
    super()._arm_from_full_engagement(CS, source_ns=source_ns)
    self.needs_rearm = False

  def update(self, CS, **kwargs):
    saved = self.settings
    automatic_main = saved.main_action == 0 and saved.lkas_action == 0
    state = structs.CarState(**CS.to_dict())
    healthy = bool(
      CS.canValid
      and not CS.canTimeout
      and not CS.steerFaultPermanent
      and not CS.accFaulted
      and kwargs.get('fault_active') is not True
      and not kwargs.get('native_rejection_ns', 0)
    )
    was_latched = self.allowed_latch
    if not healthy and (was_latched or CS.steerFaultPermanent or CS.accFaulted):
      self.needs_rearm = True
    available = bool(CS.cruiseState.available)
    if healthy and not available:
      self.needs_rearm = False
    events = []
    for event in CS.buttonEvents:
      if event.type == ButtonType.lkas and saved.lkas_action != AOL_TOGGLE:
        if event.pressed and healthy:
          self._perform(saved.lkas_action)
      else:
        events.append(structs.CarState.ButtonEvent(**event.to_dict()))
    if self.main_pulse:
      events.append(structs.CarState.ButtonEvent(type=ButtonType.mainCruise, pressed=False))
      self.main_pulse = False
    if healthy:
      if self.main_seen and available and not self.main_available:
        if saved.main_action == AOL_TOGGLE:
          events.append(structs.CarState.ButtonEvent(type=ButtonType.mainCruise, pressed=True))
          self.main_pulse = True
        elif saved.main_action:
          self._perform(saved.main_action)
      self.main_seen = True
      self.main_available = available
    else:
      self.main_seen = False
    state.buttonEvents = events
    super().update(state.as_reader(), **kwargs)
    if was_latched and not self.allowed_latch:
      self.needs_rearm = True
    if not available:
      self.allowed_latch = False
    elif (
      automatic_main
      and healthy
      and saved.enabled
      and not self.needs_rearm
      and not self.pause_lateral
      and not self._fault_inhibit
      and not self._calibration_inhibit
      and not self._calibration_rearm_required
    ):
      # Dom follows real cruise MAIN, including boot with MAIN already on.
      # This is intent only: current native/vehicle permission still gates CAN.
      self.allowed_latch = True


def create_intent(cp, settings):
  return NissanCardIntent(cp, settings)


def temporary_eps_restriction(cp, state):
  gears = structs.CarState.GearShifter
  return bool(qualified(cp, marked_only=True) and state.canValid and not state.canTimeout and
    state.steerFaultTemporary and not state.steerFaultPermanent and not state.accFaulted and
    state.cruiseState.available and not state.cruiseState.enabled and
    state.gearShifter in (gears.drive, gears.low, gears.brake) and
    not state.brakePressed and not state.gasPressed and not state.regenBraking and
    not state.doorOpen and not state.seatbeltUnlatched and not state.invalidLkasSetting)
