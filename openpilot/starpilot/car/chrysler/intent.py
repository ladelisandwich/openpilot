"""Full engagement or configured physical gestures arm lateral intent, never ACC."""

from opendbc.car import structs
from opendbc.car.chrysler.values import CAR
from opendbc.car.chrysler.aol_contract import lkas_button_supported
from openpilot.starpilot.aol.intent import AolCardIntent, AOL_TOGGLE, ButtonType


class ChryslerCardIntent(AolCardIntent):
  observe_stock_engagement = True
  requires_fault_observation = True

  def __init__(self, cp, settings):
    super().__init__(settings, explicit_latch=True, lkas_button_supported=lkas_button_supported(cp))
    self._requires_set = cp.carFingerprint == CAR.CHRYSLER_PACIFICA_2019_HYBRID
    self._set_seen = False
    self._main_cycle_required = False
    self._healthy_seen = False
    self._physical_held = {ButtonType.mainCruise: False, ButtonType.lkas: False}
    self._physical_neutral = {ButtonType.mainCruise: False, ButtonType.lkas: False}
    self._driver_eligible = False
    self._physical = None
    self.startup_calibration_pending = True

  def observe_physical_samples(self, physical, *, now_ns):
    self._physical = physical

  def _perform(self, action):
    if action == AOL_TOGGLE:
      if self.allowed_latch:
        self._main_cycle_required = True
        self._record_latch_clear('configured_toggle')
        self.allowed_latch = False
        self.pause_lateral = True
        self._withdraw_full_engagement()
      elif self.settings.enabled and self._driver_eligible and not self._fault_inhibit and (not self._requires_set or self._set_seen):
        self._main_cycle_required = False
        self.allowed_latch = True
        self.pause_lateral = False
    else:
      super()._perform(action)

  def _arm_from_full_engagement(self, CS, *, source_ns):
    self._set_seen = True
    self._main_cycle_required = False
    super()._arm_from_full_engagement(CS, source_ns=source_ns)

  def update(self, CS, **kwargs):
    self._driver_eligible = bool(
      CS.canValid
      and not CS.canTimeout
      and CS.cruiseState.available
      and not CS.steerFaultPermanent
      and not CS.accFaulted
      and CS.gearShifter in (structs.CarState.GearShifter.drive, structs.CarState.GearShifter.low)
    )
    if not CS.cruiseState.available or CS.steerFaultPermanent or kwargs.get('fault_active'):
      self._set_seen = False
    healthy = bool(CS.canValid and not CS.canTimeout)
    if healthy and not CS.cruiseState.available:
      self._main_cycle_required = False
    elif CS.steerFaultPermanent or CS.accFaulted or kwargs.get('fault_active') or kwargs.get('native_rejection_ns') or self._healthy_seen and not healthy:
      self._main_cycle_required = True
    if kwargs.get('fault_active') or CS.steerFaultPermanent or CS.accFaulted or kwargs.get('native_rejection_ns'):
      self.startup_calibration_pending = False
    if healthy:
      self._healthy_seen = True
    if any(event.type == ButtonType.cancel and event.pressed for event in CS.buttonEvents):
      self._main_cycle_required = True
    events = []
    for event in CS.buttonEvents:
      key = int(event.type.raw)
      if key not in self._physical_held:
        events.append(structs.CarState.ButtonEvent(**event.to_dict()))
        continue
      held = self._physical_held[key]
      if not event.pressed:
        self._physical_neutral[key] = True
      elif not held and self._physical_neutral[key] and self._driver_eligible:
        action = self.settings.main_action if key == ButtonType.mainCruise else self.lkas_action
        self._perform(action)
      self._physical_held[key] = bool(event.pressed)
    if CS.canValid and not CS.canTimeout:
      for key in self._physical_neutral:
        if not self._physical_held[key]:
          self._physical_neutral[key] = True
    else:
      self._physical_neutral = dict.fromkeys(self._physical_neutral, False)
    self._engagement_manual_withdrawal = any(self._physical_held.values())
    filtered = structs.CarState(**CS.to_dict())
    filtered.buttonEvents = events
    super().update(filtered.as_reader(), **kwargs)
    main_follow = self.settings.lkas_action != AOL_TOGGLE and self.settings.main_action != AOL_TOGGLE
    if main_follow and not self._requires_set and self.settings.enabled and self._driver_eligible and not self._main_cycle_required and not self._fault_inhibit:
      self.allowed_latch = True
    if self._physical is not None:
      self._physical.aol_pause_longitudinal = bool(self.pause_longitudinal and CS.canValid and not CS.canTimeout)
      self._physical.aol_pause_longitudinal_stamp_ns = kwargs.get('now_ns', 0)
    if not CS.cruiseState.available:
      self._record_latch_clear('cruise_main_off')
      self.allowed_latch = False
      self._withdraw_full_engagement()

  def output(self, CS):
    allowed, lateral, longitudinal = super().output(CS)
    return bool(allowed and CS.cruiseState.available and not CS.canTimeout), lateral, longitudinal

  def observe_calibration(self, CS, *, events=None, standard_enabled=False):
    from openpilot.cereal import log
    # Cold readiness must not manufacture a revoked MAIN epoch. Actual SD
    # calibration/output gates remain authoritative; only cold intent survives.
    if events is not None and CS.canValid and not CS.canTimeout:
      pending = (log.OnroadEvent.EventName.selfdriveInitializing,
                 log.OnroadEvent.EventName.calibrationIncomplete)
      if not any(event.name in pending for event in events):
        self.startup_calibration_pending = False
    super().observe_calibration(CS, events=events, standard_enabled=standard_enabled)
