"""Rivian BASE GEN1: successful ordinary engagement owns lateral intent."""
from opendbc.car import structs
from opendbc.car.rivian.stock_aol import qualified, temporary_restriction
from openpilot.starpilot.aol.policy import AolVehiclePolicy
from openpilot.starpilot.aol.intent import AolCardIntent, AOL_TOGGLE, ButtonType
from openpilot.selfdrive.car.cruise import CRUISE_LONG_PRESS


def policy_for(cp):
  admitted = qualified(cp)
  return AolVehiclePolicy(intent_supported=admitted, settings_supported=admitted, runtime_supported=admitted,
    normal_runtime_supported=admitted, ordinary_axis_ack_required=qualified(cp, marked_only=True),
    explicit_latch=admitted, lkas_button_supported=False, alternative_experience_addition=32 if admitted else 0)


def native_profile_supported(model, param):
  return model == int(structs.CarParams.SafetyModel.rivian) and param in (0, 1)


def native_accepts_cp(cp, model, param):
  return qualified(cp, marked_only=True) and native_profile_supported(model, param) and param == cp.safetyConfigs[0].safetyParam


def native_latch_rejected(cp, native):
  return qualified(cp, marked_only=True) and native.requestedLateral and not native.lateralAllowed


def retain_on_native_denial(cp, native, state):
  return temporary_restriction(cp, state)


def ordinary_axis_request_allowed(cp, state):
  return qualified(cp, marked_only=True) and state.canValid and not state.canTimeout and not state.steerFaultPermanent


class RivianStockCardIntent(AolCardIntent):
  observe_stock_engagement = True

  def __init__(self, settings):
    super().__init__(settings, explicit_latch=True, lkas_button_supported=False)
    self.cancel_held = False
    self.cancel_ticks = 0

  def _cancel_action(self, action):
    if action == AOL_TOGGLE and self.settings.enabled:
      if self.allowed_latch:
        self._record_latch_clear('configured_cancel_toggle')
      self.allowed_latch = not self.allowed_latch
      self.pause_lateral = not self.allowed_latch
      if not self.allowed_latch:
        self._withdraw_full_engagement()
    else:
      self._perform(action)

  def update(self, CS, **kwargs):
    # The stalk UP_1 producer owns these actions. No availability/Main edge
    # is converted into a button, and no received cancel echo can arm a session.
    state = structs.CarState(**CS.to_dict())
    state.buttonEvents = [structs.CarState.ButtonEvent(**event.to_dict()) for event in CS.buttonEvents
                          if event.type != ButtonType.cancel]
    for event in CS.buttonEvents:
      if event.type == ButtonType.cancel:
        if event.pressed:
          self.cancel_held, self.cancel_ticks = True, 0
          if not any(self.settings.cancel_actions):
            self._record_latch_clear('physical_cancel')
            self.allowed_latch = False
            self._withdraw_full_engagement()
        elif self.cancel_held:
          if 0 < self.cancel_ticks < CRUISE_LONG_PRESS:
            self._cancel_action(self.settings.cancel_actions[0])
          self.cancel_held, self.cancel_ticks = False, 0
    if not CS.canValid or CS.canTimeout:
      self.cancel_held, self.cancel_ticks = False, 0
    super().update(state.as_reader(), **kwargs)
    if self.cancel_held:
      self.cancel_ticks += 1
      if self.cancel_ticks == CRUISE_LONG_PRESS:
        self._cancel_action(self.settings.cancel_actions[1])
      elif self.cancel_ticks == CRUISE_LONG_PRESS * 5:
        self._cancel_action(self.settings.cancel_actions[2])



def create_intent(cp, settings):
  return RivianStockCardIntent(settings)


def retained_brake_lateral_intent(cp, cs, sm, feedback, owner, *, now_ns, producer):
  """Join one native-first FULL -> LAT brake occurrence; preserve intent only."""
  from openpilot.cereal import log
  from openpilot.starpilot.aol.runtime import AXIS_MAX_AGE_NS, SAFETY_MAX_AGE_NS
  from openpilot.starpilot.aol.transport_pause import cp_identity, independent_lateral_health
  try:
    if (not qualified(cp, marked_only=True) or not cp.openpilotLongitudinalControl or
        owner is None or not owner.allowed_latch or not owner.settings.enabled or owner.pause_lateral or
        feedback.cp != cp_identity(cp) or feedback.failed or feedback.paused or feedback.pending_event or
        feedback.retention_started_ns or feedback.source_retention_started_ns or
        not feedback.session or not producer or producer != feedback.availability_producer or
        not cs.canValid or cs.canTimeout or not cs.brakePressed or cs.gasPressed or cs.regenBraking or
        not cs.cruiseState.available or cs.steerFaultTemporary or cs.steerFaultPermanent or
        cs.accFaulted or cs.doorOpen or cs.seatbeltUnlatched or cs.steeringDisengage or cs.buttonEvents or
        cs.gearShifter != structs.CarState.GearShifter.drive):
      return False
    axis = sm['aolAxisState']
    stamp = int(sm.logMonoTime['aolAxisState'])
    previous = feedback.signature
    if (not sm.seen['aolAxisState'] or not sm.valid['aolAxisState'] or not sm.alive['aolAxisState'] or
        not axis.qualified or axis.sessionId != feedback.session or axis.faultSessionId != feedback.session or
        str(axis.faultReason) != 'none' or str(axis.mode) != 'combined' or not axis.nativeAcknowledged or
        not axis.lateralActive or not axis.longitudinalActive or not axis.desiredLateral or not axis.desiredLongitudinal or
        axis.intentProducerSessionId != producer or axis.intentIntegrityEpoch != feedback.availability_epoch or
        not 0 < axis.sourceCarStateMonoTime <= stamp == axis.observedMonoTime < now_ns <= axis.validUntilMonoTime or
        now_ns - axis.sourceCarStateMonoTime > AXIS_MAX_AGE_NS or
        axis.validUntilMonoTime - stamp > AXIS_MAX_AGE_NS or
        previous is None or previous[0] != feedback.session or previous[5] != 'none' or previous[7] != feedback.session or
        not all(previous[index] for index in (9, 10, 11, 12, 13)) or
        previous[14] != producer or previous[15] != feedback.availability_epoch or
        axis.sequence < feedback.sequence or
        not 0 < feedback.healthy_feedback_ns <= now_ns or now_ns - feedback.healthy_feedback_ns > AXIS_MAX_AGE_NS):
      return False
    native = independent_lateral_health(sm, cp, cs, feedback.session, now_ns)
    if (native is None or not native.pandaInventory or not native.requestedLongitudinal or native.longitudinalAllowed or
        native.observedMonoTime < stamp or now_ns - native.observedMonoTime > SAFETY_MAX_AGE_NS or
        (native.pandaSerial, tuple(p.hardwareSerial for p in native.pandaInventory)) != feedback.healthy_inventory or
        not all(not p.controlsAllowed for p in native.pandaInventory)):
      return False
    event_ns = int(sm.logMonoTime['onroadEvents'])
    if (not sm.seen['onroadEvents'] or not sm.valid['onroadEvents'] or not sm.alive['onroadEvents'] or
        not 0 < event_ns <= now_ns or now_ns - event_ns > 1_500_000_000):
      return False
    return all(not e.immediateDisable and not e.softDisable and
      (not e.userDisable or e.name in (log.OnroadEvent.EventName.pedalPressed, log.OnroadEvent.EventName.pcmDisable))
      for e in sm['onroadEvents'])
  except (AttributeError, KeyError, IndexError, TypeError, ValueError, OverflowError):
    return False
