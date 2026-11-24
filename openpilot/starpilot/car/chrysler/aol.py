"""Stock Chrysler AOL owns intent; current native axis acknowledgment owns commands."""
from opendbc.car import structs
from opendbc.car.chrysler.aol_contract import qualified, temporary_restriction, lkas_button_supported
from opendbc.car.chrysler.values import CAR, ChryslerSafetyFlags
from openpilot.starpilot.aol.policy import AolVehiclePolicy
from openpilot.starpilot.car.chrysler.intent import ChryslerCardIntent


def policy_for(cp):
  admitted = qualified(cp)
  marked = qualified(cp, marked_only=True)
  return AolVehiclePolicy(intent_supported=admitted, settings_supported=admitted, runtime_supported=admitted,
                          normal_runtime_supported=admitted, ordinary_axis_ack_required=marked,
                          full_axis_runtime_required=marked, explicit_latch=admitted,
                          lkas_button_supported=admitted and lkas_button_supported(cp),
                          safety_param_addition=int(ChryslerSafetyFlags.AOL_PACIFICA_HYBRID_SET_GUARD)
                            if admitted and cp.carFingerprint == CAR.CHRYSLER_PACIFICA_2019_HYBRID else 0,
                          alternative_experience_addition=32 if admitted else 0)


def native_profile_supported(model, param):
  return model == int(structs.CarParams.SafetyModel.chrysler) and param in (0, 1, 4, 8)


def native_accepts_cp(cp, model, param):
  return qualified(cp, marked_only=True) and native_profile_supported(model, param)


def ordinary_axis_request_allowed(cp, state):
  return qualified(cp, marked_only=True) and state.canValid and not state.canTimeout and not state.steerFaultPermanent


def native_latch_rejected(cp, native):
  return qualified(cp, marked_only=True) and native.requestedLateral and not native.lateralAllowed


def retain_on_native_denial(cp, native, state):
  return temporary_restriction(cp, state)


def create_intent(cp, settings):
  return ChryslerCardIntent(cp, settings)


def native_bootstrap_supported(cp):
  return qualified(cp, marked_only=True)


def intent_disarming_fault(cp, events, state, **context):
  from openpilot.cereal import log
  from openpilot.starpilot.aol.intent import disarming_fault
  cold = context.pop('startup_calibration_pending', False)
  if qualified(cp, marked_only=True) and cold and not state.steerFaultPermanent:
    events = [event for event in events if not (
      event.name == log.OnroadEvent.EventName.calibrationIncomplete and not event.immediateDisable)]
  return disarming_fault(events, state, **context)


def cold_calibration_intent_current(feedback, sm, cp, cs, axis, native, now_ns, *, pending, producer):
  """Only initial calibration wait; never a command or renewed native grant."""
  from openpilot.cereal import log
  from openpilot.starpilot.aol.runtime import SAFETY_MAX_AGE_NS
  if (not qualified(cp, marked_only=True) or feedback.failed or feedback.ever_lateral_active or
      not producer or axis.intentProducerSessionId != producer or
      feedback.signature is not None and
        (axis.intentProducerSessionId, axis.intentIntegrityEpoch) != feedback.signature[14:16] or
      not cs.cruiseState.available or cs.cruiseState.enabled or cs.accFaulted or cs.doorOpen or
      cs.seatbeltUnlatched or cs.steeringDisengage or cs.gearShifter not in
        (structs.CarState.GearShifter.drive, structs.CarState.GearShifter.low) or
      native is None or not native.pandaInventory or
      native.requestedLongitudinal or native.longitudinalAllowed or
      axis.desiredLongitudinal or axis.longitudinalActive):
    return False
  if axis.lateralActive:
    if axis.desiredLateral and native.requestedLateral and native.lateralAllowed:
      feedback.cold_main_wait_ns = 0
    return False
  event_ns = int(sm.logMonoTime['onroadEvents'])
  if (not sm.seen['onroadEvents'] or not sm.valid['onroadEvents'] or not sm.alive['onroadEvents'] or
      not 0 < event_ns <= now_ns or now_ns - event_ns > 1_500_000_000):
    return False
  events = sm['onroadEvents']
  cal = log.OnroadEvent.EventName.calibrationIncomplete
  pcm = log.OnroadEvent.EventName.pcmDisable
  if any(e.immediateDisable or e.softDisable and e.name != cal or e.userDisable and e.name != pcm for e in events):
    return False
  cold = any(e.name == cal and e.softDisable and not e.immediateDisable for e in events)
  if cold:
    if (not pending or str(axis.faultReason) not in ('critical', 'availabilityPause') or not 0 < axis.faultEventMonoTime <= event_ns or
        axis.desiredLateral or native.requestedLateral or native.lateralAllowed):
      return False
    feedback.cold_main_wait_ns = int(axis.observedMonoTime)
    return True
  # The old OFF response may precede the ready event. Its original transport
  # and native leases are validated by the caller; this bounded join arms none.
  return bool(0 < feedback.cold_main_wait_ns <= now_ns <= feedback.cold_main_wait_ns + SAFETY_MAX_AGE_NS and
              str(axis.faultReason) in ('critical', 'availabilityPause', 'none') and
              (not native.requestedLateral and not native.lateralAllowed or
               str(axis.faultReason) == 'none' and axis.desiredLateral and
               native.requestedLateral and native.lateralAllowed))
