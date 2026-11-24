import math

from opendbc.car import structs
from opendbc.car.tesla.values import CAR


def _stock_qualified(cp, *, marked_only=False):
  try:
    return bool(cp.brand == "tesla" and cp.carFingerprint == CAR.TESLA_MODEL_S_PREAP and cp.flags == 0 and
      not cp.passive and not cp.notCar and not cp.dashcamOnly and ((cp.safetyConfigs[0].safetyParam == 0 and cp.radarUnavailable) or
       (cp.safetyConfigs[0].safetyParam in (2, 6) and not cp.radarUnavailable)) and
      cp.transmissionType != structs.CarParams.TransmissionType.manual and
      cp.fingerprintSource != structs.CarParams.FingerprintSource.fixed and
      cp.pcmCruise and not cp.openpilotLongitudinalControl and not cp.alphaLongitudinalAvailable and
      cp.steerControlType == structs.CarParams.SteerControlType.angle and cp.steerAtStandstill and
      math.isfinite(cp.wheelbase) and math.isclose(cp.wheelbase, 2.96, rel_tol=0.0, abs_tol=1e-6) and cp.steerRatio == 15.0 and len(cp.safetyConfigs) == 1 and
      cp.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.teslaPreap and
      cp.safetyConfigs[0].safetyParam in (0, 2, 6) and cp.alternativeExperience in ((32,) if marked_only else (0, 32)))
  except (AttributeError, TypeError, ValueError, OverflowError, IndexError):
    return False


def qualified(cp, *, marked_only=False):
  from opendbc.car.tesla.preap.pedal_profile import qualified as pedal_qualified
  return _stock_qualified(cp, marked_only=marked_only) or (pedal_qualified(cp) and (not marked_only or cp.alternativeExperience == 32))


def temporary_restriction(cp, state):
  return bool(qualified(cp, marked_only=True) and state is not None and state.canValid and not state.canTimeout and
    not state.steerFaultPermanent and not state.steeringDisengage and not state.doorOpen and not state.accFaulted and
    (state.gearShifter in (structs.CarState.GearShifter.reverse, structs.CarState.GearShifter.neutral) or
     state.gearShifter == structs.CarState.GearShifter.drive and
     (state.brakePressed or state.steerFaultTemporary or state.standstill)))


def native_observation(native, *, latched, panda_ready, restricted, now_ns, pending_since_ns, pending_cancel_ns=0):
  lost = not panda_ready or native is None
  if (latched and not lost and pending_cancel_ns and native.observedMonoTime >= pending_cancel_ns and
      not native.lateralAllowed):
    return 0, False, True
  if not latched or lost or restricted:
    return 0, lost, False
  if native.requestedLateral and native.lateralAllowed:
    return 0, False, False
  pending_since_ns = pending_since_ns or now_ns
  return pending_since_ns, False, now_ns - pending_since_ns >= 200_000_000
