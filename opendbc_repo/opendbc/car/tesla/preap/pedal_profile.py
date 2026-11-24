"""Finite pedal profiles through the existing preAP safety model39."""
import math
from opendbc.car import structs
from opendbc.car.tesla.values import CAR
from opendbc.car.tesla.preap.constants import ACCEL_PREAP_BP, ACCEL_PREAP_PROFILES


def qualified(cp):
  try:
    word = int(cp.safetyConfigs[0].safetyParam)
    return bool(cp.brand == 'tesla' and cp.carFingerprint == CAR.TESLA_MODEL_S_PREAP and
      cp.flags == 0 and not cp.passive and not cp.notCar and not cp.dashcamOnly and
      cp.openpilotLongitudinalControl and not cp.pcmCruise and not cp.alphaLongitudinalAvailable and
      cp.transmissionType != structs.CarParams.TransmissionType.manual and
      cp.fingerprintSource != structs.CarParams.FingerprintSource.fixed and len(cp.safetyConfigs) == 1 and
      cp.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.teslaPreap and word in (1, 3, 7) and
      cp.radarUnavailable == (word == 1) and cp.steerControlType == structs.CarParams.SteerControlType.angle and
      cp.steerAtStandstill and math.isfinite(cp.wheelbase) and math.isclose(cp.wheelbase, 2.96, rel_tol=0., abs_tol=1e-6) and
      cp.steerRatio == 15. and cp.alternativeExperience in (0, 32))
  except (AttributeError, TypeError, ValueError, IndexError, OverflowError):
    return False


def accel_max(speed, personality):
  values = ACCEL_PREAP_PROFILES.get(personality, ACCEL_PREAP_PROFILES[1])
  if speed <= ACCEL_PREAP_BP[0]:
    return values[0]
  for i in range(1, len(ACCEL_PREAP_BP)):
    if speed <= ACCEL_PREAP_BP[i]:
      r = (speed - ACCEL_PREAP_BP[i - 1]) / (ACCEL_PREAP_BP[i] - ACCEL_PREAP_BP[i - 1])
      return values[i - 1] + r * (values[i] - values[i - 1])
  return values[-1]


def accel_limits(cp, speed, personality, existing):
  if not qualified(cp) or not math.isfinite(speed):
    return existing
  # Current upstream/native limits always win if narrower than the original tune.
  return max(existing[0], -1.5), min(existing[1], accel_max(speed, personality))
