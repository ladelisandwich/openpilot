"""Pre-AP optional close-lead acceleration cap from Dom's intentional planner."""
import math

from opendbc.car import structs
from opendbc.car.tesla.values import CAR

SOURCE_MAX_AGE_NS = 250_000_000
SOURCES = ('carState', 'carControl', 'controlsState', 'selfdriveState', 'radarState', 'modelV2')
SPEED_BP = (0.0, 1.3, 7.5, 15.0, 25.0, 40.0)
FOLLOW_ACCEL = (0.35, 0.55, 0.80, 0.75, 0.65, 0.50)


def profile_key(cp):
  try:
    if (cp.brand != 'tesla' or cp.carFingerprint != CAR.TESLA_MODEL_S_PREAP or cp.passive or cp.notCar or
        cp.dashcamOnly or not cp.openpilotLongitudinalControl or cp.pcmCruise or cp.alphaLongitudinalAvailable or
        cp.steerControlType != structs.CarParams.SteerControlType.angle or len(cp.safetyConfigs) != 1 or
        cp.transmissionType == structs.CarParams.TransmissionType.manual or
        cp.fingerprintSource == structs.CarParams.FingerprintSource.fixed):
      return None
    if (not math.isfinite(cp.wheelbase) or not math.isclose(cp.wheelbase, 2.96, rel_tol=0., abs_tol=1e-6) or
        cp.steerRatio != 15.):
      return None
    cfg = cp.safetyConfigs[0]
    if cfg.safetyModel != structs.CarParams.SafetyModel.teslaPreap or cfg.safetyParam not in (1, 3, 7):
      return None
    return (cp.brand, str(cp.carFingerprint), int(cfg.safetyParam), float(cp.wheelbase), float(cp.steerRatio))
  except (AttributeError, IndexError, ValueError, TypeError, OverflowError):
    return None


def follow_limit(speed):
  if speed <= SPEED_BP[0]:
    return FOLLOW_ACCEL[0]
  for index in range(1, len(SPEED_BP)):
    if speed <= SPEED_BP[index]:
      fraction = (speed - SPEED_BP[index - 1]) / (SPEED_BP[index] - SPEED_BP[index - 1])
      return FOLLOW_ACCEL[index - 1] + fraction * (FOLLOW_ACCEL[index] - FOLLOW_ACCEL[index - 1])
  return FOLLOW_ACCEL[-1]


class PreapAdaptiveAccel:
  def __init__(self, cp, enabled):
    self.cp = cp
    self.key = profile_key(cp)
    self.enabled = type(enabled) is bool and enabled and self.key is not None

  def sample(self, sm, *, now_ns, drive_id, upper, follow_seconds, active):
    try:
      if not self.enabled or not active or profile_key(self.cp) != self.key:
        return None
      if (type(now_ns) is not int or type(drive_id) is not int or not 0 < drive_id < now_ns or
          not all(sm.valid[name] and sm.alive[name] and drive_id <= sm.logMonoTime[name] <= now_ns and
                  now_ns - sm.logMonoTime[name] <= SOURCE_MAX_AGE_NS for name in SOURCES)):
        return None
      car, lead = sm['carState'], sm['radarState'].leadOne
      if (not car.canValid or car.canTimeout or car.gasPressed or car.brakePressed or
          car.gearShifter != structs.CarState.GearShifter.drive or car.doorOpen or car.steerFaultPermanent or
          sm['controlsState'].forceDecel or not sm['selfdriveState'].enabled or not sm['carControl'].longActive or
          not lead.present):
        return None
      speed, distance, upper, follow = float(car.vEgo), float(lead.dRel), float(upper), float(follow_seconds)
      if (not all(math.isfinite(v) for v in (speed, distance, upper, follow, float(lead.vLead))) or
          speed < 0 or distance <= 0 or upper < 0 or not .5 <= follow <= 3.):
        return None
      # Original get_safe_obstacle_distance uses COMFORT_BRAKE2.5/STOP_DISTANCE6.
      safe_distance = speed * speed / 5.0 + follow * speed + 6.0
      strength = max(0.0, min(1.0, 1.0 - (distance / max(safe_distance, 1.0) - 1.2) / 0.3))
      if strength <= 0:
        return None
      return min(upper, upper * (1.0 - strength) + follow_limit(speed) * strength)
    except (AttributeError, KeyError, IndexError, TypeError, ValueError, OverflowError):
      return None


def startup_owner(cp):
  if profile_key(cp) is None:
    return None
  from openpilot.common.params import Params
  from openpilot.starpilot.saved_source import read_saved
  # Freeze the complete saved bool once; no live toggle read inside model updates.
  try:
    raw, valid = read_saved(Params(), 'NAPAdaptiveAccel', 8)
    enabled = valid and raw in (None, b'1')
  except (OSError, AttributeError, TypeError, ValueError):
    enabled = False
  return PreapAdaptiveAccel(cp, enabled)
