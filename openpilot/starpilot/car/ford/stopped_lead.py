"""Dom's stopped-lead cap on a rolling Mach-E approach; no launch ownership."""
import math

from opendbc.car.ford.aol import qualified
from opendbc.car.ford.values import CAR

SOURCES = ('carState', 'radarState', 'modelV2', 'carControl', 'controlsState', 'selfdriveState')
MAX_AGE_NS = 250_000_000
MAX_EGO_SPEED = .5
MAX_LEAD_SPEED = .45
MAX_LEAD_DELTA = .5
MIN_MODEL_PROB = .95
MAX_LATERAL_OFFSET = 1.75
MIN_DISTANCE = 3.
DISTANCE_MARGIN = 3.
MIN_BRAKE = .16
MAX_BRAKE = .26


def owner_for(cp, stop_distance):
  try:
    if (cp.brand == 'ford' and cp.carFingerprint == CAR.FORD_MUSTANG_MACH_E_MK1 and
        cp.openpilotLongitudinalControl and qualified(cp)):
      return RollingStoppedLead(stop_distance)
  except (AttributeError, TypeError, ValueError):
    pass
  return None


class RollingStoppedLead:
  def __init__(self, stop_distance):
    self.stop_distance = stop_distance
    self.reset()

  def reset(self):
    self.drive_id = 0
    self.rolling = False

  def cap(self, lead, speed, accel_min):
    if not lead.present or not lead.radar and lead.modelProb < MIN_MODEL_PROB or abs(lead.yRel) > MAX_LATERAL_OFFSET:
      return None
    lead_speed = max(float(lead.vLead), 0.)
    delta = lead_speed - speed
    distance = max(MIN_DISTANCE, self.stop_distance + DISTANCE_MARGIN)
    if speed > MAX_EGO_SPEED or lead.dRel > distance or lead_speed > MAX_LEAD_SPEED or delta > MAX_LEAD_DELTA:
      return None
    distance_factor = min(max((distance - lead.dRel) / max(distance - MIN_DISTANCE, .1), 0.), 1.)
    speed_factor = min(max(lead_speed / max(MAX_LEAD_SPEED, .1), 0.), 1.)
    delta_factor = min(max((MAX_LEAD_DELTA - delta) / max(MAX_LEAD_DELTA, .1), 0.), 1.)
    brake = min(max(MIN_BRAKE + .06 * distance_factor + .02 * speed_factor + .02 * delta_factor, MIN_BRAKE), MAX_BRAKE)
    return -brake if accel_min >= 0. else max(accel_min, -brake)

  def sample(self, sm, *, now_ns, drive_id, target, stopping, accel_min, active):
    try:
      car = sm['carState']
      if (type(now_ns) is not int or type(drive_id) is not int or not 0 < drive_id < now_ns or not active or
          not all(sm.valid[name] and sm.alive[name] and drive_id < sm.logMonoTime[name] <= now_ns and
                  now_ns - sm.logMonoTime[name] <= MAX_AGE_NS for name in SOURCES) or
          not car.canValid or car.canTimeout or car.accFaulted or car.gasPressed or car.brakePressed or
          str(car.gearShifter) not in ('drive', 'low', 'sport') or
          not sm['carControl'].longActive or not sm['selfdriveState'].enabled or sm['controlsState'].forceDecel):
        self.reset()
        return target, stopping
      if self.drive_id != drive_id:
        self.reset()
        self.drive_id = drive_id
      speed = float(car.vEgo)
      leads = (sm['radarState'].leadOne, sm['radarState'].leadTwo)
      if (not all(math.isfinite(float(v)) for v in (speed, target, accel_min, self.stop_distance)) or
          speed < 0. or self.stop_distance < 0. or accel_min > 0. or any(
            not all(math.isfinite(float(getattr(lead, field))) for field in ('dRel', 'vLead', 'yRel', 'modelProb')) or
            lead.dRel < 0. or not 0. <= lead.modelProb <= 1. for lead in leads if lead.present)):
        self.reset()
        return target, stopping
      if car.standstill:
        self.rolling = False
      elif speed > MAX_EGO_SPEED:
        self.rolling = True
      # The original stopped-car departure owner remains responsible after an
      # actual standstill. Missing history cannot invent a new rolling approach.
      if not self.rolling or speed > MAX_EGO_SPEED:
        return target, stopping
      caps = [value for lead in leads if (value := self.cap(lead, speed, accel_min)) is not None]
      return (min(target, min(caps)), True) if caps else (target, stopping)
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
      self.reset()
      return target, stopping
