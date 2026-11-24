"""Dom's existing low-speed vision-lead stop buffer for active HKG longitudinal."""
import math

from opendbc.car.hyundai.values import CAR

SOURCES = ('modelV2', 'carState', 'radarState', 'carControl', 'controlsState', 'selfdriveState')
MAX_SOURCE_AGE_NS = 250_000_000
MAX_MODEL_GAP_NS = 150_000_000
MAX_EGO_SPEED = 6.5
MAX_LEAD_SPEED = 3.25
HOLD_MAX_LEAD_SPEED = 4.0
MIN_MODEL_PROB = .9
MIN_CLOSING_SPEED = .35
MIN_HOLD_REL_SPEED = -.2
BASE_BUFFER = 3.8
EGO_GAIN = .8
LEAD_GAIN = .25
RELEASE_MARGIN = .9
HOLD_NS = 800_000_000
MIN_BRAKE = 1.25
BRAKE_GAIN = .25


def owner_for(cp):
  try:
    if (cp.brand == 'hyundai' and cp.carFingerprint in CAR.__members__ and cp.openpilotLongitudinalControl and
        not cp.passive and not cp.dashcamOnly and not cp.notCar):
      return VisionStopBuffer()
  except (AttributeError, TypeError):
    pass
  return None


class VisionStopBuffer:
  def __init__(self):
    self.reset()

  def reset(self):
    self.drive_id = 0
    self.last_model_ns = 0
    self.hold_until_ns = 0

  def cap(self, lead, speed, accel_min, now_ns):
    if not lead.present or lead.radar or lead.modelProb < MIN_MODEL_PROB:
      return None, False
    lead_speed = max(float(lead.vLead), 0.)
    relative_speed = speed - lead_speed
    entry = speed <= MAX_EGO_SPEED and lead_speed <= MAX_LEAD_SPEED and max(0., relative_speed) >= MIN_CLOSING_SPEED
    hold = speed <= MAX_EGO_SPEED and lead_speed <= HOLD_MAX_LEAD_SPEED and relative_speed >= MIN_HOLD_REL_SPEED
    buffer = max(3.2, BASE_BUFFER + EGO_GAIN * speed + LEAD_GAIN * lead_speed)
    if entry and lead.dRel <= buffer:
      self.hold_until_ns = now_ns + HOLD_NS
    active = bool((entry and lead.dRel <= buffer) or
                  (now_ns < self.hold_until_ns and hold and lead.dRel <= buffer + RELEASE_MARGIN))
    return (max(accel_min, -(MIN_BRAKE + BRAKE_GAIN * speed)), True) if active else (None, False)

  def sample(self, sm, *, now_ns, drive_id, target, stopping, accel_min, active):
    try:
      if type(drive_id) is not int or drive_id <= 0 or type(now_ns) is not int or now_ns <= drive_id:
        self.reset()
        return target, stopping
      if drive_id != self.drive_id:
        self.reset()
        self.drive_id = drive_id
      model_ns = sm.logMonoTime['modelV2']
      car = sm['carState']
      fresh = all(sm.valid[name] and sm.alive[name] and drive_id < sm.logMonoTime[name] <= now_ns and
                  now_ns - sm.logMonoTime[name] <= MAX_SOURCE_AGE_NS for name in SOURCES)
      if (not active or not fresh or model_ns <= self.last_model_ns or
          not car.canValid or car.canTimeout or car.gasPressed or car.brakePressed or
          not sm['carControl'].longActive or not sm['selfdriveState'].enabled or sm['controlsState'].forceDecel):
        self.reset()
        return target, stopping
      speed = float(car.vEgo)
      leads = (sm['radarState'].leadOne, sm['radarState'].leadTwo)
      if (not all(math.isfinite(float(v)) for v in (speed, target, accel_min)) or speed < 0 or accel_min > 0 or
          any(not all(math.isfinite(float(getattr(lead, field))) for field in ('vLead', 'dRel', 'modelProb')) or
              lead.dRel < 0 or not 0 <= lead.modelProb <= 1 for lead in leads if lead.present)):
        self.reset()
        return target, stopping
      if self.last_model_ns and model_ns - self.last_model_ns > MAX_MODEL_GAP_NS:
        self.hold_until_ns = 0
      self.last_model_ns = model_ns
      cap = None
      should_stop = stopping
      for lead in leads:
        value, stop = self.cap(lead, speed, accel_min, now_ns)
        if value is not None:
          cap = value if cap is None else min(cap, value)
        should_stop |= stop
      return min(target, cap) if cap is not None else target, should_stop
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
      self.reset()
      return target, stopping
