"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License
(https://github.com/zoompilot/zoompilot, LICENSE.md). classify() is ported into StarPilot unchanged from
openpilot/sunnypilot/selfdrive/controls/lib/steer_limit.py (zoompilot 5b87a72, 2026-10-22); the Mazda
schedule and SteerLimitReclassifier below are StarPilot's. It runs behind the MazdaSteerLimitClassifier
toggle (controlsd.publish).

Steer-limit classifier.

controlsd derives steer_limited_by_safety from |CC.actuators.torque - carOutput.torque| > 0.01,
and the lateral controller freezes its integrator on it and suppresses the saturation alert
while it is set. On a torque car with a rate-limited carcontroller that flag fires on ordinary
motion (one 10-count step of the Mazda's 800 is 0.0125), so the integrator spends much of a
drive frozen: 8-30% of hands-off steering frames on the CX-9 (routes 00000022-35), most on the
highway. classify() splits the mismatch into rate-limited, at the rail and driver-limited, and
hands the tunes a directional, rail-aware flag under the same name:

  limited = mismatch and not at_rail and deepening,  deepening = error_prev * pid.i >= 0

Rate limiting stays in the flag: while the command outruns the actuator's slew the plant is not
following it, and integrating that error is actuator-rate windup. The freeze is directional so
decay toward a reversing error stays live, and the rail is carved out because the PID limits
already sit on it (latcontrol_torque steer_max, the TI ceiling on a Torque Interceptor car) and a
False flag lets the tune's own saturation test raise the alert.

Blind spot: a frame the panda rejects is still reported as delivered in carOutput, so a starved
actuator looks like a clean one here.
"""
from typing import NamedTuple

MISMATCH_THRESHOLD = 1e-2  # controlsd.publish: abs(CC.actuators.torque - CO.actuatorsOutput.torque) > 1e-2
RAIL_EPS = 1e-3            # the tunes' own saturation test: steer_max - |output| < 1e-3
# a speed-dependent scale (Rivian's) is read at vEgoRaw and rounded by the carcontroller and
# interpolated at vEgo here, so accept a slightly short step
RATE_STEP_FRACTION = 0.9


class SteerLimit(NamedTuple):
  limited: bool         # what the tunes receive as steer_limited_by_safety
  driver_limited: bool
  rate_limited: bool
  at_rail: bool


CLEAN = SteerLimit(False, False, False, False)


def classify(cmd: float, applied: float, applied_prev: float | None, slew_up: float, slew_down: float,
             rail_scale: float, upstream_flag: bool, error_prev: float, integrator: float) -> SteerLimit:
  """Classify controlsd's steer-limit mismatch for one frame. All torques are normalized, in the
  actuator's sign convention (CC.actuators.torque and carOutput.actuatorsOutput.torque).

  cmd           this frame's commanded torque
  applied       the carcontroller's applied torque as of this frame
  applied_prev  the applied torque one frame earlier, None on the first frame after lateral
                became active (no history: limited and driver_limited fall back to upstream_flag)
  slew_up/down  the carcontroller's per-frame normalized rate limits at the current speed,
                up = growing in magnitude, down = shrinking
  rail_scale    the actuator's ceiling as a fraction of scale at the current speed (1.0 when none)
  upstream_flag controlsd's own steer_limited_by_safety for this frame
  error_prev    the tune's pid_log.error from the frame just computed
  integrator    the tune's live pid.i
  """
  mag = abs(applied)
  at_rail = mag >= rail_scale - RAIL_EPS or mag >= 1.0 - RAIL_EPS
  if abs(cmd - applied) <= MISMATCH_THRESHOLD:
    return SteerLimit(False, False, False, at_rail)
  if applied_prev is None:
    return SteerLimit(bool(upstream_flag), bool(upstream_flag), False, at_rail)

  move = applied - applied_prev
  step = slew_up if mag > abs(applied_prev) else slew_down
  toward = move * (cmd - applied_prev) > 0.0
  # A full step: the rate limit is what holds the actuator back. A gap no wider than the move
  # it just made: the actuator is tracking a command that walks slower than the rate limit,
  # and the mismatch is only the one-frame lag between CC and carOutput.
  rate_limited = toward and (abs(move) >= RATE_STEP_FRACTION * step or abs(cmd - applied) <= abs(move) + MISMATCH_THRESHOLD)
  # freeze only integration that would deepen |i|; decay toward a reversing error stays live
  deepening = error_prev * integrator >= 0.0
  return SteerLimit(not at_rail and deepening, not rate_limited and not at_rail, rate_limited, at_rail)


def mazda_steer_limit_schedule(CP) -> tuple[float, float, float] | None:
  """(slew_up, slew_down, rail_scale) for a Mazda torque car, as fractions of the car controller's STEER_MAX:
  its STEER_DELTA_UP and STEER_DELTA_DOWN per frame, and the Torque Interceptor's ceiling as the rail (the
  full scale without one). None for any other car, where controlsd keeps its own flag."""
  if getattr(CP, "brand", None) != "mazda" or CP.lateralTuning.which() != "torque":
    return None
  from opendbc.car.mazda.values import CarControllerParams, MazdaSafetyFlags
  ccp = CarControllerParams(CP)
  steer_max = getattr(ccp, "STEER_MAX", None)
  if not steer_max:
    return None
  ceiling = ccp.TI_CEILING if int(CP.flags) & MazdaSafetyFlags.TORQUE_INTERCEPTOR and hasattr(ccp, "TI_CEILING") else steer_max
  return ccp.STEER_DELTA_UP / steer_max, ccp.STEER_DELTA_DOWN / steer_max, ceiling / steer_max


class SteerLimitReclassifier:
  """controlsd's steer_limited_by_safety for the next frame, re-decided by classify() on a car with a schedule.
  Holds the one frame of history classify() needs; a frame it does not run on (toggle off, lateral inactive, no
  schedule) hands controlsd's own flag through and drops that history."""

  def __init__(self, schedule: tuple[float, float, float] | None):
    self.schedule = schedule
    self.applied_prev: float | None = None

  def reset(self) -> None:
    self.applied_prev = None

  def update(self, enabled: bool, lat_active: bool, cmd: float, applied: float, upstream_flag: bool,
             error_prev: float, integrator: float) -> bool:
    if self.schedule is None or not enabled or not lat_active:
      self.reset()
      return bool(upstream_flag)
    slew_up, slew_down, rail_scale = self.schedule
    limit = classify(cmd, applied, self.applied_prev, slew_up, slew_down, rail_scale, upstream_flag, error_prev, integrator)
    self.applied_prev = applied
    return limit.limited
