"""Optional Mazda camera observation; never a longitudinal control grant."""
import time

from opendbc.car.mazda.values import GEN1, MazdaFlags
from openpilot.cereal import messaging

MAX_AGE_NS = 150_000_000
DASHBOARD_CARS = frozenset(GEN1)  # the mazda_2017 DBC's camera; the fork's GEN2/GEN3 platforms use other DBCs


def capable(cp):
  # Read-only observation scope is separate from active-control/AOL admission.
  # Dom's intentional dashboard commit explicitly covers all six GEN1 IDs. Any GEN1 flag set (the Torque
  # Interceptor and radar emulation configurations of the ladelisandwich fork included) keeps the camera.
  return (cp.brand == 'mazda' and cp.carFingerprint in DASHBOARD_CARS and bool(int(cp.flags) & int(MazdaFlags.GEN1)) and
          not int(cp.flags) & int(MazdaFlags.GEN2 | MazdaFlags.GEN3) and not cp.notCar)


def compose(cp, parsed, state, message, *, car_state_stamp):
  if not capable(cp):
    return message
  message = write_stop_sign(parsed, state, message, car_state_stamp=car_state_stamp)
  return write_body_inputs(parsed, message)


# The body inputs the car state reads for openpilot's own use (opendbc mazda carstate; tools/mazda/CONTROLLABILITY.md),
# on starpilotCarState for modeld's wiper gate, the tune export and the card's own hooks.
BODY_INPUTS = (("distance_increase_button", "distanceIncreasePressed"), ("hazard_lights", "hazardLights"),
               ("wipers_active", "wipersActive"), ("washer_active", "washerActive"), ("horn_pressed", "hornPressed"),
               ("auto_hold_active", "autoHoldActive"), ("lamps_on", "lampsOn"))


def write_body_inputs(parsed, message):
  for attribute, field in BODY_INPUTS:
    setattr(message.starpilotCarState, field, bool(getattr(parsed, attribute, False)))
  return message


def write_stop_sign(parsed, state, message, *, car_state_stamp):
  if message is None:
    message = messaging.new_message('starpilotCarState', valid=bool(state.canValid), logMonoTime=car_state_stamp)
    message.starpilotCarState.sourceCarStateMonoTime = car_state_stamp
  # Missing or stale optional camera data must not invalidate unrelated evidence.
  message.starpilotCarState.dashboardStopSign = 0
  before = time.monotonic_ns()
  boot = time.clock_gettime_ns(getattr(time, 'CLOCK_BOOTTIME', time.CLOCK_MONOTONIC))
  after = time.monotonic_ns()
  observation = getattr(parsed, 'dashboard_stop_sign', None)
  if (type(observation) is tuple and len(observation) == 2 and type(observation[0]) is bool and
      type(observation[1]) is int and 0 <= after - before <= 1_000_000 and state.canValid and not state.canTimeout):
    age = boot - observation[1]
    stamp = before - age
    if 0 <= age <= MAX_AGE_NS and 0 < stamp <= car_state_stamp:
      message.starpilotCarState.dashboardStopSign = int(observation[0])
  return message
