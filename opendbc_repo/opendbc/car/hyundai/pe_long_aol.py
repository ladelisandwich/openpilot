"""Exact PE LONG/AOL contract; EV9 words and calibrations are not aliases."""
import math

from opendbc.car import structs
from opendbc.car.hyundai.ioniq5pe_long_profile import qualified as long_qualified, LONG_PARAM

WORDS = frozenset((LONG_PARAM,))


def qualified(cp, *, marked_only=False):
  return bool(cp.steerControlType == structs.CarParams.SteerControlType.angle and long_qualified(cp) and
              cp.alternativeExperience in ((32,) if marked_only else (0, 32)))


def temporary_restriction(cp, state):
  gears = structs.CarState.GearShifter
  return bool(state is not None and qualified(cp, marked_only=True) and state.canValid and not state.canTimeout and
              not state.steerFaultPermanent and state.gearShifter in (gears.drive, gears.reverse, gears.park, gears.neutral) and
              math.isfinite(state.vEgoRaw) and (state.brakePressed or state.gasPressed or state.steerFaultTemporary or
              state.gearShifter != gears.drive or state.standstill or state.vEgoRaw <= 0.3))
