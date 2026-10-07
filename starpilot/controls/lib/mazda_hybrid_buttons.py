#!/usr/bin/env python3
"""The distance button as the Mazda hybrid longitudinal mode switch.

With hybrid longitudinal the driver, not an automatic trigger, picks the ACC master:
forced Experimental Mode drives with openpilot emulating the radar, anything else hands the
car back to the stock radar (MRCC). The card maps the distance button onto that choice:

  - short tap while forced experimental: back to standard (MRCC). The tap is used up, so it
    does not also change the driving personality. A short tap in standard keeps its normal job.
  - long press (released between the long and very long thresholds): toggles forced
    experimental (emulation) <-> standard (MRCC).
  - very long press: only its own configured action; the mode is left alone.

The mode lives where the car controller reads it (carcontroller._hybrid_experimental): CEStatus
with Conditional Experimental Mode, the ExperimentalMode param without it.
"""
from opendbc.car.mazda.values import MazdaSafetyFlags

from openpilot.starpilot.common.experimental_state import CEStatus, sync_manual_ce_state


def mazda_hybrid_buttons_active(CP) -> bool:
  flags = int(getattr(CP, "flags", 0))
  return getattr(CP, "brand", None) == "mazda" and bool(flags & MazdaSafetyFlags.HYBRID_LONG) and \
    bool(flags & MazdaSafetyFlags.RADAR_EMULATION)


def hybrid_forced_experimental(params, params_memory, starpilot_toggles) -> bool:
  if getattr(starpilot_toggles, "conditional_experimental_mode", False):
    return params_memory.get_int("CEStatus", default=CEStatus["OFF"]) == CEStatus["USER_OVERRIDDEN"]
  return params.get_bool("ExperimentalMode")


def set_hybrid_forced_experimental(params, params_memory, starpilot_toggles, forced: bool) -> None:
  if getattr(starpilot_toggles, "conditional_experimental_mode", False):
    # Standard is plain OFF, as StarPilot's own toggle leaves forced experimental: the radar
    # only follows the forced state, so Conditional Experimental Mode's automatic triggers
    # keep working for the planner without ever switching the radar.
    status = CEStatus["USER_OVERRIDDEN"] if forced else CEStatus["OFF"]
    params_memory.put_int("CEStatus", status)
    sync_manual_ce_state(params, status)
  else:
    params.put_bool_nonblocking("ExperimentalMode", forced)
