"""Mazda Torque Interceptor startup.

The interceptor answers on bus 1, which the panda routes to the OBD-C port only in the Mazda safety mode with the
TORQUE_INTERCEPTOR flag. pandad applies that mode after card signals ControlsReady, and card's generic rule holds
ControlsReady until CarState is valid, which needs the interceptor's feedback: a cycle nothing breaks on its own.
While the car bus itself is healthy and the interceptor's bus is the only silent one, the startup is pending, and
card may initialize on the car bus alone, as the fork's previous base did.
"""
from opendbc.car import Bus
from opendbc.car.mazda.values import MazdaSafetyFlags


def ti_startup_pending(CP, can_parsers, CS) -> bool:
  """True while the car bus is healthy and only the interceptor's bus (bus 1) keeps CarState invalid."""
  if CP.brand != "mazda" or not (CP.flags & MazdaSafetyFlags.TORQUE_INTERCEPTOR):
    return False
  if CS.canValid and not CS.canTimeout:
    return False
  body = can_parsers.get(Bus.body)
  if body is None or not body.bus_timeout:
    return False  # the interceptor answers: something else keeps CarState invalid
  for bus, cp in can_parsers.items():
    # can_invalid_cnt is 0 right after the interface found every message of that parser valid; reading it, unlike
    # can_valid, has no side effect on the debounce
    if bus != Bus.body and (cp.bus_timeout or cp.can_invalid_cnt != 0):
      return False
  return True
