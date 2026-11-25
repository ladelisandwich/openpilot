"""Mazda Torque Interceptor startup.

The interceptor answers on bus 1, which the panda routes to the OBD-C port only in the Mazda safety mode with the
TORQUE_INTERCEPTOR flag. pandad applies that mode after card signals ControlsReady, and card's generic rule holds
ControlsReady until CarState is valid, which needs the interceptor's feedback: a cycle nothing breaks on its own.
While the car bus itself is healthy and the interceptor's bus is the only silent one, the startup is pending, and
card may initialize on the car bus alone, as the fork's previous base did. controlsd marks carControl valid only
with a valid CarState, so the same pending state also stands in for that message's validity.
"""
from opendbc.car import Bus
from opendbc.car.mazda.values import MazdaSafetyFlags
from opendbc.can.parser import MAX_BAD_COUNTER


def parser_healthy(cp) -> bool:
  """can_valid without its side effect on the invalid-count debounce (and without the short-circuit of the
  interface's all(), which leaves parsers after the first invalid one unevaluated)."""
  bus_timeout = cp.bus_timeout
  if bus_timeout:
    return False
  now = cp._last_update_nanos
  return all(st.valid(now, bus_timeout) and st.counter_fail < MAX_BAD_COUNTER for st in cp.message_states.values())


def ti_startup_pending(CP, can_parsers, CS) -> bool:
  """True while the car bus is healthy and only the interceptor's bus (bus 1) keeps CarState invalid."""
  if CP.brand != "mazda" or not (CP.flags & MazdaSafetyFlags.TORQUE_INTERCEPTOR):
    return False
  if CS.canValid and not CS.canTimeout:
    return False
  body = can_parsers.get(Bus.body)
  if body is None or not body.bus_timeout:
    return False  # the interceptor answers: something else keeps CarState invalid
  return all(parser_healthy(cp) for bus, cp in can_parsers.items() if bus != Bus.body)
