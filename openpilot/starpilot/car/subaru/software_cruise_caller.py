"""Bind the Dom button state machine to current Card publications."""

import math
from opendbc.car.subaru.software_cruise import selected


def update(card, cs, cc, now_ns):
  controller = card.CI.CC
  controller.software_cruise_command = 0
  if not selected(card.CP):
    return
  if not hasattr(controller, "software_cruise"):
    from openpilot.starpilot.car.subaru.software_cruise import SoftwareCruise

    controller.software_cruise = SoftwareCruise(card.CP)
  try:
    services = ("longitudinalPlan", "carControl", "selfdriveState")
    if not all(
      card.sm.seen[name]
      and card.sm.valid[name]
      and card.sm.alive[name]
      and 0 < int(card.sm.logMonoTime[name]) <= now_ns
      and now_ns - int(card.sm.logMonoTime[name]) <= 150_000_000
      and 0 < int(card.sm.recv_time[name] * 1e9) <= now_ns
      and now_ns - int(card.sm.recv_time[name] * 1e9) <= 150_000_000
      for name in services
    ):
      controller.software_cruise._reset()
      return
    plan = card.sm["longitudinalPlan"]
    acceleration = float(plan.aTarget)
    ceiling = float(cs.vCruise) / 3.6
    if not math.isfinite(acceleration) or not math.isfinite(ceiling) or not 0 < ceiling < 255 / 3.6:
      controller.software_cruise._reset()
      return
    target = min(ceiling, max(0.0, cs.vEgo * 1.01 + 3 * acceleration))
    command, _ = controller.software_cruise.run(cs, cc, target, card.is_metric, lead_present=bool(plan.hasLead))
    controller.software_cruise_command = command
    controller.software_cruise_command_ns = now_ns
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
    controller.software_cruise._reset()
