"""Require current production Controls and SelfdriveD publications for SNG decisions."""

from opendbc.car.subaru.sng import selected


def update(card, cs, cc, now_ns):
  controller = card.CI.CC
  controller.sng_command_ns = 0
  if not selected(card.CP):
    return
  services = ('carControl', 'selfdriveState', 'longitudinalPlan')
  try:
    fresh = all(
      card.sm.seen[name]
      and card.sm.valid[name]
      and card.sm.alive[name]
      and 0 < int(card.sm.logMonoTime[name]) <= now_ns
      and now_ns - int(card.sm.logMonoTime[name]) <= 150_000_000
      and 0 < int(card.sm.recv_time[name] * 1e9) <= now_ns
      and now_ns - int(card.sm.recv_time[name] * 1e9) <= 150_000_000
      for name in services
    )
    if fresh:
      controller.sng_command_ns = now_ns
    else:
      controller.sng_owner.reset(controller.frame)
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
    controller.sng_owner.reset(controller.frame)
