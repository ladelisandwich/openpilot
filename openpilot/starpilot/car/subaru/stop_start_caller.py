"""Start the bounded request only after the actual Panda configuration is current."""


def update(card, now_ns):
  owner = card.CI.CC.stop_start
  avh = card.CI.CC.avh_startup
  if (owner is None or owner.done) and (avh is None or avh.done):
    return
  sm = card.sm
  try:
    current = bool(sm.seen['carControl'] and sm.valid['carControl'] and sm.alive['carControl'] and
                   0 < int(sm.logMonoTime['carControl']) <= now_ns and now_ns - int(sm.logMonoTime['carControl']) <= 150_000_000 and
                   0 < int(sm.recv_time['carControl'] * 1e9) <= now_ns and now_ns - int(sm.recv_time['carControl'] * 1e9) <= 150_000_000)
    configured = current and card.startup_panda_configured() and not any(ps.faults for ps in sm['pandaStates'])
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
    configured = False
  for request in (owner, avh):
    if request is not None:
      request.observe_configured(configured, now_ns / 1e9)
