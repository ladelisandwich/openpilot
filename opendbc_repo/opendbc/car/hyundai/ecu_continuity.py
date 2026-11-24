import time

from opendbc.car import make_tester_present_msg
from opendbc.car.hyundai.ecu_startup import HyundaiECUStartup, Outcome
from opendbc.car.hyundai.ev9_keeper import EV9Keeper as StartupKeeper


def commit_controller_tester(owner, frames, *, valid):
  keeper = getattr(owner, "keeper", None)
  if keeper is None or owner.handed_off or not valid or owner.closed or not owner.published:
    return
  tester = make_tester_present_msg(owner.address, owner.bus, suppress_response=True)
  if tester not in frames:
    return
  keeper.stop()
  owner.check()
  owner.handed_off = True


class ContinuousECUStartup(HyundaiECUStartup):
  def __init__(self, cp, callbacks, *, address, bus, label, clock=time.monotonic, keeper_factory=StartupKeeper):
    super().__init__(cp, callbacks, address=address, bus=bus, label=label)
    self.clock = clock
    self.keeper_factory = keeper_factory
    self.keeper = None
    self.handed_off = False

  def prepare(self, *, admission):
    result = super().prepare(admission=admission)
    if self.continuity_required() and self.outcome is Outcome.SENT_UNCONFIRMED:
      self.keeper = self.keeper_factory(lambda: self.callbacks[1]([make_tester_present_msg(self.address, self.bus, suppress_response=True)]), clock=self.clock)
      try:
        self.keeper.start()
      except Exception:
        if self.keeper.thread is not None and self.keeper.thread.ident is None:
          self.keeper.event.set()
          self.keeper.thread = None
        self.close()
        raise
    return result

  def check(self):
    super().check()
    keeper = getattr(self, "keeper", None)
    if keeper is not None and keeper.abort_reason is not None:
      keeper.stop()
      self.outcome = Outcome.ABORT_UNCERTAIN
      raise RuntimeError(f"{self.label} startup diagnostic continuity failed")

  def before_control(self, **context):
    self.check()
    return self.outcome is not Outcome.SENT_UNCONFIRMED or (context['configured'] and context['sources_current'] and context['control_current'])

  def maintain(self, *, configured):
    self.check()

  def sent(self, frames, *, valid):
    commit_controller_tester(self, frames, valid=valid)

  def close(self):
    keeper = getattr(self, "keeper", None)
    if keeper is not None:
      keeper.stop()
    super().close()

  def continuity_required(self):
    return False
