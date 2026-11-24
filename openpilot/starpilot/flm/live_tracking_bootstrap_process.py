"""Nonblocking response-only diagnostic actor, separate from legacy fitter."""
from openpilot.starpilot.flm.fit_process import FitProcess
from openpilot.starpilot.flm.live_tracking_bootstrap_worker import PURPOSE, MAX_WALL_NS


class LiveBootstrapProcess(FitProcess):
  worker_module = 'openpilot.starpilot.flm.live_tracking_bootstrap_worker'
  wall_ns = MAX_WALL_NS
  interval_ns = 0

  def submit(self, snapshot, *, token, now_ns):
    # The parent makes purpose explicit in its child environment. This actor
    # never changes parent scheduling, legacy fitter limits or native policy.
    import os
    if os.environ.get('STARPILOT_LIVE_BOOTSTRAP_PURPOSE') != PURPOSE:
      self.reason = 'production_bootstrap_budget_not_yet_qualified'
      return False
    return super().submit(snapshot, token=token, now_ns=now_ns)
