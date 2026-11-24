"""One explicitly selected production response child with inherited real reaping."""
from openpilot.starpilot.flm.fit_process import FitProcess
from openpilot.starpilot.flm.live_tracking_response_policy import MAX_WALL_NS, WORKER_MODULE, validate_production_result


class ProductionBootstrapProcess(FitProcess):
  worker_module = WORKER_MODULE
  wall_ns = MAX_WALL_NS
  interval_ns = 0

  def poll(self, *, token, now_ns):
    result = super().poll(token=token, now_ns=now_ns)
    if result is None:
      return None
    try:
      return validate_production_result(result)
    except (KeyError, TypeError, ValueError):
      self.reason = 'production_response_report_unqualified'
      return None
