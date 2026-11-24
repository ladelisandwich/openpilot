"""Root-only lifecycle fixture: no controller, native or sensitivity proof."""
import os
import resource
import time
from unittest.mock import patch

from openpilot.starpilot.flm import live_tracking_probe_worker as worker


def report(job, *, now_ns):
  if job.get('stall'):
    time.sleep(10.)
  return {'cpu': resource.getrlimit(resource.RLIMIT_CPU)[0],
    'as': resource.getrlimit(resource.RLIMIT_AS)[0], 'nice': os.nice(0),
    'scheduler': os.sched_getscheduler(0) if hasattr(os, 'sched_getscheduler') else None,
    'parent': os.getppid(), 'real_now': now_ns}


if __name__ == '__main__':
  with patch.object(worker, "evaluate_job", report):
    raise SystemExit(worker.main())
