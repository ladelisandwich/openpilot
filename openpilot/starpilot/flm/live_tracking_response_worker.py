"""Fixed production response-calibration purpose, separate from legacy fitting."""
import gc
import json
import os
import resource
import sys

from openpilot.starpilot.flm.live_tracking_response_policy import MAX_AS, MAX_CPU, MAX_INPUT, MAX_OUTPUT, PURPOSE, production_result

from openpilot.starpilot.flm.live_tracking_response_policy import MAX_WALL_NS as MAX_WALL_NS


def main():
  try:
    if not sys.platform.startswith('linux'):
      raise ValueError('qualified Linux response worker required')
    import ctypes
    import signal
    parent = int(os.environ['STARPILOT_FLM_PARENT_PID'])
    if ctypes.CDLL(None).prctl(1, signal.SIGKILL, 0, 0, 0) != 0 or os.getppid() != parent:
      raise ValueError('bootstrap parent disappeared')
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
    os.nice(10)
    for kind, ceiling in ((resource.RLIMIT_AS, MAX_AS), (resource.RLIMIT_CPU, MAX_CPU)):
      soft, hard = resource.getrlimit(kind)
      limit = min((ceiling, *[value for value in (soft, hard) if value >= 0]))
      resource.setrlimit(kind, (limit, hard))
    if any(os.environ.get(name) != '1' for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS')):
      raise ValueError('one numerical thread required')
    # This exact module/purpose and actual resource guards are independently
    # checked by the native helper; no inherited environment opt-in selects it.
    from openpilot.starpilot.flm.native_primitives import enable_response_worker
    if not enable_response_worker(PURPOSE):
      raise ValueError('qualified response native runtime unavailable')
    raw = sys.stdin.buffer.read(MAX_INPUT+1)
    if len(raw) > MAX_INPUT:
      raise ValueError('response worker input bound')
    from openpilot.starpilot.flm.live_tracking_bootstrap import bootstrap_snapshot
    was_enabled = gc.isenabled()
    try:
      gc.disable()
      result = bootstrap_snapshot(json.loads(raw))
    finally:
      if was_enabled:
        gc.enable()
    result = production_result(result)
    encoded = json.dumps(result, separators=(',', ':'), allow_nan=False).encode()
    if len(encoded) > MAX_OUTPUT:
      raise ValueError('response worker output bound')
    sys.stdout.buffer.write(encoded)
    sys.stdout.buffer.flush()
    return 0
  except (Exception, MemoryError) as error:
    sys.stderr.write(type(error).__name__+': '+str(error)[:240]+'\n')
    return 1


if __name__ == '__main__':
  raise SystemExit(main())
