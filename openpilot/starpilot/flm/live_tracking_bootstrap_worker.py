"""Explicit response-only purpose worker; production budget awaits measurement.

This version is an isolated diagnostic purpose, not the legacy20-second fitter.
Its result is numerical calibration only and cannot authorize actuation/save.
"""
import gc
import json
import os
import resource
import sys

PURPOSE = 'live-response-bootstrap-diagnostic-v1'
MAX_CPU = 180
MAX_WALL_NS = 300_000_000_000
MAX_AS = 2*1024*1024*1024
MAX_INPUT = 8*1024*1024
MAX_OUTPUT = 65536


def main():
  try:
    if os.environ.get('STARPILOT_LIVE_BOOTSTRAP_PURPOSE') != PURPOSE:
      raise ValueError('explicit isolated response measurement purpose required')
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
    raw = sys.stdin.buffer.read(MAX_INPUT+1)
    if len(raw) > MAX_INPUT:
      raise ValueError('response worker input bound')
    from openpilot.starpilot.flm.live_tracking_bootstrap import bootstrap_snapshot
    was_enabled = gc.isenabled()
    try:
      gc.disable()
      # Native accelerator remains disabled under this separate180-second
      # purpose until an explicit new native-worker policy is qualified. Pure
      # Python fallback is the original algorithm, not a retagged binary proof.
      result = bootstrap_snapshot(json.loads(raw))
    finally:
      if was_enabled:
        gc.enable()
    result = dict(result, diagnostic_purpose=PURPOSE, production_admissible=False)
    encoded = json.dumps(result, separators=(',', ':'), allow_nan=False).encode()
    if len(encoded) > MAX_OUTPUT:
      raise ValueError('response worker output bound')
    sys.stdout.buffer.write(encoded)
    sys.stdout.buffer.flush()
    return 0
  except (Exception, MemoryError) as error:
    # Bounded diagnostic metadata only, never snapshot/source content.
    sys.stderr.write(type(error).__name__+': '+str(error)[:240]+'\n')
    return 1


if __name__ == '__main__':
  raise SystemExit(main())
