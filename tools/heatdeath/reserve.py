"""Explicit synthetic CFS CPU pulses; never presented as actual model workload."""

import argparse
import json
import os
from pathlib import Path
import struct
import time


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--core', type=int, required=True)
  parser.add_argument('--cpu-ms', type=float, required=True)
  parser.add_argument('--period-ms', type=float, default=50)
  parser.add_argument('--duration-ns', type=int, required=True)
  parser.add_argument('--release', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  assert 0 < args.cpu_ms < args.period_ms <= 1000
  os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  os.sched_setaffinity(0, {args.core})
  os.setpriority(os.PRIO_PROCESS, 0, 0)
  deadline = time.monotonic() + 20
  while not args.release.exists():
    if time.monotonic() > deadline:
      raise TimeoutError('No source release')
    time.sleep(0.005)
  (start,) = struct.unpack('<Q', args.release.read_bytes())
  period, dose = round(args.period_ms * 1e6), round(args.cpu_ms * 1e6)
  rows = []
  for due in range(start, start + args.duration_ns - period + 1, period):
    delay = (due - time.monotonic_ns()) / 1e9
    if delay > 0:
      time.sleep(delay)
    begun, cpu = time.monotonic_ns(), time.thread_time_ns()
    while time.thread_time_ns() - cpu < dose and time.monotonic_ns() < due + period:
      pass
    rows.append({'dueNs': due, 'startNs': begun, 'endNs': time.monotonic_ns(), 'cpuNs': time.thread_time_ns() - cpu})
  with args.output.open('x') as stream:
    json.dump(
      {
        'kind': 'synthetic-cfs-cpu-reserve',
        'policy': os.sched_getscheduler(0),
        'nice': os.getpriority(os.PRIO_PROCESS, 0),
        'core': args.core,
        'doseNs': dose,
        'periodNs': period,
        'rows': rows,
        'completed': True,
        'missedPulses': sum(row['cpuNs'] < dose for row in rows),
      },
      stream,
    )


if __name__ == '__main__':
  main()
