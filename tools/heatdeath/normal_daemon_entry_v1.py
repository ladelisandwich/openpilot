"""Private early import / final-CP gate around unchanged normal daemon mains."""

import argparse
import hashlib
import importlib
import json
import os
from pathlib import Path
import sys
import time


def digest(path):
  return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--module', required=True)
  parser.add_argument('--module-sha256', required=True)
  parser.add_argument('--gate', type=Path, required=True)
  parser.add_argument('--output', type=Path)
  parser.add_argument('--limit', type=int, default=16000)
  args = parser.parse_args()
  assert args.module in ('openpilot.selfdrive.selfdrived.selfdrived', 'openpilot.selfdrive.controls.controlsd')
  assert os.geteuid() == 0 and sys.platform == 'linux' and sys.dont_write_bytecode
  assert os.environ.get('SP_PRIVATE_NATIVE_BENCH') == '1'
  assert os.environ['OPENPILOT_PREFIX'].startswith('replay-bench-')
  assert all(name not in os.environ for name in ('REPLAY', 'SIMULATION', 'CEREAL_FAKE', 'FAKESEND', 'STARTED'))
  gate = args.gate.resolve()
  assert gate.parent / 'params' == Path(os.environ['PARAMS_ROOT']).resolve()
  assert not gate.is_relative_to(Path.cwd().resolve())
  # Cold imports overlap Card initialization; normal main/scheduler remain
  # untouched and execute only after the parent validates final CP bytes.
  module = importlib.import_module(args.module)
  assert digest(module.__file__) == args.module_sha256
  from actor import install

  install(module, 'controlsd' if args.module.endswith('controlsd') else 'selfdrived')
  deadline = time.monotonic_ns() + 12_000_000_000
  while True:
    assert time.monotonic_ns() < deadline, 'No parent CP token within12s after imports'
    try:
      token = json.loads(gate.read_text())
    except FileNotFoundError:
      time.sleep(0.005)
      continue
    assert token['kind'] == 'private-verified-car-params-main-gate-v1'
    assert token['prefix'] == os.environ['OPENPILOT_PREFIX']
    assert token['paramsRoot'] == os.environ['PARAMS_ROOT']
    assert token['carParamsSha256'] == os.environ['SP_DAEMON_MAIN_GATE_CP_SHA256']
    assert 0 < token['verifiedMonoNs'] <= time.monotonic_ns()
    from openpilot.common.params import Params

    assert hashlib.sha256(Params().get('CarParams')).hexdigest() == token['carParamsSha256']
    break
  if args.output is None:
    module.main()
    return
  from phase_timing import PhaseCapture
  from openpilot.common.realtime import Ratekeeper

  capture = PhaseCapture(args.output, args.limit, 'private-normal-daemon-ratekeeper-only')
  capture.add(
    Ratekeeper,
    'monitor_time',
    'Ratekeeper.monitor_time',
    sample=lambda values, kwargs, result, raised: (round(values[0].avg_dt.get_average() * 1e9), int(values[0].lagging)),
    sample_columns=('averagePeriodNs', 'lagging'),
  )
  if os.environ.get('SP_PRIVATE_LOOP_MACRO') == '1':
    if args.module.endswith('selfdrived'):
      owner = module.SelfdriveD
      capture.add(owner, 'step', 'SelfdriveD.step', cycle=True)
      for name in ('data_sample', 'update_events', 'update_alerts', '_decide_aol_axes', 'publish_axis_proof', 'publish_selfdriveState'):
        capture.add(owner, name, 'SelfdriveD.' + name)
    else:
      owner = module.Controls
      capture.add(owner, 'update', 'Controls.update', cycle=True)
      capture.add(owner, 'state_control', 'Controls.state_control')
      capture.add(owner, 'publish', 'Controls.publish')
  profiler = None
  if os.environ.get('SP_PRIVATE_SPARSE_PROFILE') == '1':
    import cProfile
    from functools import wraps

    profiler = cProfile.Profile()
    owner, name = (module.SelfdriveD, 'update_events') if args.module.endswith('selfdrived') else (module.Controls, 'state_control')
    original, samples = getattr(owner, name), [0, 0]

    @wraps(original)
    def sparse(*values, **kwargs):
      samples[0] += 1
      if samples[0] % 197:
        return original(*values, **kwargs)
      samples[1] += 1
      profiler.enable()
      try:
        return original(*values, **kwargs)
      finally:
        profiler.disable()

    setattr(owner, name, sparse)
  if os.environ.get('SP_PRIVATE_CONTROLS_FIFO52') == '1' and args.module.endswith('controlsd'):
    from functools import wraps
    from openpilot.common.realtime import Priority

    original_run = module.Controls.run

    @wraps(original_run)
    def private_fifo_run(owner):
      assert Path('/data/params/d/IsOffroad').read_bytes().strip() == b'1'
      assert Path('/data/params/d/IsEngaged').read_bytes().strip() == b'0'
      before = {'policy': os.sched_getscheduler(0), 'priority': os.sched_getparam(0).sched_priority, 'affinity': sorted(os.sched_getaffinity(0))}
      assert before == {'policy': os.SCHED_OTHER, 'priority': 0, 'affinity': [6]}
      target = int(Priority.CTRL_HIGH) - 1
      assert target == 52
      os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(target))
      after = {'policy': os.sched_getscheduler(0), 'priority': os.sched_getparam(0).sched_priority, 'affinity': sorted(os.sched_getaffinity(0))}
      assert after == {'policy': os.SCHED_FIFO, 'priority': 52, 'affinity': [6]}
      args.output.with_suffix('.private-policy.json').write_text(
        json.dumps(
          {
            'pid': os.getpid(),
            'before': before,
            'after': after,
            'scope': 'Private owned benchmark main thread only; production source and normal daemons untouched',
            'normal_constructor_and_scheduler_completed': True,
          }
        )
      )
      return original_run(owner)

    module.Controls.run = private_fifo_run
  try:
    with capture:
      module.main()
  finally:
    if profiler is not None:
      profiler.dump_stats(str(args.output.with_suffix('.pstats')))
      args.output.with_suffix('.profile.json').write_text(
        json.dumps(
          {
            'function': name,
            'calls': samples[0],
            'sampled': samples[1],
            'period': 197,
            'scope': 'Sparse diagnostic only; cProfile overhead and periodic sampling do not prove headroom or whole-cycle CPU',
          }
        )
      )


if __name__ == '__main__':
  main()
