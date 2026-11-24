"""Durable test identities and native fault stacks; never changes test outcomes."""
import faulthandler
import hashlib
import json
import os
from pathlib import Path
import time
import warnings

_ROOT = None
_PID = None
_FAULT = None
_STATE = {}


def initialize(warning_action=None):
  global _ROOT, _PID, _FAULT, _STATE
  if warning_action is not None:
    warnings.simplefilter(warning_action)
  directory = os.environ.get('STARPILOT_TEST_DIAGNOSTICS')
  if not directory or _PID == os.getpid():
    return
  _ROOT, _PID = Path(directory).resolve(), os.getpid()
  _ROOT.mkdir(parents=True, exist_ok=True)
  _FAULT = (_ROOT / f'worker-{_PID}.fault.log').open('ab', buffering=0)
  # faulthandler bounds native dumps to 100 threads and 100 frames per thread.
  # Its separate fd remains open when per-test stdout/stderr capture is lost.
  faulthandler.enable(file=_FAULT, all_threads=True)
  _STATE = {'pid': _PID, 'batch': [], 'active_test': None, 'phase': 'initialized'}
  update()


def update(**fields):
  if _ROOT is None or _PID != os.getpid():
    return
  _STATE.update(fields, monotonic_ns=time.monotonic_ns())
  path = _ROOT / f'worker-{_PID}.json'
  temporary = path.with_suffix('.tmp')
  temporary.write_text(json.dumps(_STATE, sort_keys=True) + '\n')
  os.replace(temporary, path)


def event(kind, **fields):
  if _ROOT is None or _PID != os.getpid():
    return
  # Only identities/status/timestamps, never captured output or exception chains.
  row = {'event': kind, 'pid': _PID, 'monotonic_ns': time.monotonic_ns(), **fields}
  with (_ROOT / f'worker-{_PID}.jsonl').open('a') as stream:
    stream.write(json.dumps(row, sort_keys=True) + '\n')


def batch_started(identities):
  initialize()
  update(batch=list(identities), active_test=None, phase='batch')


def node_started(identity):
  update(active_test=identity, phase='test')
  event('start', id=identity)


def node_finished(identity, status):
  event('finish', id=identity, status=status)
  update(active_test=None, phase='between_tests')


def pool_failure(processes, detail):
  directory = os.environ.get('STARPILOT_TEST_DIAGNOSTICS')
  if not directory:
    return None
  root = Path(directory).resolve()
  root.mkdir(parents=True, exist_ok=True)
  path = root / 'pool-failure.json'
  if path.exists():
    try:
      value = json.loads(path.read_text())
      rows = value['processes']
      observed = {}
      for process in processes:
        try:
          observed[process.pid] = process.exitcode
        except (OSError, ValueError):
          continue
      changed = False
      for row in rows:
        code = observed.get(row['pid'])
        if row['exitcode'] is None and code is not None:
          row['exitcode'] = code
          changed = True
      if changed:
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps(value, sort_keys=True, indent=2) + '\n')
        os.replace(temporary, path)
    except (OSError, ValueError, TypeError, KeyError):
      # An unavailable or malformed diagnostic must not replace test outcomes.
      pass
  else:
    rows = [{'pid': process.pid, 'exitcode': process.exitcode} for process in processes]
    value = {'detail': detail, 'detail_sha256': hashlib.sha256(detail.encode()).hexdigest(),
             'processes': rows, 'workers': [json.loads(item.read_text()) for item in sorted(root.glob('worker-*.json'))],
             'scope': 'Observed worker exits and last durable test identities; no inferred OOM or culprit for alive workers'}
    path.write_text(json.dumps(value, sort_keys=True, indent=2) + '\n')
  return path
