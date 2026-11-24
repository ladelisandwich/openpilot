"""Private bounded observers; original calls, clocks and scheduling stay intact."""

from array import array
from functools import wraps
import hashlib
import itertools
import json
import os
from pathlib import Path
import signal
import sys
import threading
import time


SAMPLE_WIDTH = 12
BASE_COLUMNS = ['sequence', 'nativeTid', 'iteration', 'startMonoNs', 'endMonoNs', 'startThreadCpuNs', 'endThreadCpuNs', 'raised']


def private_output(value):
  output = Path(value).resolve()
  assert sys.dont_write_bytecode and sys.platform == 'linux'
  assert os.environ.get('SP_PRIVATE_NATIVE_BENCH') == '1'
  assert os.environ.get('OPENPILOT_PREFIX', '').startswith('replay-bench-')
  assert all(name not in os.environ for name in ('REPLAY', 'SIMULATION', 'CEREAL_FAKE', 'FAKESEND', 'STARTED'))
  assert Path(os.environ['PARAMS_ROOT']).resolve() == output.parent / 'params'
  assert not output.exists() and not output.is_relative_to(Path.cwd().resolve())
  assert output.parent.is_dir()
  return output


class Rows:
  WIDTH = len(BASE_COLUMNS) + SAMPLE_WIDTH

  def __init__(self, limit):
    self.limit, self.seen = limit, 0
    self.data = array('q', [0]) * (limit * self.WIDTH)

  def put(self, tid, iteration, start, end, cpu_start, cpu_end, raised, samples):
    sequence = self.seen
    position = (sequence % self.limit) * self.WIDTH
    data = self.data
    data[position] = sequence
    data[position + 1] = tid
    data[position + 2] = iteration
    data[position + 3] = start
    data[position + 4] = end
    data[position + 5] = cpu_start
    data[position + 6] = cpu_end
    data[position + 7] = int(raised)
    for index in range(SAMPLE_WIDTH):
      data[position + 8 + index] = int(samples[index]) if index < len(samples) else 0
    self.seen = sequence + 1

  def export(self, path):
    # Native numeric ring is written directly; no millions-of-ints JSON pass.
    with path.open('xb') as stream:
      self.data.tofile(stream)
    return {
      'callsSeen': self.seen,
      'rowsDropped': max(0, self.seen - self.limit),
      'rowsRetained': min(self.seen, self.limit),
      'path': str(path),
      'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
      'bytes': path.stat().st_size,
      'rowOrder': 'For each retained sequence use ring slot sequence modulo rowsPerPhaseMaximum',
    }


class PhaseCapture:
  def __init__(self, output, limit, kind):
    assert 1 <= limit <= 16000
    self.output, self.limit, self.kind = private_output(output), limit, kind
    self.specs, self.bindings, self.rows, self.sources = [], [], {}, {}
    self.local, self.iterations = threading.local(), itertools.count()
    self.errors, self.sample_columns, self.old_handlers = {}, {}, {}
    self.stop_signal = None

  def add(self, owner, name, label, *, step=False, cycle=False, sample=None, sample_columns=()):
    assert label not in self.rows and len(sample_columns) <= SAMPLE_WIDTH
    self.rows[label] = Rows(self.limit)
    self.sample_columns[label] = list(sample_columns)
    self.specs.append((owner, name, label, step, cycle, sample))

  def note_error(self, exc):
    key = type(exc).__module__ + '.' + type(exc).__qualname__
    self.errors[key] = self.errors.get(key, 0) + 1

  def stop(self, signum, _frame):
    self.stop_signal = signum
    raise SystemExit(128 + signum)

  def install(self, owner, name, label, step, cycle, sample):
    original = getattr(owner, name)
    source = sys.modules.get(getattr(original, '__module__', ''))
    if source is not None and getattr(source, '__file__', None):
      path = Path(source.__file__).resolve()
      self.sources[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    recorder, rows = self, self.rows[label]

    @wraps(original)
    def timed(*args, **kwargs):
      prior_iteration = getattr(recorder.local, 'iteration', -1)
      if step or cycle:
        recorder.local.iteration = next(recorder.iterations)
      iteration = getattr(recorder.local, 'iteration', -1)
      tid = threading.get_native_id()
      start, cpu_start = time.monotonic_ns(), time.thread_time_ns()
      result, raised = None, False
      try:
        if recorder.stop_signal is not None:
          raise SystemExit(128 + recorder.stop_signal)
        result = original(*args, **kwargs)
        return result
      except BaseException:
        raised = True
        raise
      finally:
        end, cpu_end = time.monotonic_ns(), time.thread_time_ns()
        try:
          values = sample(args, kwargs, result, raised) if sample is not None else ()
          assert isinstance(values, tuple) and len(values) <= SAMPLE_WIDTH
          rows.put(tid, iteration, start, end, cpu_start, cpu_end, raised, values)
        except Exception as exc:
          recorder.note_error(exc)
        finally:
          if step and not cycle:
            recorder.local.iteration = prior_iteration

    self.bindings.append((owner, name, original, timed))
    setattr(owner, name, timed)

  def __enter__(self):
    self.started_ns = time.monotonic_ns()
    try:
      for spec in self.specs:
        self.install(*spec)
      for signum in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        self.old_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, self.stop)
    except BaseException:
      self.restore()
      raise
    return self

  def restore(self):
    for owner, name, original, wrapped in reversed(self.bindings):
      if getattr(owner, name) is wrapped:
        setattr(owner, name, original)
    for signum, original in self.old_handlers.items():
      signal.signal(signum, original)

  def __exit__(self, exc_type, exc, traceback):
    # Ignore further parent signals only during the bounded final export.
    for signum in self.old_handlers:
      signal.signal(signum, signal.SIG_IGN)
    try:
      for owner, name, original, wrapped in reversed(self.bindings):
        if getattr(owner, name) is wrapped:
          setattr(owner, name, original)
      record = {
        'kind': self.kind,
        'pid': os.getpid(),
        'mainNativeTid': threading.get_native_id(),
        'startedMonoNs': self.started_ns,
        'finishedMonoNs': time.monotonic_ns(),
        'stopSignal': self.stop_signal,
        'rowsPerPhaseMaximum': self.limit,
        'baseColumns': BASE_COLUMNS,
        'sampleColumns': self.sample_columns,
        'sources': self.sources,
        'observerErrors': self.errors,
        'timing': {
          label: rows.export(self.output.with_name(self.output.stem + '.' + str(index) + '.bin')) for index, (label, rows) in enumerate(self.rows.items())
        },
        'binaryFormat': {
          'scalar': 'signed 64-bit native endian',
          'byteorder': sys.byteorder,
          'scalarBytes': array('q').itemsize,
          'columnsPerRow': Rows.WIDTH,
          'unusedSlots': 'zero; only rowsRetained sequences are observations',
        },
        'scope': 'Real calls, wall and per-thread CPU; no new clocks, grants, inputs, GC policy or scheduler changes',
        'observerEffect': 'Python wrappers, clock reads, primitive sampling and preallocated ring writes add overhead; no per-call I/O or JSON',
        'inclusivePhases': True,
        'samplingOrder': 'exit clocks precede observational scalar sampling',
      }
      temporary = self.output.with_suffix(self.output.suffix + '.tmp')
      assert not temporary.exists() and not self.output.exists()
      temporary.write_text(json.dumps(record, separators=(',', ':')) + '\n')
      temporary.replace(self.output)
    finally:
      self.restore()
    return False
