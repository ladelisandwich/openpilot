"""Bounded private observations only; no input, policy, IPC or clock overrides."""

from array import array
import gc
import hashlib
import json
import resource
import sys
import time

WIDTH = 32
POINT_COLUMNS = ['wallMonoNs', 'threadCpuNs', 'minflt', 'majflt', 'nvcsw', 'nivcsw']


def point():
  wall, cpu = time.monotonic_ns(), time.thread_time_ns()
  usage = resource.getrusage(resource.RUSAGE_THREAD)
  return wall, cpu, usage.ru_minflt, usage.ru_majflt, usage.ru_nvcsw, usage.ru_nivcsw


class Ring:
  def __init__(self, limit):
    self.limit, self.seen = limit, 0
    self.data = array('q', [0]) * (limit * WIDTH)

  def put(self, values):
    assert len(values) < WIDTH
    position = (self.seen % self.limit) * WIDTH
    self.data[position] = self.seen
    for i in range(WIDTH - 1):
      self.data[position + 1 + i] = int(values[i]) if i < len(values) else 0
    self.seen += 1

  def export(self, path):
    with path.open('xb') as stream:
      self.data.tofile(stream)
    return {
      'path': str(path),
      'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
      'callsSeen': self.seen,
      'rowsRetained': min(self.seen, self.limit),
      'rowsDropped': max(0, self.seen - self.limit),
      'limit': self.limit,
    }


class Probe:
  def __init__(self, services):
    assert len(services) <= 64
    self.services = services
    self.ids = {name: i for i, name in enumerate(services)}
    self.attempts = dict.fromkeys(services, 0)
    self.first, self.outliers, self.sleeps = Ring(32), Ring(16), Ring(32)
    self.sleep_calls, self.errors = 0, []
    self.sleep_kinds = dict.fromkeys(range(3), 0)
    self.dispatch_calls = 0
    self.last_loop = point()
    self.gc = Ring(64)
    self.gc_callback = self.observe_gc
    gc.callbacks.append(self.gc_callback)

  def observe_gc(self, phase, info):
    try:
      self.gc.put(
        (0 if phase == "start" else 1, info["generation"], time.monotonic_ns(), time.thread_time_ns(), info.get("collected", 0), info.get("uncollectable", 0))
      )
    except Exception as exc:
      self.note_error(exc)

  def note_error(self, exc):
    if len(self.errors) < 16:
      self.errors.append(type(exc).__name__ + ': ' + str(exc))

  def send(self, service, event_index, offset, late, stage, completed, payload_bytes, points):
    try:
      attempt = self.attempts[service]
      self.attempts[service] = attempt + 1
      # -1: pre-guard failure;0: exception in original mixed PubMaster call;
      # 3: completed send (including later unchanged post-guard failure).
      values = (self.ids[service], event_index, offset, late, stage, completed, payload_bytes, *sum((tuple(p) for p in points), ()))
      if self.first.seen < 32:
        self.first.put(values)
      elapsed = points[-1][0] - points[0][0] if len(points) > 1 else 0
      if stage != 3 or late > 20_000_000 or elapsed > 1_000_000:
        self.outliers.put(values)
    except Exception as exc:
      self.note_error(exc)

  def sleep(self, seconds, kind):
    before = point()
    try:
      return time.sleep(seconds)
    finally:
      after = point()
      try:
        self.sleep_calls += 1
        self.sleep_kinds[kind] += 1
        requested = round(seconds * 1e9)
        elapsed = after[0] - before[0]
        if self.sleep_kinds[kind] <= 8 or elapsed - requested > 2_000_000:
          self.sleeps.put((kind, requested, elapsed - requested, *before, *after))
      except Exception as exc:
        self.note_error(exc)

  def loop(self, event_index, offset, late):
    # Capture off-CPU gaps not limited to the explicit sleep syscall.
    current = point()
    previous, self.last_loop = self.last_loop, current
    if current[0] - previous[0] > 5_000_000 or late > 20_000_000:
      try:
        self.sleeps.put((3, event_index, offset, late, *previous, *current))
      except Exception as exc:
        self.note_error(exc)

  def dispatch(self, event_index, dispatched, before, after):
    try:
      self.dispatch_calls += 1
      if self.dispatch_calls <= 4 or after[0] - before[0] > 2_000_000:
        self.sleeps.put((4, event_index, dispatched, *before, *after))
    except Exception as exc:
      self.note_error(exc)

  def export(self, output):
    gc.callbacks.remove(self.gc_callback)
    result = {
      'kind': 'private-peer-timing-probe-v2',
      'services': self.services,
      'pointColumns': POINT_COLUMNS,
      'widthInt64': WIDTH,
      'byteorder': sys.byteorder,
      'scalarBytes': array('q').itemsize,
      'observerErrors': self.errors,
      'sendAttempts': self.attempts,
      'sleepCalls': self.sleep_calls,
      'sleepKinds': self.sleep_kinds,
      'dispatchCalls': self.dispatch_calls,
      'sendColumnsAfterSequence': [
        'serviceIndex',
        'eventIndex',
        'sourceOffsetNs',
        'latenessNs',
        'stage',
        'completedMonoNs',
        'payloadBytes',
        'points:beforeMixedPubMasterSend,afterMixedPubMasterSend',
      ],
      'sleepColumnsAfterSequence': ['kind0release1prestart2yield', 'requestedNs', 'oversleepNs', 'beforePoint', 'afterPoint'],
      'dispatchKind4ColumnsAfterSequence': ['kind4', 'eventIndex', 'dispatched', 'beforePoint', 'afterPoint'],
      'loopGapKind3ColumnsAfterSequence': ['kind3', 'eventIndex', 'offsetNs', 'lateNs', 'previousLoopPoint', 'currentLoopPoint'],
      'gcColumnsAfterSequence': ['phase0start1stop', 'generation', 'wallMonoNs', 'threadCpuNs', 'collected', 'uncollectable'],
      'gcCallbackScope': 'Installed before readiness/release; removed before export; no GC policy change',
      'rings': {
        name: ring.export(output.with_name(output.stem + '-probe-' + name + '.bin'))
        for name, ring in (('first', self.first), ('outliers', self.outliers), ('sleep', self.sleeps), ('gc', self.gc))
      },
      'scope': 'Diagnostic overhead included in original strict20ms delivery budget; no warm-up messages or scheduling/prealloc change',
    }
    path = output.with_name(output.stem + '-probe.json')
    temporary = path.with_suffix('.json.tmp')
    assert not temporary.exists() and not path.exists()
    temporary.write_text(json.dumps(result, indent=2) + '\n')
    temporary.replace(path)
    return str(path)
