"""Actual daemon/native-parser boundary units; declared inputs, no road/fit proof."""
from openpilot.starpilot.flm.tests._drain_fixture import separate_drain_fixture
from dataclasses import replace
from types import SimpleNamespace

import pytest

from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.flm import steerlearnerd
from openpilot.starpilot.flm.fullrate_collector import FullrateCollector
from openpilot.starpilot.flm.runtime_adapter_registry import SourceState, car_params_digest, current_authority
from openpilot.starpilot.flm.tests.test_fullrate_pipeline import snapshots, serialized_control, serialized_output
from openpilot.starpilot.flm.tests.test_native_health import toyota_cp, typed_state

BASE = 600_000_000_000


class Finished(Exception):
  pass


def exercise(monkeypatch, mode):
  now = [BASE + 20_000_000]
  cp = toyota_cp()
  fresh = typed_state(cp, BASE + 10_000_000)
  allowed, _, native_identity = current_authority(fresh, cp, now_ns=now[0], required=True)
  assert allowed
  old = typed_state(cp, BASE - 250_000_000)
  pending = fresh
  if mode == 'denied':
    pending = typed_state(cp, BASE + 10_000_000, denied=True)
  elif mode == 'epoch':
    pending = typed_state(cp, BASE + 10_000_000, session='different-declared-session')
    pending_allowed, _, pending_identity = current_authority(pending, cp, now_ns=now[0], required=True)
    assert pending_allowed and pending_identity != native_identity
  elif mode in ('empty', 'stale'):
    pending = old

  source = SourceState(True, False, '0', '2' * 64, BASE - 10_000_000, BASE + 490_000_000,
    car_params_digest(cp), (0., 0., 0.), 'c' * 32, 'd' * 64)
  reads = []

  class Receiver:
    def __init__(self):
      self.backing = fresh if mode in ('healthy', 'missing_control') else old
      if mode == 'output_first':
        self.backing = typed_state(cp, BASE - 175_000_000)
      self.updated = {'deviceMotion': False, 'aolSafetyWire': False}
      self.motion_header = 0
      self.calls = []
      self.refreshes = 0
      self.control_refreshes = []

    def __getitem__(self, name):
      return self.backing[name]

    def __getattr__(self, name):
      return getattr(self.backing, name)

    @property
    def logMonoTime(self):
      return {**self.backing.logMonoTime, 'deviceMotion': self.motion_header}

    def update(self, timeout=None):
      self.calls.append(timeout)
      self.updated = dict.fromkeys(self.updated, False)
      if timeout == 0:
        self.refreshes += 1
        if self.refreshes == 1 and mode != 'healthy':
          now[0] += 120_000_000 if mode == 'slow_refresh' else 1_000
          self.backing = pending
          self.updated['aolSafetyWire'] = mode != 'empty'
          if mode == 'pose':
            self.motion_header = BASE + 10_000_000
            self.updated['deviceMotion'] = True

  sm = Receiver()

  def reader(stamp):
    reads.append(stamp)
    if sm.refreshes:
      changes = {'off': {'enabled': False}, 'manual': {'manual': True},
        'source_epoch': {'source_epoch': 'e' * 64}, 'calibration': {'calibration_generation': 'e' * 32},
        'cp': {'cp_sha256': 'e' * 64}, 'reset': {'reset': '1'}}
      if mode == 'slow_reader':
        now[0] += 500_000_001
      return replace(source, **changes.get(mode, {}))
    return source

  calls = []
  instances = []

  class Adapter:
    command_domain = 'torque'

    def select_objective(self, key):
      pass

    def prepare_control(self, row):
      return row

  class Collector(FullrateCollector):
    def control(self, value, now_ns):
      accepted = super().control(value, now_ns)
      calls.append(('control', self._learning_current_native_allowed, accepted, now_ns, self.reason))
      return accepted

    def output(self, value, now_ns):
      accepted = super().output(value, now_ns)
      calls.append(('output', self._learning_current_native_allowed, accepted, now_ns, self.reason))
      return accepted

  def factory(adapter):
    instance = Collector(adapter)
    instances.append(instance)
    return instance

  def row(stamp):
    c, o, _ = snapshots(stamp)
    c = replace(c, native_required=True, native_allowed=True, native_identity=native_identity,
      native_observed_ns=BASE - 50_000_000, native_expires_ns=BASE + 150_000_000)
    return serialized_control(c), serialized_output(o)

  control_socket, output_socket = object(), object()
  batches = [0]
  refresh_at_end = []
  next_headers = []
  original_next = steerlearnerd._SourceDelivery.next

  def delivery_next(self, receiver):
    value = original_next(self, receiver)
    next_headers.append(value)
    return value

  def drain(socket, callback, receive, sink, stamp):
    if socket is control_socket:
      batches[0] += 1
      if batches[0] > 1:
        raise Finished
      for cstamp in ((BASE + 2_000_000,) if mode == 'missing_control' else (BASE, BASE + 2_000_000)):
        callback(row(cstamp)[0], sink, stamp)
      sm.control_refreshes.append(sm.refreshes)
    else:
      if mode == 'output_first':
        now[0] = BASE + 30_000_000
      callback(row(BASE)[1], sink, stamp)
      refresh_at_end.append(sm.refreshes)

  actor = SimpleNamespace(ingress=None, certificate=None, reason='collecting', pending=False,
    seed=None, response_recovery_generation=0, tick=lambda *args, **kw: None,
    interrupt=lambda *args, **kw: None, close=lambda *args: True)
  pm = SimpleNamespace(send=lambda *args: None)

  def clock():
    now[0] += 100
    return now[0]

  monkeypatch.setattr(steerlearnerd, 'FullrateCollector', factory)
  monkeypatch.setattr(steerlearnerd, 'drain_pair', separate_drain_fixture(drain))
  monkeypatch.setattr(steerlearnerd._SourceDelivery, 'next', delivery_next)
  with pytest.raises(Finished):
    steerlearnerd.run(cp, sm, control_socket, output_socket, pm, lambda *args: Adapter(),
      lambda *args: None, reader, lambda evidence: ('fixture',), lambda evidence: 'fixture',
      lambda *args: None, None, None, None, clock=clock, live_actor=actor)
  return calls, sm, refresh_at_end, next_headers, reads, instances


@pytest.mark.parametrize('mode', ('fresh', 'pose', 'healthy'))
def test_pending_native_is_consumed_before_first_collector_call(monkeypatch, mode):
  with OpenpilotPrefix():
    calls, sm, refreshes, headers, reads, _instances = exercise(monkeypatch, mode)
  assert [c[:3] for c in calls] == [('control', True, True), ('control', True, True), ('output', True, True)]
  assert refreshes == [0 if mode == 'healthy' else 1]
  assert len(reads) == (3 if mode == 'healthy' else 4)
  # The mandatory final refresh survives. A pending incidental delivery is
  # consumed before another blocking poll, including through an empty final refresh.
  assert sm.calls == ([None, 0, None] if mode == 'healthy' else [None, 0, 0])
  if mode == 'pose':
    assert headers == [0, BASE + 10_000_000]


@pytest.mark.parametrize('mode', ('empty', 'stale', 'denied', 'epoch'))
def test_no_native_authority_is_created_and_budget_spans_both_drains(monkeypatch, mode):
  with OpenpilotPrefix():
    calls, sm, refreshes, _headers, _reads, _instances = exercise(monkeypatch, mode)
  assert len(calls) == 3 and not any(c[1] for c in calls)
  assert refreshes == [1] and sm.refreshes == 2  # One recovery plus mandatory final.


@pytest.mark.parametrize('mode', ('off', 'manual', 'source_epoch', 'calibration', 'cp', 'reset', 'slow_reader'))
def test_refresh_revalidates_source_before_old_collector_is_called(monkeypatch, mode):
  with OpenpilotPrefix():
    calls, _sm, refreshes, _headers, _reads, instances = exercise(monkeypatch, mode)
  assert not calls and len(instances) == 1 and refreshes == [1]
  assert not instances[0].controls and not instances[0].ready


def test_slow_refresh_does_not_backdate_control_admission(monkeypatch):
  with OpenpilotPrefix():
    calls, _sm, refreshes, _headers, _reads, _instances = exercise(monkeypatch, 'slow_refresh')
  controls = [c for c in calls if c[0] == 'control']
  assert controls and all(not c[2] and c[3] - BASE > 100_000_000 and c[4] == 'stale_control' for c in controls)
  outputs = [c for c in calls if c[0] == 'output']
  assert len(outputs) == 1 and not outputs[0][2]
  assert outputs[0][3] - (BASE + 1_000_000) > 100_000_000 and outputs[0][4] == 'stale_applied_output'
  assert refreshes == [1]


def test_missing_control_cannot_trigger_native_recovery(monkeypatch):
  with OpenpilotPrefix():
    calls, sm, refreshes, _headers, _reads, instances = exercise(monkeypatch, 'missing_control')
  assert [c[:3] for c in calls] == [('control', True, True), ('output', False, True)]
  assert len(instances) == 1 and refreshes == [0] and sm.refreshes == 1
  assert not instances[0].ready


def test_output_consumes_pending_native_after_control_drain(monkeypatch):
  with OpenpilotPrefix():
    calls, sm, refreshes, _headers, _reads, _instances = exercise(monkeypatch, 'output_first')
  assert sm.control_refreshes == [0]
  assert [c[:3] for c in calls] == [('control', True, True), ('control', True, True), ('output', True, True)]
  assert refreshes == [1] and sm.calls == [None, 0, 0]
