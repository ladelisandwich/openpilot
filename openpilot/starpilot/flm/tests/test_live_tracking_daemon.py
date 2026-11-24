"""Actual daemon loop and cereal publication; fake actor does not model permission."""
from openpilot.starpilot.flm.tests._drain_fixture import separate_drain_fixture

from dataclasses import replace
from types import SimpleNamespace
import pytest
from collections.abc import Callable
from typing import NoReturn, cast
from openpilot.starpilot.flm import steerlearnerd
from openpilot.starpilot.flm.runtime_adapter_registry import SourceState
from openpilot.starpilot.flm.tests.test_live_trial_owner import offer

BASE = 10_000_000_000


def source(now):
  return SourceState(True, False, 'a' * 64, 'b' * 64, now, now + 500_000_000, 'c' * 64, (0.0, 0.0, 0.0), 'd' * 32, 'e' * 64)


class NoLegacyFitter:
  def __init__(self):
    self.closed = False

  def poll(self, **kwargs):
    cast(Callable[[str], NoReturn], pytest.fail)('live daemon must never poll legacy full FD fitter')

  def submit(self, *args, **kwargs):
    cast(Callable[[str], NoReturn], pytest.fail)('live daemon must never submit legacy full FD fitter')

  @property
  def child(self):
    cast(Callable[[str], NoReturn], pytest.fail)('live daemon must never inspect legacy fitter child')

  def close(self, reason):
    self.closed = True


class RecordingActor:
  reason = 'production_bootstrap_budget_not_yet_qualified'
  ingress = None
  pending = False
  certificate = None

  def __init__(self):
    self.calls = []
    self.closed = False

  def tick(self, collector, current_source, **kwargs):
    self.calls.append((collector, current_source, kwargs))
    return None

  def persistence_inputs(self, now_ns):
    return None

  def close(self, reason):
    self.closed = True


class LoopComplete(Exception):
  pass


@pytest.mark.parametrize('shutdown_reaped', (True, False))
def test_actual_daemon_publishes_separate20_and4_without_legacy_fit(monkeypatch, shutdown_reaped):
  now = [BASE - 50_000_000]

  class SM:
    updated = {'deviceMotion': False}
    logMonoTime = {'deviceMotion': 0}

    def update(self, timeout=None):
      now[0] += 50_000_000
      if now[0] > BASE + 1_000_000_000:
        raise LoopComplete
      self.updated['deviceMotion'] = True
      self.logMonoTime['deviceMotion'] = now[0]

  sent = []
  pm = SimpleNamespace(send=lambda topic, message: sent.append((now[0], topic, message.as_reader())))
  actor, fitter = RecordingActor(), NoLegacyFitter()
  original_close = actor.close

  def actor_close(reason):
    original_close(reason)
    return shutdown_reaped

  monkeypatch.setattr(actor, 'close', actor_close)
  # No control/native is invented: this exercises actual collecting publications.
  monkeypatch.setattr(steerlearnerd, 'drain_pair', lambda *args: None)
  with pytest.raises(LoopComplete if shutdown_reaped else RuntimeError):
    steerlearnerd.run(
      SimpleNamespace(),
      SM(),
      object(),
      object(),
      pm,
      None,
      None,
      lambda stamp: source(stamp),
      None,
      None,
      None,
      None,
      fitter,
      None,
      clock=lambda: now[0],
      live_actor=actor,
    )
  tracking = [(stamp, msg) for stamp, topic, msg in sent if topic == 'starpilotSteeringTracking']
  legacy = [(stamp, msg) for stamp, topic, msg in sent if topic == 'autoSteeringLearning']
  assert len(tracking) == 21 and len(legacy) == 5
  assert all(b[0] - a[0] == 50_000_000 for a, b in zip(tracking, tracking[1:], strict=False))
  assert all(b[0] - a[0] == 250_000_000 for a, b in zip(legacy, legacy[1:], strict=False))
  assert all(msg.starpilotSteeringTracking.version == 1 and msg.starpilotSteeringTracking.trial.version == 0 for _, msg in tracking)
  assert all(msg.autoSteeringLearning.learning.status == 'collecting' for _, msg in legacy)
  assert all(call[0] is None and call[2]['current_native'] is False for call in actor.calls)
  assert actor.closed and fitter.closed


def test_tracking_serialization_never_renews_existing_proof_or_fabricates_native():
  pending = offer(BASE)
  initial = steerlearnerd.tracking_message(source(BASE), now_ns=BASE, current_native=True, offer=pending)
  renewed = steerlearnerd.tracking_message(source(BASE + 50_000_000), now_ns=BASE + 50_000_000, current_native=True, offer=pending)
  for message in (initial, renewed):
    row = message.starpilotSteeringTracking.trial
    assert row.observedMonoTime == pending.observed_ns and row.validUntilMonoTime == pending.expires_ns
    assert row.deadlineMonoTime == pending.deadline_ns and row.sequence == pending.sequence
  expired = steerlearnerd.tracking_message(source(BASE + 110_000_000), now_ns=BASE + 110_000_000, current_native=True, offer=pending)
  denied = steerlearnerd.tracking_message(source(BASE), now_ns=BASE, current_native=False, offer=pending)
  manual = steerlearnerd.tracking_message(replace(source(BASE), manual=True), now_ns=BASE, current_native=True, offer=pending)
  assert all(message.starpilotSteeringTracking.trial.version == 0 for message in (expired, denied, manual))


def _run_poll_script(monkeypatch, rows, *, expire_during_drain=False):
  now = [BASE]

  class SM:
    def __init__(self):
      self.updated = {'deviceMotion': False}
      self.logMonoTime = {'deviceMotion': 0}
      self.valid = {'deviceMotion': False}
      self.index = self.zero_polls = 0

    def update(self, timeout=None):
      if timeout == 0:
        self.zero_polls += 1
        self.updated['deviceMotion'] = False
        return
      if self.index == len(rows):
        raise LoopComplete
      stamp, header, valid = rows[self.index]
      self.index += 1
      now[0] = stamp
      self.updated['deviceMotion'] = header is not None
      if header is not None:
        self.logMonoTime['deviceMotion'] = header
        self.valid['deviceMotion'] = valid

  sm = SM()
  actor, fitter = RecordingActor(), NoLegacyFitter()
  sent = []
  pm = SimpleNamespace(send=lambda topic, message: sent.append((now[0], topic, message.as_reader())))
  expired = False

  def drain(*args):
    nonlocal expired
    if expire_during_drain and not expired:
      # Trigger the REAL source_current expiry path and its update(0). No
      # physical/native/controller evidence is injected into this loop test.
      now[0] += 600_000_000
      expired = True

  monkeypatch.setattr(steerlearnerd, 'drain_pair', separate_drain_fixture(drain))
  with pytest.raises(LoopComplete):
    steerlearnerd.run(SimpleNamespace(), sm, object(), object(), pm, None, None,
      lambda stamp: source(stamp), None, None, None, None, fitter, None,
      clock=lambda: now[0], live_actor=actor)
  assert actor.closed and fitter.closed
  assert all(call[0] is None and call[2]['current_native'] is False for call in actor.calls)
  return sent, sm


def _tracking_frequency(sent):
  from openpilot.cereal.messaging import FrequencyTracker
  tracker = FrequencyTracker(20., 100., False)  # Actual Controls receiver: selfdriveState100Hz poll.
  for stamp, topic, _message in sent:
    if topic == 'starpilotSteeringTracking':
      tracker.record_recv_time(stamp/1e9)
  return tracker


def test_actual_run_tracking_survives_alternating_poll_jitter(monkeypatch):
  # Existing reset-to-now >=50ms gate drops every49ms interval, yielding10Hz.
  # These are21 genuine20Hz source deliveries with realistic poll jitter.
  offsets = [0]
  for index in range(20):
    offsets.append(offsets[-1]+(49_000_000 if index % 2 == 0 else 51_000_000))
  rows = tuple((BASE+offset, BASE+offset, True) for offset in offsets)
  sent, _sm = _run_poll_script(monkeypatch, rows)
  tracking = [(stamp, message) for stamp, topic, message in sent if topic == 'starpilotSteeringTracking']
  assert [stamp for stamp, _ in tracking] == [stamp for stamp, _header, _valid in rows]
  assert _tracking_frequency(sent).valid
  assert all(message.valid and message.starpilotSteeringTracking.version == 1 and
    message.starpilotSteeringTracking.trial.version == 0 for _, message in tracking)


def test_actual_run_neutral_status_does_not_promote_duplicate_regressed_future_or_missing_input(monkeypatch):
  rows = (
    (BASE, BASE, True),
    (BASE+49_000_000, BASE+49_000_000, True),
    (BASE+100_000_000, BASE+100_000_000, True),
    (BASE+149_000_000, BASE+100_000_000, True),  # Actual duplicate header.
    (BASE+200_000_000, None, False),  # Poll timeout, no new source delivery.
    (BASE+250_000_000, BASE+90_000_000, True),  # Regressed source header.
    (BASE+300_000_000, BASE+350_000_000, True),  # Future source header.
    (BASE+350_000_000, 0, False),
    (BASE+400_000_000, BASE+400_000_000, False),  # Genuine invalid raw packet: neutral status remains observable.
    (BASE+449_000_000, BASE+449_000_000, True),
  )
  sent, _sm = _run_poll_script(monkeypatch, rows)
  tracking = [(stamp, message) for stamp, topic, message in sent if topic == 'starpilotSteeringTracking']
  assert [stamp for stamp, _ in tracking] == [stamp for stamp, _header, _valid in rows]
  assert _tracking_frequency(sent).valid  # Settings transport health is independent of physical pose delivery.
  assert all(message.starpilotSteeringTracking.trial.version == 0 and
             message.starpilotSteeringTracking.restore.version == 0 and
             message.starpilotSteeringTracking.sourceReceipt.observedMonoTime == stamp and
             message.starpilotSteeringTracking.sourceReceipt.validUntilMonoTime == stamp + 500_000_000
             for stamp, message in tracking)


def test_actual_run_captures_polled_header_before_source_current_zero_poll(monkeypatch):
  sent, sm = _run_poll_script(monkeypatch, ((BASE, BASE, True),), expire_during_drain=True)
  assert sm.zero_polls >= 1 and sm.updated['deviceMotion'] is False
  tracking = [(stamp, message) for stamp, topic, message in sent if topic == 'starpilotSteeringTracking']
  assert len(tracking) == 1 and tracking[0][0] == BASE+600_000_000
  assert tracking[0][1].starpilotSteeringTracking.trial.version == 0


def test_actual_nonlive_run_does_not_read_tracking_poll_metadata(monkeypatch):
  now = [BASE-50_000_000]

  class SM:
    # Ordinary SubMaster delivery metadata is required; no tracking-service metadata exists.
    updated = {'deviceMotion': False}
    def update(self, timeout=None):
      now[0] += 50_000_000
      if now[0] > BASE+100_000_000:
        raise LoopComplete

  # This pre-existing run API does not have a live actor or a tracking service.
  # Minimal SM intentionally lacks tracking metadata; the legacy path is intact.
  sent = []
  pm = SimpleNamespace(send=lambda topic, message: sent.append(topic))
  monkeypatch.setattr(steerlearnerd, 'drain_pair', lambda *args: None)
  fitter = SimpleNamespace(poll=lambda **kwargs: None, close=lambda reason: True, child=None)
  with pytest.raises(LoopComplete):
    steerlearnerd.run(SimpleNamespace(), SM(), object(), object(), pm, None, None,
      lambda stamp: source(stamp), None, None, None, None, fitter, None, clock=lambda: now[0])
  assert sent == ['autoSteeringLearning']
