"""Actual daemon publication + ordinary receiver flags; no controller authority."""

from dataclasses import replace
from types import SimpleNamespace
import json

import pytest

from openpilot.cereal import messaging
from openpilot.starpilot.flm import steerlearnerd
from openpilot.starpilot.flm.tests.test_live_tracking_daemon import (
  BASE,
  LoopComplete,
  NoLegacyFitter,
  RecordingActor,
  source,
)

TOPIC = 'autoSteeringLearning'


def _run_regular(monkeypatch, offsets, *, mode, delay_first_publication=0):
  now = [BASE]
  reads = []

  class SM:
    def __init__(self):
      self.index = 0
      self.updated = {'deviceMotion': False}
      self.valid = {'deviceMotion': True}
      self.logMonoTime = {'deviceMotion': 0}

    def update(self, timeout=None):
      if timeout == 0:
        self.updated['deviceMotion'] = False
        return
      if self.index == len(offsets):
        raise LoopComplete
      now[0] = BASE + offsets[self.index]
      self.index += 1
      self.updated['deviceMotion'] = True
      self.logMonoTime['deviceMotion'] = now[0]

  sent = []
  pm = SimpleNamespace(send=lambda topic, message: sent.append((now[0], topic, message.as_reader())))
  actor = RecordingActor() if mode == 'live' else None
  fitter = (
    NoLegacyFitter() if actor is not None else SimpleNamespace(child=None, poll=lambda **kwargs: None, cancel=lambda reason: None, close=lambda reason: True)
  )

  def read(stamp):
    reads.append(stamp)
    value = source(stamp)
    return replace(value, enabled=False) if mode == 'off' else value

  publication_reads = 0

  def publication_read(stamp):
    nonlocal publication_reads
    publication_reads += 1
    if publication_reads == 1:
      now[0] += delay_first_publication
    return read(now[0])

  monkeypatch.setattr(steerlearnerd, 'drain_pair', lambda *args: None)
  with pytest.raises(LoopComplete):
    steerlearnerd.run(
      SimpleNamespace(),
      SM(),
      object(),
      object(),
      pm,
      None,
      None,
      read,
      None,
      None,
      None,
      None,
      fitter,
      None,
      clock=lambda: now[0],
      publication_source_reader=publication_read,
      live_actor=actor,
    )
  if actor is not None:
    assert actor.closed and fitter.closed
  return [(stamp, message) for stamp, topic, message in sent if topic == TOPIC], reads


def _ordinary_receiver(monkeypatch, publications):
  # Actual Controls topology: selfdriveState100Hz poll, learning service4Hz.
  # No ignore_alive/valid/frequency options, simulation bypass or padded msgs.
  monkeypatch.delenv('SIMULATION', raising=False)
  receiver = messaging.SubMaster(['selfdriveState', TOPIC], poll='selfdriveState')
  assert receiver.ignore_alive == receiver.ignore_valid == receiver.ignore_average_freq == []
  assert not receiver.simulation
  for stamp, message in publications:
    receiver.update_msgs(stamp / 1e9, [message])
  return receiver


@pytest.mark.parametrize('mode', ('live', 'nonlive', 'off'))
def test_actual_regular_run_preserves_four_hz_phase_under_49_51_ms_poll_jitter(monkeypatch, mode):
  offsets = [0]
  for index in range(200):
    offsets.append(offsets[-1] + (49_000_000 if index % 2 == 0 else 51_000_000))
  publications, reads = _run_regular(monkeypatch, offsets, mode=mode)
  receiver = _ordinary_receiver(monkeypatch, publications)
  tracker = receiver.freq_tracker[TOPIC]
  average = 1 / tracker.avg_dt.get_average()
  summary = {
    'mode': mode,
    'count': len(publications),
    'average_hz': average,
    'alive': receiver.alive[TOPIC],
    'valid': receiver.valid[TOPIC],
    'freq_ok': receiver.freq_ok[TOPIC],
    'stamps': [stamp for stamp, _message in publications],
  }
  print('ACTUAL_REGULAR_PUBLICATION_CADENCE ' + json.dumps(summary, sort_keys=True))
  assert len(publications) == 41, summary
  assert average == pytest.approx(4.0, abs=1e-10), summary
  assert receiver.all_checks([TOPIC]), summary
  assert len({stamp for stamp, _message in publications}) == len(publications)
  assert all(stamp in {BASE + offset for offset in offsets} for stamp, _message in publications)
  for stamp, message in publications:
    row = message.autoSteeringLearning.learning
    assert stamp in reads and message.logMonoTime == stamp
    assert row.observedMonoTime == stamp and row.validUntilMonoTime == stamp + 500_000_000
    assert row.sourceReceipt.observedMonoTime == stamp
    assert row.sourceReceipt.validUntilMonoTime == stamp + 500_000_000
    assert row.status == ('collecting' if mode == 'live' else 'unavailable')


def test_actual_regular_run_skips_missed_periods_without_burst_or_fake_receiver_health(monkeypatch):
  offsets = (0, 49_000_000, 100_000_000, 600_000_000, 649_000_000, 700_000_000, 749_000_000, 800_000_000, 1_000_000_000)
  publications, _reads = _run_regular(monkeypatch, offsets, mode='live')
  assert [stamp - BASE for stamp, _message in publications] == [0, 600_000_000, 800_000_000, 1_000_000_000]
  assert len({stamp for stamp, _message in publications}) == len(publications)
  receiver = _ordinary_receiver(monkeypatch, publications)
  assert not receiver.freq_ok[TOPIC] and not receiver.all_checks([TOPIC])
  for stamp, message in publications:
    row = message.autoSteeringLearning.learning
    assert message.logMonoTime == row.observedMonoTime == stamp
    assert row.validUntilMonoTime == stamp + 500_000_000


def test_actual_regular_run_skips_periods_spent_inside_current_source_read(monkeypatch):
  offsets = (0, 649_000_000, 700_000_000, 749_000_000, 800_000_000, 1_000_000_000)
  publications, reads = _run_regular(monkeypatch, offsets, mode='live', delay_first_publication=600_000_000)
  assert [stamp - BASE for stamp, _message in publications] == [600_000_000, 800_000_000, 1_000_000_000]
  for stamp, message in publications:
    row = message.autoSteeringLearning.learning
    assert stamp in reads and message.logMonoTime == row.observedMonoTime == stamp
    assert row.validUntilMonoTime == stamp + 500_000_000
    assert row.sourceReceipt.observedMonoTime == stamp
    assert row.sourceReceipt.validUntilMonoTime == stamp + 500_000_000


def test_final_native_refresh_is_processed_before_another_blocking_motion_poll():
  from openpilot.starpilot.flm.steerlearnerd import _SourceDelivery

  class SM:
    def __init__(self):
      self.updated = {'deviceMotion': False, 'pandaStates': False}
      self.logMonoTime = {'deviceMotion': 0, 'pandaStates': 0}
      self.bodies = {}
      self.calls = []

    def update(self, timeout=None):
      self.calls.append(timeout)
      self.updated = dict.fromkeys(self.updated, False)
      stamp = 100 + len(self.calls) * 50
      self.updated['deviceMotion'] = True
      self.logMonoTime['deviceMotion'] = stamp
      self.bodies['deviceMotion'] = ('original', stamp)

  old = SM()
  old.update()
  first = old.logMonoTime['deviceMotion']
  old.update(0)
  skipped = old.logMonoTime['deviceMotion']
  old.update()
  assert old.logMonoTime['deviceMotion'] != skipped
  sm = SM()
  delivery = _SourceDelivery()
  assert delivery.next(sm) == first
  delivery.refresh(sm)
  body = sm.bodies['deviceMotion']
  flags = sm.updated.copy()
  header = sm.logMonoTime.copy()
  assert delivery.next(sm) == skipped
  assert sm.calls == [None, 0]
  assert sm.bodies['deviceMotion'] is body and sm.updated == flags and sm.logMonoTime == header
  assert delivery.next(sm) == 250
  assert sm.calls == [None, 0, None]


def test_empty_incidental_refresh_cannot_erase_a_real_pending_pose_or_create_new_health():
  from types import SimpleNamespace
  from openpilot.starpilot.flm.steerlearnerd import _SourceDelivery

  sm = SimpleNamespace(updated={'deviceMotion': False, 'pandaStates': False}, logMonoTime={'deviceMotion': 100})
  calls = []

  def update(timeout=None):
    calls.append(timeout)
    sm.updated = {'deviceMotion': len(calls) == 1, 'pandaStates': False}
    if len(calls) == 1:
      sm.logMonoTime['deviceMotion'] = 150

  sm.update = update
  delivery = _SourceDelivery()
  delivery.refresh(sm)
  delivery.refresh(sm)
  assert not sm.updated['deviceMotion']
  assert delivery.next(sm) == 150
  assert calls == [0, 0] and not any(sm.updated.values())
  # The saved ORIGINAL header was consumed once, not renewed on empty input.
  assert delivery.next(sm) == 0
  assert calls == [0, 0, None]


def test_native_only_refresh_is_consumed_once_without_inventing_a_motion_delivery():
  from types import SimpleNamespace
  from openpilot.starpilot.flm.steerlearnerd import _SourceDelivery

  sm = SimpleNamespace(updated={'deviceMotion': False, 'pandaStates': False}, logMonoTime={'deviceMotion': 100})
  calls = []

  def update(timeout=None):
    calls.append(timeout)
    sm.updated = {'deviceMotion': False, 'pandaStates': len(calls) == 1}

  sm.update = update
  delivery = _SourceDelivery()
  delivery.refresh(sm)
  flags = sm.updated.copy()
  assert delivery.next(sm) == 0 and sm.updated == flags
  assert calls == [0]
  assert delivery.next(sm) == 0 and calls == [0, None]
