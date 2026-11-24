"""Actual daemon status delivery order; no native or physical authority is created."""
from types import SimpleNamespace

import pytest

from openpilot.starpilot.flm import steerlearnerd
from openpilot.starpilot.flm.tests.test_live_tracking_daemon import (
  BASE, LoopComplete, NoLegacyFitter, RecordingActor, source,
)
from openpilot.starpilot.flm.tests.test_live_trial_owner import offer


@pytest.mark.parametrize('new_offer', (False, True))
def test_real_pose_status_precedes_optional_work_and_late_writes_use_current_clock(monkeypatch, new_offer):
  now = [BASE - 50_000_000]
  sends, writes = [], []

  class SM:
    updated = {'deviceMotion': False}
    logMonoTime = {'deviceMotion': 0}
    steps = 0

    def update(self, timeout=None):
      self.steps += 1
      if self.steps > 3:
        raise LoopComplete
      now[0] += 50_000_000
      self.updated['deviceMotion'] = True
      self.logMonoTime['deviceMotion'] = now[0]

  actor = RecordingActor()
  original_writer = steerlearnerd.tracking_message

  def write_source(*args, **kwargs):
    writes.append((kwargs['now_ns'], now[0]))
    return original_writer(*args, **kwargs)

  def tick(collector, current_source, **kwargs):
    if not actor.calls:
      assert len(sends) == 1, 'Genuine source status must not wait for optional numerical preparation'
      assert sends[0][1].starpilotSteeringTracking.trial.version == 0
      assert sends[0][1].starpilotSteeringTracking.restore.version == 0
    actor.calls.append((collector, current_source, kwargs))
    now[0] += 100_000_000  # Declared optional-work delay, not a changed message timestamp.
    return offer(BASE) if new_offer else None

  def send(topic, message):
    if topic == 'starpilotSteeringTracking':
      sends.append((now[0], message.as_reader()))

  monkeypatch.setattr(actor, 'tick', tick)
  monkeypatch.setattr(steerlearnerd, 'tracking_message', write_source)
  monkeypatch.setattr(steerlearnerd, 'drain_pair', lambda *args: None)
  fitter = NoLegacyFitter()
  with pytest.raises(LoopComplete):
    steerlearnerd.run(SimpleNamespace(), SM(), object(), object(), SimpleNamespace(send=send),
      None, None, lambda stamp: source(stamp), None, None, None, None, fitter, None,
      clock=lambda: now[0], live_actor=actor)
  assert writes and all(passed == actual for passed, actual in writes)
  assert all(message.starpilotSteeringTracking.trial.version == 0 for _, message in sends)
  assert actor.closed and fitter.closed
