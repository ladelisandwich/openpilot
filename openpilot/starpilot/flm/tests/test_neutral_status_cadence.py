"""Neutral source transport cadence is independent of physical witness cadence."""
from types import SimpleNamespace
import pytest
from openpilot.cereal.messaging import FrequencyTracker
from openpilot.starpilot.flm import steerlearnerd
from openpilot.starpilot.flm.tests.test_live_tracking_daemon import BASE, LoopComplete, NoLegacyFitter, RecordingActor, source

@pytest.mark.parametrize('renew_settings', [False, True])
def test_neutral_status_meets_declared_frequency_without_renewing_physical_proof(monkeypatch, renew_settings):
  now = [BASE]
  sends = []
  fixed = source(BASE)
  class SM:
    updated = {'deviceMotion': False}
    logMonoTime = {'deviceMotion': 0}
    steps = 0
    def update(self, timeout=None):
      self.steps += 1
      if self.steps > 220:
        raise LoopComplete
      now[0] += 10_000_000
      self.updated['deviceMotion'] = self.steps % 7 == 0
      if self.updated['deviceMotion']:
        self.logMonoTime['deviceMotion'] = now[0]
  def send(topic, message):
    if topic == 'starpilotSteeringTracking':
      sends.append((now[0], message.as_reader()))
  monkeypatch.setattr(steerlearnerd, 'drain_pair', lambda *args: None)
  actor, fitter = RecordingActor(), NoLegacyFitter()
  fitter.cancel = lambda reason: None
  actor.interrupt = lambda reason, **kwargs: None
  with pytest.raises(LoopComplete):
    steerlearnerd.run(SimpleNamespace(), SM(), object(), object(), SimpleNamespace(send=send),
      None, None, lambda stamp: source(stamp) if renew_settings else fixed, None, None, None, None, fitter, None,
      clock=lambda: now[0], live_actor=actor)
  assert sends
  assert all(m.starpilotSteeringTracking.trial.version == 0 and m.starpilotSteeringTracking.restore.version == 0 for _, m in sends)
  if renew_settings:
    frequency = FrequencyTracker(20, 100, False)
    validity = []
    for stamp, message in sends:
      frequency.record_recv_time(stamp / 1e9)
      validity.append(frequency.valid)
      assert message.starpilotSteeringTracking.sourceReceipt.validUntilMonoTime <= stamp + 500_000_000
    assert len(sends) >= 40
    assert all(validity[2:])
  else:
    assert len(sends) >= 8
    assert all(stamp <= fixed.expires_ns for stamp, _ in sends)
    assert all(m.starpilotSteeringTracking.sourceReceipt.observedMonoTime == fixed.observed_ns and
               m.starpilotSteeringTracking.sourceReceipt.validUntilMonoTime == fixed.expires_ns for _, m in sends)
  assert actor.closed and fitter.closed


def test_explicit_learning_off_still_publishes_withdrawal_on_the_next_source_packet(monkeypatch):
  from dataclasses import replace
  now, sends = [BASE - 50_000_000], []
  class SM:
    updated = {'deviceMotion': False}
    logMonoTime = {'deviceMotion': 0}
    steps = 0
    def update(self, timeout=None):
      self.steps += 1
      if self.steps > 6:
        raise LoopComplete
      now[0] += 50_000_000
      self.updated['deviceMotion'] = True
      self.logMonoTime['deviceMotion'] = now[0]
  actor, fitter = RecordingActor(), NoLegacyFitter()
  actor.interrupt = lambda reason, **kwargs: None
  fitter.cancel = lambda reason: None
  def current(stamp):
    return source(stamp) if stamp < BASE + 100_000_000 else replace(source(stamp), enabled=False, source_epoch='')
  def send(topic, message):
    if topic == 'starpilotSteeringTracking':
      sends.append((now[0], message.as_reader()))
  monkeypatch.setattr(steerlearnerd, 'drain_pair', lambda *args: None)
  with pytest.raises(LoopComplete):
    steerlearnerd.run(SimpleNamespace(), SM(), object(), object(), SimpleNamespace(send=send),
      None, None, current, None, None, None, None, fitter, None, clock=lambda: now[0], live_actor=actor)
  withdrawn = [(stamp, msg) for stamp, msg in sends if stamp >= BASE + 100_000_000]
  assert withdrawn and withdrawn[0][0] == BASE + 100_000_000
  assert all(not msg.starpilotSteeringTracking.sourceReceipt.enabled and
             msg.starpilotSteeringTracking.trial.version == 0 and msg.starpilotSteeringTracking.restore.version == 0
             for _, msg in withdrawn)
  assert actor.closed and fitter.closed


def test_missing_source_does_not_enter_the_explicit_withdrawal_serializer(monkeypatch):
  now, sends = [BASE - 50_000_000], []
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
  actor, fitter = RecordingActor(), NoLegacyFitter()
  actor.interrupt = lambda reason, **kwargs: None
  fitter.cancel = lambda reason: None
  def current(stamp):
    return source(stamp) if stamp < BASE + 100_000_000 else None
  def send(topic, message):
    if topic == 'starpilotSteeringTracking':
      sends.append((now[0], message.as_reader()))
  monkeypatch.setattr(steerlearnerd, 'drain_pair', lambda *args: None)
  with pytest.raises(LoopComplete):
    steerlearnerd.run(SimpleNamespace(), SM(), object(), object(), SimpleNamespace(send=send),
      None, None, current, None, None, None, None, fitter, None, clock=lambda: now[0], live_actor=actor)
  assert sends and all(stamp < BASE + 100_000_000 for stamp, _ in sends)
  assert all(msg.starpilotSteeringTracking.trial.version == 0 and msg.starpilotSteeringTracking.restore.version == 0 for _, msg in sends)
  assert actor.closed and fitter.closed
