"""Actual typed retainer/history path on synthetic qualified physical evidence."""
from typing import Any

from dataclasses import replace
from types import SimpleNamespace
import pytest

from openpilot.starpilot.flm.canonical_surface_adapter import NAMES, full_profile_sha256
from openpilot.starpilot.flm.live_trial_store import LiveConfirmedPrior
from openpilot.starpilot.flm.live_tracking_actor import LiveTrackingActor
from openpilot.starpilot.flm.live_tracking_requalification import RetainedResponseRequalifier
from openpilot.starpilot.flm.steerlearnerd import _LiveHistoryRestore
from openpilot.starpilot.flm.tests.test_live_tracking_actor import PollProcess
from openpilot.starpilot.flm.tests.test_live_tracking_requalification import certificate, basis, synthetic_pair


def setup(monkeypatch):
  old = certificate()
  current = replace(old.binding, identity='6'*64, source_epoch='2'*64, calibration_generation='3'*32)
  zero = tuple((name, 0.) for name in NAMES)
  prior = LiveConfirmedPrior(old.binding, zero, full_profile_sha256(old.binding.baseline_revision, dict(zero)),
    '7'*64, 1, old, response_basis=basis(old))
  history = _LiveHistoryRestore((prior,))
  process = PollProcess()
  actor: Any = LiveTrackingActor(process)
  class SnapshotPreparation:
    pid=epoch=token=None
    reason='snapshot_submitted'
    pending=None
    def poll(self,*,token,now_ns):
      value,self.pending=self.pending,None
      self.pid=None
      return value
    def submit(self,collector,seed,*,token,now_ns):
      from openpilot.starpilot.flm.live_tracking_bootstrap_producer import response_snapshot
      self.epoch,self.token,self.pid=collector.original_source_capture.epoch,token,1
      self.pending={'snapshot':response_snapshot(collector,seed,now_ns)}
      return True
  actor.snapshot_process=SnapshotPreparation()

  actor.seed = current
  monkeypatch.setattr(actor, 'binding_for', lambda collector, source: current)
  # Exact provisional numerical binding, never returned as current admission.
  template = RetainedResponseRequalifier(old, basis(old), current)
  collector = SimpleNamespace(ready=[], original_source_capture=SimpleNamespace(rows=[], epoch=1), live_tracking=None,
    adapter=SimpleNamespace(replay=SimpleNamespace(emissions=SimpleNamespace(events={}))))
  return history, actor, collector, template, process


def feed(history, actor, collector, template, index, *, bad=True, native=True, stale=False):
  c, o, p = synthetic_pair(template, index, command=.1)
  if bad:
    yaw = p.yaw_rate+.12/p.speed_mps
    p = replace(p, yaw_rate=yaw, angular_velocity_device=(0., 0., yaw))
  now = p.mono_ns+(100_000_001 if stale else 0)
  collector.ready = [(c, o)]
  collector.original_source_capture.rows = [p]
  history.advance(actor, collector, object(), now_ns=now, current_native=native,
    native_key=(), hardware_inventory=('measured-hardware-type',))
  return now


@pytest.mark.parametrize('already_attempted', (False, True))
def test_new_drive_bad_historical_model_quarantines_then_allows_only_unused_epoch_fit(monkeypatch, already_attempted):
  from openpilot.starpilot.flm import live_tracking_bootstrap_producer
  history, actor, collector, template, process = setup(monkeypatch)
  if already_attempted:
    actor.attempted_epoch = 1
  original_rows = history.priors
  for i in range(60):
    now = feed(history, actor, collector, template, i)
    assert history.blocked_seed is None
    assert actor.certificate is actor.ingress is None
  for i in range(60, 69):
    now = feed(history, actor, collector, template, i)
  assert history.blocked_seed == actor.seed
  assert history.prior is history.requalifier is history.offer is None
  assert history.priors is original_rows
  assert not actor.restore_pending
  snapshots = []
  def snapshot(*args):
    snapshots.append(1)
    return {'scheduling_test_only': True}
  monkeypatch.setattr(live_tracking_bootstrap_producer, 'response_snapshot', snapshot)
  actor.tick(collector, object(), now_ns=now, current_native=True,
    allow_bootstrap=history.requalifier is None)
  actor.tick(collector,object(),now_ns=now+1,current_native=True,allow_bootstrap=history.requalifier is None)
  assert len(process.submissions) == (0 if already_attempted else 1)
  assert actor.certificate is actor.ingress is None
  # A reset cannot bypass source quarantine or promote the old target.
  history.reset()
  feed(history, actor, collector, template, 70)
  assert history.blocked_seed == actor.seed and history.prior is None
  if already_attempted:
    collector.original_source_capture.epoch = 2
    actor.tick(collector, object(), now_ns=now+2, current_native=True)
    actor.tick(collector, object(), now_ns=now+3, current_native=True)
    assert len(process.submissions) == 1


@pytest.mark.parametrize('failure', ('transient', 'stale', 'native'))
def test_nonphysical_or_transient_failure_never_quarantines_history(failure, monkeypatch):
  history, actor, collector, template, _ = setup(monkeypatch)
  for i in range(80):
    feed(history, actor, collector, template, i, bad=(failure != 'transient' or i == 0),
      native=(failure != 'native'), stale=(failure == 'stale'))
    assert history.blocked_seed is None
  assert actor.certificate is actor.ingress is None


def test_permission_loss_clears_bad_observation_count_before_fresh_requalification(monkeypatch):
  history, actor, collector, template, _ = setup(monkeypatch)
  for i in range(55):
    feed(history, actor, collector, template, i)
  assert history.requalifier is not None
  feed(history, actor, collector, template, 55, native=False)
  assert not history.requalifier.persistent_residual_failure(12_750_000_000)
  for i in range(56, 80):
    feed(history, actor, collector, template, i)
    assert history.blocked_seed is None


def test_source_pose_gap_requires_new_full_persistent_failure_window(monkeypatch):
  history, actor, collector, template, _ = setup(monkeypatch)
  for i in range(55):
    feed(history, actor, collector, template, i)
  assert history.blocked_seed is None
  # Genuine next source sample is200ms later; no intermediate samples invented.
  for i in range(58, 83):
    feed(history, actor, collector, template, i)
    assert history.blocked_seed is None
  assert history.requalifier is not None
  assert not history.requalifier.persistent_residual_failure(14_100_000_000)
