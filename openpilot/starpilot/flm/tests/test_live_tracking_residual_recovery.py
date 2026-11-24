"""Fresh physical drift recovery, without new numerical admission or permission."""
from dataclasses import replace
from types import SimpleNamespace

from openpilot.starpilot.flm.live_tracking_response import ResponseMonitor
from openpilot.starpilot.flm.live_tracking_actor import LiveTrackingActor
from openpilot.starpilot.flm.tests.test_live_tracking_requalification import certificate, synthetic_pair


def feed(monitor, index, *, bad=False, current_native=True):
  c, o, p = synthetic_pair(SimpleNamespace(_binding=monitor.certificate.binding, original=monitor.certificate), index)
  if bad:
    p = replace(p, yaw_rate=p.yaw_rate+.12/p.speed_mps)
  return monitor.observe(c, o, p, now_ns=p.mono_ns, binding=monitor.certificate.binding, current_native=current_native), p.mono_ns


def test_one_bad_row_and_missing_permission_do_not_request_reidentification():
  monitor = ResponseMonitor(certificate())
  for i in range(10):
    assert feed(monitor, i)[0]
  assert feed(monitor, 10, bad=True)[0]
  for i in range(11, 100):
    accepted, now = feed(monitor, i)
    assert accepted
    assert not monitor.persistent_residual_failure(now)
  assert not feed(monitor, 100, bad=True, current_native=False)[0]
  assert not monitor.persistent_residual_failure(now+50_000_000)


def test_distinct_fresh_physical_residual_failure_needs_full_persistent_window():
  monitor = ResponseMonitor(certificate())
  for i in range(60):
    accepted, now = feed(monitor, i, bad=True)
    assert accepted
    assert not monitor.persistent_residual_failure(now)
  for i in range(60, 69):
    accepted, now = feed(monitor, i, bad=True)
    assert accepted
  assert monitor.persistent_residual_failure(now)
  assert monitor.witness(now) is None
  assert not monitor.persistent_residual_failure(now+100_000_001)
  assert not feed(monitor, 69, bad=True, current_native=False)[0]
  assert not monitor.persistent_residual_failure(now+50_000_000)


def test_withdraw_once_cancel_probe_no_same_epoch_retry_then_new_epoch(monkeypatch):
  from openpilot.starpilot.flm.tests.test_live_tracking_actor import PollProcess
  from openpilot.starpilot.flm import live_tracking_bootstrap_producer
  b = certificate().binding
  process, probe = PollProcess(), PollProcess(reap_after=2)
  process.cancel = lambda reason: None
  cancellations = []
  probe.cancel = cancellations.append
  actor = LiveTrackingActor(process, probe_process=probe)
  actor.seed = b
  monkeypatch.setattr(actor, 'binding_for', lambda collector, source: b)
  monitor = ResponseMonitor(certificate())
  for i in range(69):
    _, now = feed(monitor, i, bad=True)
  interrupted = []
  monkeypatch.setattr(actor, 'ingress', SimpleNamespace(monitor=monitor, optimizer=SimpleNamespace(offer_state=None),
    interrupt=lambda reason, **kwargs: interrupted.append(reason)))
  actor.certificate = monitor.certificate
  from openpilot.starpilot.flm.runtime_adapter_registry import SourceState
  from openpilot.starpilot.flm.torque_emission_replay import EmissionHistory
  source = SourceState(True, False, b.reset_generation, b.source_epoch, now, now+500_000_000,
    b.cp_sha256, (), b.calibration_generation, b.persistent_source_epoch)
  collector = SimpleNamespace(ready=[None], adapter=SimpleNamespace(
    replay=SimpleNamespace(raw_mode=False, emissions=EmissionHistory(None))),
    original_source_capture=SimpleNamespace(epoch=1), live_tracking=actor.ingress)
  snapshots = []
  def snapshot(*args):
    snapshots.append(1)
    return {'lifecycle_only': True}
  monkeypatch.setattr(live_tracking_bootstrap_producer, 'response_snapshot', snapshot)
  class SnapshotPreparation:
    pid: int | None = None
    epoch: int | None = None
    token: tuple[object, ...] | None = None
    pending: dict[str, object] | None = None
    reason = 'snapshot_submitted'

    def cancel(self, reason):
      self.pid = self.pending = None

    def poll(self, *, token, now_ns):
      result, self.pending = self.pending, None
      self.pid = None
      return result if token == self.token else None

    def submit(self, collector, seed, *, token, now_ns):
      self.epoch, self.token, self.pid = collector.original_source_capture.epoch, token, 1
      self.pending = {'snapshot': live_tracking_bootstrap_producer.response_snapshot(collector, seed, now_ns)}
      return True

  # Lifecycle-only asynchronous preparation seam; source/decoder/monitor and
  # current bootstrap admission decisions remain actual production code.
  monkeypatch.setattr(actor, 'snapshot_process', SnapshotPreparation())
  actor.tick(collector, source, now_ns=now, current_native=True)
  assert actor.ingress is actor.certificate is collector.live_tracking is None
  assert actor.attempted_epoch == actor.response_recovery_generation == 1
  assert interrupted == cancellations == ['persistent_physical_response_drift']
  actor.tick(collector, source, now_ns=now+1, current_native=True)
  assert not snapshots and not process.submissions
  assert actor.response_recovery_generation == 1
  actor.tick(collector, source, now_ns=now+2, current_native=False)
  assert not snapshots
  collector.original_source_capture.epoch = 2
  actor.tick(collector, source, now_ns=now+3, current_native=True)
  assert len(snapshots) == 1 and not process.submissions
  assert actor.attempted_epoch == 1  # No attempt until the owned preparation result is polled.
  actor.tick(collector, source, now_ns=now+4, current_native=True)
  assert len(snapshots) == len(process.submissions) == 1
  assert actor.attempted_epoch == 2
  assert actor.certificate is None  # Recalibration result + fresh residual are still required.
