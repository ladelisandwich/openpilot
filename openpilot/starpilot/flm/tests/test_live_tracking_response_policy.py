"""Purpose/admission boundaries, not a fabricated physical calibration result."""
import ast
from pathlib import Path

import pytest

from openpilot.starpilot.flm.live_tracking_response_policy import (
  MAX_AS, MAX_CPU, MAX_WALL_NS, PURPOSE, WORKER_MODULE, production_result, validate_production_result,
)


def measured_shape():
  from openpilot.starpilot.flm.live_tracking_response import certificate_to_dict
  from openpilot.starpilot.flm.tests.test_live_tracking import fixture_certificate
  return {'live_job_version': PURPOSE, 'status': 'response_identified',
    'reason': 'independent_train_heldout_response_passed', 'certificate': certificate_to_dict(fixture_certificate()),
    'profile': None, 'current_permission': False}


def test_exact_production_worker_only_emits_numerical_admission():
  raw = measured_shape()
  report = production_result(raw)
  assert validate_production_result(report) == report
  assert report['worker_purpose'] == PURPOSE and report['production_admissible'] is True
  assert report['current_permission'] is False and report['profile'] is None
  assert 'worker_purpose' not in raw and 'production_admissible' not in raw


@pytest.mark.parametrize('change', [
  {'diagnostic_purpose': 'live-response-bootstrap-diagnostic-v1'},
  {'worker_purpose': PURPOSE}, {'production_admissible': True},
  {'current_permission': True}, {'profile': {}}, {'certificate': {}},
  {'live_job_version': 'live-response-bootstrap-diagnostic-v1'},
])
def test_math_result_cannot_supply_admission_or_permission(change):
  with pytest.raises((KeyError, TypeError, ValueError)):
    production_result(dict(measured_shape(), **change))


def test_collecting_never_becomes_admissible():
  report = production_result({'live_job_version': PURPOSE, 'status': 'collecting',
    'reason': 'insufficient_original_support', 'certificate': None, 'profile': None, 'current_permission': False})
  assert report['production_admissible'] is False
  with pytest.raises(ValueError, match='production response'):
    validate_production_result(report)


@pytest.mark.parametrize('change', [
  {'worker_purpose': 'live-response-bootstrap-diagnostic-v1'}, {'production_admissible': False},
  {'diagnostic_purpose': 'live-response-bootstrap-diagnostic-v1'}, {'current_permission': True}, {'profile': {}},
])
def test_process_report_rejects_diagnostics_and_permission(change):
  report = production_result(measured_shape())
  with pytest.raises((KeyError, TypeError, ValueError)):
    validate_production_result(dict(report, **change))


def test_legacy_and_diagnostic_limits_are_unchanged():
  from openpilot.starpilot.flm.fit_process import FitProcess
  from openpilot.starpilot.flm.fit_worker import MAX_CPU as LEGACY_CPU
  from openpilot.starpilot.flm.live_tracking_bootstrap_process import LiveBootstrapProcess
  from openpilot.starpilot.flm.live_tracking_bootstrap_worker import PURPOSE as DIAGNOSTIC_PURPOSE
  from openpilot.starpilot.flm.live_tracking_response_process import ProductionBootstrapProcess
  assert LEGACY_CPU == 20 and FitProcess.wall_ns == 30_000_000_000
  assert FitProcess.interval_ns == 60_000_000_000
  assert LiveBootstrapProcess.wall_ns == 300_000_000_000
  assert DIAGNOSTIC_PURPOSE == 'live-response-bootstrap-diagnostic-v1'
  assert MAX_CPU == 180 and MAX_WALL_NS == 240_000_000_000 and MAX_AS == 2*1024**3
  assert ProductionBootstrapProcess.worker_module == WORKER_MODULE
  assert ProductionBootstrapProcess.wall_ns == MAX_WALL_NS
  assert ProductionBootstrapProcess.cancel is FitProcess.cancel
  assert ProductionBootstrapProcess.close is FitProcess.close


def test_production_selection_is_constructor_not_environment(monkeypatch):
  from openpilot.starpilot.flm.fit_process import FitProcess
  from openpilot.starpilot.flm.live_tracking_response_process import ProductionBootstrapProcess
  observed = []
  def submit(process, snapshot, *, token, now_ns):
    observed.append((process.worker_module, snapshot, token, now_ns))
    return True
  monkeypatch.setattr(FitProcess, 'submit', submit)
  for purpose in (None, 'anything', 'live-response-bootstrap-diagnostic-v1'):
    if purpose is None:
      monkeypatch.delenv('STARPILOT_LIVE_BOOTSTRAP_PURPOSE', raising=False)
    else:
      monkeypatch.setenv('STARPILOT_LIVE_BOOTSTRAP_PURPOSE', purpose)
    assert ProductionBootstrapProcess().submit({'source': 'test-only'}, token=('test',), now_ns=1)
  assert observed == [(WORKER_MODULE, {'source': 'test-only'}, ('test',), 1)]*3


def test_parent_discards_unqualified_successful_child_report(monkeypatch):
  from openpilot.starpilot.flm.fit_process import FitProcess
  from openpilot.starpilot.flm.live_tracking_response_process import ProductionBootstrapProcess
  report = dict(measured_shape(), production_admissible=True,
    diagnostic_purpose='live-response-bootstrap-diagnostic-v1')
  monkeypatch.setattr(FitProcess, 'poll', lambda self, **kwargs: report)
  process = ProductionBootstrapProcess()
  assert process.poll(token=('test',), now_ns=1) is None
  assert process.reason == 'production_response_report_unqualified'


def test_daemon_explicitly_constructs_production_response_process():
  # Current live source tracking owns no numerical plant-bootstrap process.
  import openpilot.starpilot.flm.steerlearnerd as daemon
  tree = ast.parse(Path(daemon.__file__).read_text())
  calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call) and
    isinstance(node.func, ast.Name) and node.func.id == 'LiveTrackingActor']
  assert len(calls) == 1
  keywords = {item.arg: item.value for item in calls[0].keywords}
  assert not calls[0].args
  assert set(keywords) == {'probe_process', 'source_tracking'}
  assert isinstance(keywords['source_tracking'], ast.Constant)
  assert keywords['source_tracking'].value is True
  from openpilot.starpilot.flm.live_tracking_actor import LiveTrackingActor
  with pytest.raises(ValueError, match='source tracking has no plant bootstrap process'):
    LiveTrackingActor(bootstrap_process=object(), source_tracking=True)
  assert isinstance(keywords['probe_process'], ast.Call)
  assert isinstance(keywords['probe_process'].func, ast.Name)
  assert keywords['probe_process'].func.id == 'ProbeProcess'


def test_real_pending_child_survives_capture_rollover_but_not_changed_physical_basis():
  # Real optional-child IPC/lifecycle and actual canonical binding builder; the
  # numerical evidence fixture remains explicitly synthetic, not road proof.
  import os
  import subprocess
  import sys
  from dataclasses import replace
  from types import SimpleNamespace
  from openpilot.starpilot.flm.live_tracking_actor import LiveTrackingActor
  from openpilot.starpilot.flm.live_tracking_response_process import ProductionBootstrapProcess
  from openpilot.starpilot.flm.live_tracking_requalification import RetainedResponseRequalifier
  from openpilot.starpilot.flm.tests.test_live_tracking_requalification import certificate, basis, new_drive, synthetic_pair
  old = certificate()
  owner = RetainedResponseRequalifier(old, basis(old), new_drive(old))
  control, output, _pose = synthetic_pair(owner, 0)
  binding = owner._binding
  from openpilot.starpilot.flm.runtime_adapter_registry import SourceState
  from openpilot.starpilot.flm.torque_emission_replay import EmissionHistory
  def source_at(now):
    return SourceState(True, False, binding.reset_generation, binding.source_epoch, now, now+500_000_000,
      binding.cp_sha256, (), binding.calibration_generation, binding.persistent_source_epoch)
  source = source_at(99_000_000_000)
  # Empty original inventory is truthful for this child-lifecycle-only fixture.
  replay = SimpleNamespace(raw_mode=False, emissions=EmissionHistory(None))
  collector = SimpleNamespace(ready=[(control, output)], adapter=SimpleNamespace(replay=replay),
    original_source_capture=SimpleNamespace(epoch=0), live_tracking=None)
  process = ProductionBootstrapProcess()
  actor = LiveTrackingActor(bootstrap_process=process)
  seed = actor.binding_for(collector, source)
  token = tuple(vars(seed).values())
  actor.seed, actor.token, actor.attempted_epoch = seed, token, 0
  child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'],
    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, start_new_session=True)
  process.child, process.token, process.started_ns = child, token, 39_000_000_000
  assert child.stdin is not None and child.stdout is not None
  os.set_blocking(child.stdin.fileno(), False)
  os.set_blocking(child.stdout.fileno(), False)
  try:
    assert actor.tick(collector, source, now_ns=99_000_000_000, current_native=True) is None
    assert process.child is child and child.poll() is None and not process.reaping
    collector.original_source_capture.epoch = 1
    assert actor.tick(collector, source_at(101_000_000_000), now_ns=101_000_000_000, current_native=True) is None
    assert actor.seed == seed and process.token == token
    assert process.child is child and child.poll() is None and not process.reaping
    assert actor.attempted_epoch == 0  # No duplicate child at capture rotation.
    collector.ready = [(replace(control, vehicle_model_anchor=(16., 100000., 110000.)), output)]
    assert actor.tick(collector, source_at(102_000_000_000), now_ns=102_000_000_000, current_native=True) is None
    assert process.reaping or process.child is None
    assert actor.reason == 'actual_response_source_unavailable'
  finally:
    assert process.close('test_actual_child_cleanup')
  assert child.returncode is not None and process.child is None
