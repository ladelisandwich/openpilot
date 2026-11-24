"""Actual optional publisher cache reports child rejection without admission."""
from openpilot.starpilot.flm.tests._drain_fixture import separate_drain_fixture
from types import SimpleNamespace
from unittest import mock

import pytest

from openpilot.starpilot.flm import steerlearnerd
from openpilot.starpilot.flm.fullrate_collector import FullrateCollector
from openpilot.starpilot.flm.learner_factory import decode_fit_result
from openpilot.starpilot.flm.learning_wire import read_profile
from openpilot.starpilot.flm.runtime_adapter_registry import SourceState

NOW = 1_000_000_000
KEY = ('event', 'center')


@pytest.mark.parametrize('status,reason', (
  ('rejected', 'actual_controller_replay'),
  ('collecting', 'independent_physical_model'),
  ('collecting', 'disjoint_stage_maneuvers'),
))
@pytest.mark.parametrize('blocked', (None, 'native', 'physical_context', 'settings', 'prior_target'))
def test_actual_event_cached_child_reports_reason_only_for_exact_current_prior(status, reason, blocked):
  adapter = mock.Mock(event_collection=True, raw_storage=True,
    source_plant_protocol=None, original_feedback_archive=None,
    envelope=SimpleNamespace(prior_full_profile_sha256='e' * 64))
  adapter.prepare_control.side_effect = lambda evidence: evidence
  adapter.install_fit.return_value = False
  collector = FullrateCollector(adapter)
  collector.physical_context = ('current-physical-context', 'model-generation', 'lag-generation', 100_000_000)
  collector.ready.append((object(), object()))
  source = SourceState(True, False, '0', 'a' * 64, NOW, NOW + 500_000_000, 'b' * 64, (0., 0., 0.))
  evidence = SimpleNamespace(source_epoch='a' * 64, source_receipt_ns=NOW, control_ns=NOW,
                             identity='c' * 64, generation=1, domain='normalized_torque', stage_revision='stage',
                             response_revision='response', controller_mode='standard', policy='generic',
                             baseline_revision='baseline', controller_revision='controller', turn_assist=False)
  report = {'status': status, 'reason': reason, 'profile': None, 'validation': None}
  tick = [-1]
  sm = mock.Mock(updated={'deviceMotion': False}, logMonoTime={'deviceMotion': 0})

  def update(timeout=None):
    if timeout == 0:
      return
    tick[0] += 1
    if tick[0] == 3:
      raise StopIteration
    if tick[0] == 2 and blocked == 'prior_target':
      adapter.envelope.prior_full_profile_sha256 = 'f' * 64
    if tick[0] == 2 and blocked == 'physical_context':
      collector.physical_context = ('replaced-physical-context', 'model-generation', 'lag-generation', 100_000_000)

  sm.update.side_effect = update
  fitter = mock.Mock()
  fitter.child = None

  def submit(*_args, **_kwargs):
    fitter.child = object()
    return True

  def poll(*_args, **_kwargs):
    if tick[0] == 1:
      fitter.child = None
      return report
    return None

  fitter.submit.side_effect = submit
  fitter.poll.side_effect = poll
  pm = mock.Mock()
  control_socket, applied_socket = object(), object()

  def drain(socket, callback, _receive, _sink, now):
    if socket is control_socket:
      event = SimpleNamespace(which=lambda: 'starpilotLateralState',
                              starpilotLateralState=SimpleNamespace(learningEvidence=SimpleNamespace(version=1)))
      callback(event, now, now)

  def read_source(_now):
    if tick[0] == 2 and blocked == 'settings':
      return SourceState(False, False, '0', source.source_epoch, source.observed_ns, source.expires_ns,
                         source.cp_sha256, source.calibration, source.calibration_generation)
    return source

  with mock.patch('openpilot.starpilot.flm.event_collector.EventFullrateCollector', return_value=collector), \
       mock.patch.object(steerlearnerd, 'drain_pair', separate_drain_fixture(drain)), \
       mock.patch.object(steerlearnerd, 'read_control', return_value=evidence), \
       mock.patch.object(collector, 'control'), mock.patch.object(collector, 'physical'), \
       mock.patch.object(collector, 'job_snapshot', side_effect=[{'job': 'bounded'}, None, None]), \
       mock.patch('openpilot.starpilot.flm.native_health.current_native_health',
                  side_effect=lambda *_args: not (tick[0] == 2 and blocked == 'native')):
    with pytest.raises(StopIteration):
      steerlearnerd.run(SimpleNamespace(), sm, control_socket, applied_socket, pm,
                        mock.Mock(return_value=adapter), mock.Mock(return_value=object()), read_source,
                        lambda _evidence: (KEY,), lambda _evidence: KEY, decode_fit_result,
                        mock.Mock(), fitter, mock.Mock(), clock=lambda: NOW + tick[0] * 250_000_000)

  assert pm.send.call_count == 3
  child_publication = pm.send.call_args_list[1].args[1]
  assert str(child_publication.autoSteeringLearning.learning.status) == status
  assert str(child_publication.autoSteeringLearning.learning.reason) == reason
  final = pm.send.call_args.args[1]
  if blocked is None:
    assert str(final.autoSteeringLearning.learning.status) == status
    assert str(final.autoSteeringLearning.learning.reason) == reason
  elif blocked == 'settings':
    assert str(final.autoSteeringLearning.learning.status) == 'unavailable'
    assert str(final.autoSteeringLearning.learning.reason) == 'learning_disabled'
  else:
    assert str(final.autoSteeringLearning.learning.status) == 'collecting'
    assert str(final.autoSteeringLearning.learning.reason) == 'independent_evidence'
  for call in pm.send.call_args_list:
    assert read_profile(call.args[1]) is None
    assert call.args[1].autoSteeringLearning.learning.runtimeProfile.version == 0
  assert collector.result is collector.fit_result is collector.fit_validation is collector.last_validation is None
  fitter.close.assert_called_once_with('learner_parent_exit')
