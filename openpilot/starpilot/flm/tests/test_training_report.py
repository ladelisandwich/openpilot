"""Actual optional writer/collector and typed application, never control authority."""
from openpilot.starpilot.flm.tests._drain_fixture import separate_drain_fixture
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace
from unittest import mock

import pytest

from openpilot.cereal import messaging
from openpilot.starpilot.flm.adaptive_profile import ApplicationOutcome, Parameter, Profile, Result
from openpilot.starpilot.flm.application_wire import write_application
from openpilot.starpilot.flm.fullrate_collector import AppliedEvidence, ControlEvidence, FullrateCollector, RawPoint
from openpilot.starpilot.flm.training_report import TrainingReporter, read_report, READ_MAX_BYTES

NOW = 40_000_000_000
IDENTITY = 'a' * 64


def evidence(now=NOW):
  return ControlEvidence(
    identity=IDENTITY, generation=0, control_ns=now, state_ns=now - 1,
    domain='torque', stage_revision='stage', response_revision='response', requested=.2,
    features=(1.,), baseline_prediction=.18, enabled=True, manual=False, active=True, valid=True,
    override=False, fault=False, saturated=False, limited=False, model_wobble=False, controller_chatter=False,
    applied_coordinates=(.02, 0.), applied_profile_sequence=42,
    source_epoch="epoch", source_receipt_ns=now - 1_000_000, baseline_revision="baseline",
    applied_profile_observed_ns=now - 1_000_000, applied_profile_expires_ns=now + 400_000_000,
  )


def fixture(tmp_path):
  adapter = SimpleNamespace(raw_collection=True, objective=SimpleNamespace(required_cohorts=('high.left.steady',)))
  collector = FullrateCollector(adapter)
  control = evidence()
  applied = AppliedEvidence(NOW, NOW - 1, NOW + 1, 'torque', .2, True, True, False, False, False, True)
  collector.context = (
    control.identity, control.generation, control.domain, control.stage_revision, control.response_revision,
    control.source_epoch, control.controller_mode, control.policy, control.baseline_revision, control.controller_revision,
    control.turn_assist,
  )
  collector.cohort = 'high.left.steady'
  physical_context = ('physical', 'model', 'lag', 0)
  collector.physical_context = physical_context
  # Actual collector seal constructs a distinct RawBlock and consumes its buffer.
  for tick in range(40):
    stamp = NOW - 400_000_000 + tick * 10_000_000
    collector.raw_points.append(RawPoint(stamp, stamp + 1, stamp + 2, control, applied, {}, physical_context))
  collector._seal_raw()
  assert len(collector.blocks) == 1 and not collector.raw_points
  for tick in range(12):
    stamp = NOW - 120_000_000 + tick * 10_000_000
    collector.raw_points.append(RawPoint(stamp, stamp + 1, stamp + 2, control, applied, {}, physical_context))
  collector.ready.append((control, applied))
  collector.reason = 'collecting_maneuver'
  collector.split('test_interruption')
  # Restore the recorded ready sample after the genuine interruption.
  collector.ready.append((control, applied))
  for tick in range(12):
    stamp = NOW - 120_000_000 + tick * 10_000_000
    collector.raw_points.append(RawPoint(stamp, stamp + 1, stamp + 2, control, applied, {}, physical_context))
  path = tmp_path / 'training.json'
  reporter = TrainingReporter('TEST_TORQUE_CAR', path=path)
  source = SimpleNamespace(enabled=True, manual=False, source_epoch='epoch', expires_ns=NOW + 400_000_000,
                           calibration_generation='calibration', observed_ns=NOW - 100_000_000)
  return collector, control, reporter, source, path


def message(status='collecting', reason='independent_evidence'):
  event = messaging.new_message('autoSteeringLearning', valid=True, logMonoTime=NOW)
  event.autoSteeringLearning.learning.status = status
  event.autoSteeringLearning.learning.reason = reason
  return event


def application_event(control, *, observed=NOW, profile_sequence=42, coordinates=(.02, 0.), identity=IDENTITY):
  event = messaging.new_message('starpilotLateralState', valid=True, logMonoTime=NOW)
  write_application(event.starpilotLateralState, ApplicationOutcome(identity, profile_sequence, observed, 'applied',
                    coordinates, 'qualified_profile', 'baseline', 'physical'))
  return event.as_reader(), control


def publish(reporter, collector, source, *, now=NOW, event=None, native=True, fit=None):
  reporter.publish(now_ns=now, message=message() if event is None else event, collector=collector,
                   source=source, objective=('feedforward', 'high', 'steady'), pending=False,
                   native_current=native, fit_result=fit)


def test_actual_collector_progress_typed_application_and_bounded_writer_reader(tmp_path):
  collector, control, reporter, source, path = fixture(tmp_path)
  reporter.control = application_event(control)
  before = deepcopy(vars(collector))
  publish(reporter, collector, source)
  assert vars(collector) == before
  assert path.stat().st_size <= READ_MAX_BYTES
  assert not path.stat().st_mode & 0o077
  current = read_report(path, now_ns=NOW)['current']
  assert current['progress'] == {'samples': 52, 'blocks': 1, 'requiredBlocks': 16}
  assert current['application'] == {'applied': True, 'reason': 'qualified_profile'}
  assert current['details']['Applied profile sequence'] == 42
  assert current['details']['Applied correction coordinates'] == [.02, 0.]
  assert current['details']['Capture interruptions'] == {'test_interruption': 1}


@pytest.mark.parametrize('case', ('missing', 'stale_control', 'stale_application', 'native', 'identity', 'sequence', 'coordinates'))
def test_applied_requires_current_matching_controller_evidence(tmp_path, case):
  collector, control, reporter, source, path = fixture(tmp_path)
  if case == 'stale_control':
    control = replace(control, control_ns=NOW - 100_000_001)
  reporter.control = application_event(control, observed=NOW - 500_000_001 if case == 'stale_application' else NOW,
                                      profile_sequence=43 if case == 'sequence' else 42,
                                      coordinates=(.03, 0.) if case == 'coordinates' else (.02, 0.))
  if case == 'missing':
    reporter.control = None
  if case == 'identity':
    collector.ready[-1] = (replace(control, identity='b' * 64), collector.ready[-1][1])
  publish(reporter, collector, source, native=case != 'native')
  current = read_report(path, now_ns=NOW)['current']
  assert not current['application']['applied']
  assert current['details']['Applied profile sequence'] == 0
  assert current['details']['Applied correction coordinates'] == []


def test_qualified_fit_alone_never_claims_applied_and_stale_report_withdraws_current_claim(tmp_path):
  collector, control, reporter, source, path = fixture(tmp_path)
  profile = Profile(IDENTITY, 42, NOW, NOW + 500_000_000,
                    (Parameter('ff', 'stage', -.03, .03, .03, .005),), (.02,),
                    ('left_train', 'right_train'), ('left_held', 'right_held'), 1., .9, (('left', .01), ('right', .01)))
  result = Result('qualified', 'qualified_actual_replay', profile)
  reporter.fitted(result, ('feedforward', 'high', 'steady'))
  reporter.fit_attempts = 1
  publish(reporter, collector, source, fit=result)
  current = read_report(path, now_ns=NOW)['current']
  assert current['fit'] == {'status': 'qualified', 'reason': 'qualified_actual_replay', 'trainWindows': 2, 'heldoutWindows': 2}
  assert not current['application']['applied']
  assert collector.fit_result is collector.result is collector.last_validation is None
  reporter.control = application_event(evidence(NOW + 1_000_000_000), observed=NOW + 1_000_000_000)
  collector.ready[-1] = (reporter.control[1], collector.ready[-1][1])
  source.observed_ns, source.expires_ns = NOW + 1_000_000_000, NOW + 1_400_000_000
  publish(reporter, collector, source, now=NOW + 1_000_000_000, fit=result)
  assert read_report(path, now_ns=NOW + 1_000_000_000)['current']['application']['applied']
  stale = read_report(path, now_ns=NOW + 4_000_000_000)
  assert not stale['current']['fresh'] and not stale['current']['application']['applied']
  assert stale['history'][-1]['applied']


def test_writer_rate_and_history_are_bounded(tmp_path):
  collector, _, reporter, source, path = fixture(tmp_path)
  publish(reporter, collector, source, event=message('rejected', 'rejection_0'))
  before = path.read_bytes(), path.stat().st_mtime_ns, tuple(reporter.history)
  publish(reporter, collector, source, now=NOW + 999_999_999, event=message('rejected', 'must_not_write'))
  assert before == (path.read_bytes(), path.stat().st_mtime_ns, tuple(reporter.history))
  for index in range(1, 25):
    publish(reporter, collector, source, now=NOW + index * 1_000_000_000, event=message('rejected', f'rejection_{index}'))
  actual = read_report(path, now_ns=NOW + 24_000_000_000)
  assert len(actual['history']) == 20
  assert actual['history'][0]['reason'] == 'rejection_5'
  assert actual['history'][-1]['reason'] == 'rejection_24'
  assert path.stat().st_size <= READ_MAX_BYTES


@pytest.mark.parametrize('operation', ('create', 'replace', 'cleanup'))
def test_reporting_io_failure_never_mutates_or_stops_collection(tmp_path, operation):
  collector, _, reporter, source, _ = fixture(tmp_path)
  before = deepcopy(vars(collector))
  target = {'create': 'tempfile.mkstemp', 'replace': 'os.replace', 'cleanup': 'Path.unlink'}[operation]
  prefix = 'openpilot.starpilot.flm.training_report.'
  with mock.patch(prefix + target, side_effect=OSError('report unavailable')):
    publish(reporter, collector, source)
  assert vars(collector) == before


@pytest.mark.parametrize('field,value', (
  ('applied_profile_expires_ns', NOW - 1), ('applied_profile_observed_ns', NOW + 1),
  ('source_receipt_ns', NOW - 500_000_001), ('source_epoch', 'replaced'),
  ('baseline_revision', 'replaced'), ('enabled', False), ('active', False), ('valid', False),
  ('manual', True), ('fault', True),
))
def test_original_controller_profile_and_source_guards_are_not_restamped(tmp_path, field, value):
  collector, control, reporter, source, path = fixture(tmp_path)
  reporter.control = application_event(replace(control, **{field: value}))
  publish(reporter, collector, source)
  assert not read_report(path, now_ns=NOW)['current']['application']['applied']


def test_payload_error_isolated_from_learning(tmp_path):
  collector, _, reporter, source, _ = fixture(tmp_path)
  before = deepcopy(vars(collector))
  with mock.patch('openpilot.starpilot.flm.training_report.read_application', side_effect=ValueError('bad report')):
    reporter.control = application_event(evidence())
    publish(reporter, collector, source)
  assert vars(collector) == before


def test_actual_parent_publication_also_writes_training_snapshot(tmp_path):
  from openpilot.starpilot.flm import steerlearnerd
  from openpilot.starpilot.flm.runtime_adapter_registry import SourceState
  from openpilot.starpilot.flm.tests.test_fullrate_pipeline import SyntheticAdapter, snapshots, serialized_control, serialized_output

  class Adapter(SyntheticAdapter):
    def select_objective(self, key):
      pass

    def prepare_control(self, row):
      return row

  control, applied, physical = snapshots(NOW)
  source = SourceState(True, False, '0', control.source_epoch, NOW, NOW + 500_000_000,
                       'b' * 64, (0., 0., 0.))
  sm = mock.Mock(updated={'deviceMotion': False}, logMonoTime={'deviceMotion': 0})
  polls = iter((None,))
  sm.update.side_effect = lambda timeout=None: None if timeout == 0 else next(polls)
  fitter = mock.Mock(child=None)
  fitter.poll.return_value = None
  publisher = mock.Mock()
  control_socket, output_socket = object(), object()
  reporter = TrainingReporter('ACTUAL_PARENT', path=tmp_path / 'parent.json')

  def drain(socket, callback, _receive, sink, now):
    callback(serialized_control(control) if socket is control_socket else serialized_output(applied), sink, now)

  with mock.patch.object(steerlearnerd, 'drain_pair', separate_drain_fixture(drain)), \
       mock.patch('openpilot.starpilot.flm.native_health.current_native_health', return_value=True):
    with pytest.raises(StopIteration):
      steerlearnerd.run(SimpleNamespace(), sm, control_socket, output_socket, publisher,
        mock.Mock(return_value=Adapter()), mock.Mock(return_value=physical), lambda now: source,
        lambda row: ('ff',), lambda row: 'ff', mock.Mock(), mock.Mock(), fitter, mock.Mock(),
        clock=lambda: NOW + 1_000_000, reporter=reporter)
  assert publisher.send.call_count == 1
  actual = read_report(reporter.path, now_ns=NOW + 1_000_000)
  assert actual['current']['fresh'] and actual['current']['enabled']
  assert actual['current']['progress'] == {'samples': 0, 'blocks': 0, 'requiredBlocks': 8}
  assert not actual['current']['application']['applied']
  assert actual['current']['details']['Vehicle'] == 'ACTUAL_PARENT'
  assert reporter.control is not None
  assert reporter.control[1] == control
  fitter.close.assert_called_once_with('learner_parent_exit')
