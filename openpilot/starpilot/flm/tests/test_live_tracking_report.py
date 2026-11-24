"""Actual report writer/reader contract; all source points below are synthetic."""
import json
from types import SimpleNamespace

import pytest

from openpilot.starpilot.flm.training_report import TrainingReporter, read_report
from openpilot.starpilot.flm.tests.test_live_tracking_rollout import _optimizer, _refresh, NS


def test_actual_report_writer_reader_preserve_tracking_values_and_original_points(tmp_path):
  now = 100*NS
  owner = _optimizer()
  _refresh(owner, now)
  points = [{'controlMonoTime': now-NS+index*50_000_000,
    'applyMonoTime': now-NS+index*50_000_000,
    'poseMonoTime': now-NS+index*50_000_000+170_000_000,
    'desiredForceAccel': .2, 'measuredForceAccel': .1, 'errorForceAccel': .2-.1} for index in range(4)]
  plot = {'version': 'live-tracking-paired-plot-v1', 'episodeId': '1'*64,
    'profileSha256': owner._hash(owner.accepted), 'lagNs': 170_000_000, 'units': 'm/s²', 'sampled': False,
    'totalEndpoints': 4, 'points': points, 'bindingCurrent': True,
    'ageSeconds': (now-points[-1]['poseMonoTime'])/NS}
  live = SimpleNamespace(latest_offer=None, report=lambda stamp: {'tracking': owner.report(stamp),
    'trackingValues': owner.tracking_values(), 'trackingPlot': plot})
  collector = SimpleNamespace(adapter=SimpleNamespace(raw_collection=False, event_collection=False, objective=None),
    blocks=(), points=(), session='synthetic-test-only', sealed=False, reason='collecting',
    training_interruptions={}, dropped=0, live_tracking=live)
  source = SimpleNamespace(enabled=True, calibration_generation='synthetic', observed_ns=now)
  message = SimpleNamespace(autoSteeringLearning=SimpleNamespace(learning=SimpleNamespace(status='collecting', reason='synthetic')))
  path = tmp_path/'producer.json'
  TrainingReporter('synthetic test source', path=path).publish(now_ns=now, message=message,
    collector=collector, source=source, objective='synthetic', pending=False, native_current=False)
  actual = read_report(path, now_ns=now)
  assert actual['current'] is not None
  current = actual['current']
  assert current['tracking'] == owner.report(now)
  assert current['trackingValues'] == owner.tracking_values()
  assert current['trackingPlot'] == plot
  assert current['application']['applied'] is False
  assert len(path.read_bytes()) <= 32768
  # Root may retain this exact read_report result for the canonical JS consumer.
  (tmp_path/'producer-read-report.json').write_text(json.dumps(actual, allow_nan=False))
  stale = read_report(path, now_ns=now+3*NS)['current']
  assert stale['tracking']['bindingCurrent'] is False
  assert stale['tracking']['applicationAcknowledged'] is False
  assert stale['trackingPlot']['bindingCurrent'] is False
  assert stale['trackingValues']['accepted'] == current['trackingValues']['accepted']


@pytest.mark.parametrize('field,value', [('poseMonoTime', 0), ('applyMonoTime', 10**20)])
def test_tracking_plot_timestamp_malformed_data_is_not_renderable(field, value):
  from openpilot.starpilot.flm.live_tracking_report import tracking_objects
  point = {'controlMonoTime': 10, 'applyMonoTime': 11, 'poseMonoTime': 12,
    'desiredForceAccel': .2, 'measuredForceAccel': .1, 'errorForceAccel': .2-.1}
  point[field] = value
  plot = {'version': 'live-tracking-paired-plot-v1', 'episodeId': '1'*64, 'profileSha256': '2'*64,
    'lagNs': 0, 'units': 'm/s²', 'sampled': False, 'totalEndpoints': 4, 'points': [point]*4,
    'bindingCurrent': True, 'ageSeconds': 0.}
  with pytest.raises(ValueError, match='timestamps'):
    tracking_objects({'trackingPlot': plot}, fresh=True)


@pytest.mark.parametrize('extra_ns,accepted', [(0, True), (30_000_000, True), (30_000_001, False), (-1, False)])
def test_report_plot_preserves_exact_lag_bracket_boundary(extra_ns, accepted):
  from openpilot.starpilot.flm.live_tracking_report import tracking_objects
  points = [{'controlMonoTime': 1_000_000_000+index*50_000_000,
    'applyMonoTime': 1_000_000_000+index*50_000_000,
    'poseMonoTime': 1_170_000_000+index*50_000_000+extra_ns,
    'desiredForceAccel': .2, 'measuredForceAccel': .1, 'errorForceAccel': .2-.1} for index in range(4)]
  plot = {'version': 'live-tracking-paired-plot-v1', 'episodeId': '1'*64, 'profileSha256': '2'*64,
    'lagNs': 170_000_000, 'units': 'm/s²', 'sampled': False, 'totalEndpoints': 4, 'points': points,
    'bindingCurrent': True, 'ageSeconds': 0.}
  if accepted:
    assert tracking_objects({'trackingPlot': plot}, fresh=True)['trackingPlot'] == plot
  else:
    with pytest.raises(ValueError, match='timestamps'):
      tracking_objects({'trackingPlot': plot}, fresh=True)
