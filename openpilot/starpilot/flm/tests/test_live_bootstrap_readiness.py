"""Reporting preserves original selection/caps; synthetic availability only."""
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from openpilot.starpilot.flm import live_tracking_bootstrap_cohorts as cohorts
from openpilot.starpilot.flm.live_tracking_bootstrap_producer import response_snapshot, _epoch_checkpoints
from openpilot.starpilot.flm.controller_replay import Snapshot
from openpilot.starpilot.flm.tests.test_live_tracking_bootstrap_cohorts import source_data


def test_missing_raw_capture_still_declines_and_now_reports_its_actual_counts():
  binding = source_data(seconds=1)[0]
  collector = SimpleNamespace(adapter=SimpleNamespace(envelope=None), original_source_capture=None, ready=())
  assert response_snapshot(collector, binding, 1) is None
  assert collector._live_bootstrap_readiness == {
    'reason': 'raw_prerequisites_unavailable', 'observedMonoTime': 1,
    'rawPoses': 0, 'readyPairs': 0, 'captureEpoch': None, 'captureStartNs': None, 'captureCutoffNs': None}


@pytest.mark.parametrize('bad_until,expected_none', [(12, True), (0, False)])
def test_diagnostics_do_not_change_original_cohort_decision(bad_until, expected_none):
  binding, q, frames, poses, events, controls, checkpoints = source_data(bad_until=bad_until, seconds=41)
  arguments = {'binding': binding, 'epoch': 1, 'epoch_start_ns': q, 'cutoff_ns': q+100_000_000_000,
    'now_ns': q+40_000_000_000, 'frames': frames, 'poses': poses, 'emissions': events,
    'controls': controls, 'checkpoints': checkpoints}
  original = cohorts.select_first_cohort(**arguments)
  diagnostics = {}
  reported = cohorts.select_first_cohort(**arguments, diagnostics=diagnostics)
  assert reported == original
  assert (reported is None) is expected_none
  assert diagnostics['requiredCohortNs'] == 38_800_000_000
  assert diagnostics['examinedAnchors'] > 0
  assert diagnostics['reason'] == ('fixed_cohort_frame_health_or_contiguity_unavailable' if expected_none else
                                    'source_cohort_selected_not_child_qualified')


def test_original_checkpoint_count_bound_still_raises_and_reports_literal_reason():
  diagnostics = {}
  checkpoints = tuple((index+1, Snapshot('reporting-only', ())) for index in range(129))
  with pytest.raises(ValueError, match='genuine bounded checkpoint metadata'):
    cohorts.source_quality_keys((), (), (), checkpoints, (), diagnostics=diagnostics)
  assert diagnostics['reason'] == 'checkpoint_count_exceeds_128_bound'


def test_original_pose_support_decline_is_reported_without_response_inspection():
  binding, q, frames, poses, events, controls, _ = source_data(bad_until=0, seconds=41)
  diagnostics = {}
  assert cohorts.quality_for_start(q, frames=frames, poses=poses[::3], emissions=events,
    controls=controls, binding=binding, now_ns=q+40_000_000_000, diagnostics=diagnostics) is None
  assert diagnostics['reason'] == 'fixed_slot_pose_count_unavailable'


def test_throttle_keeps_current_counts_and_labels_previous_selection_historical():
  binding = source_data(seconds=1)[0]
  previous = {'reason': 'fixed_cohort_clock_incomplete', 'observedMonoTime': 1,
    'rawPoses': 2, 'readyPairs': 1}
  capture = SimpleNamespace(rows=(1, 2, 3), epoch=7, epoch_start_ns=1, epoch_cutoff_ns=100)
  collector = SimpleNamespace(adapter=SimpleNamespace(envelope=object()), original_source_capture=capture,
    ready=(1, 2), _live_bootstrap_selection_epoch=7, _live_bootstrap_selection_next_ns=100,
    _live_bootstrap_last_selection=previous)
  with patch('openpilot.starpilot.flm.source_plant_runtime._current_fit_pair_reason', return_value=None):
    assert response_snapshot(collector, binding, 50) is None
  report = collector._live_bootstrap_readiness
  assert report['observedMonoTime'] == 50
  assert report['rawPoses'] == 3
  assert report['readyPairs'] == 2
  assert report['captureEpoch'] == 7
  assert report['reason'] == 'selection_throttled'
  assert report['throttled'] is True
  assert report['previousSelection'] == previous
  assert report['previousSelection'] is not previous


def test_unready_clock_avoids_archive_serialization_but_ready_and_bad_metadata_keep_full_checks():
  binding, q, frames, poses, events, controls, checkpoints = source_data(bad_until=0, seconds=41)

  def make(elapsed, checkpoint_override=None):
    now = q+int(elapsed*1e9)
    capture = SimpleNamespace(rows=tuple(row for row in poses if row.completed_ns <= now),
      epoch=1, epoch_start_ns=poses[0].mono_ns, epoch_cutoff_ns=q+100_000_000_000)
    replay = SimpleNamespace(checkpoints=checkpoint_override if checkpoint_override is not None else
      tuple(row for row in checkpoints if row[0] <= now), records=frames,
      emissions=SimpleNamespace(events={row.apply_ns: row for row in events}))
    adapter = SimpleNamespace(envelope=object(), replay=replay,
      original_context_receipts=SimpleNamespace(controls={row.control_ns: row for row in controls}))
    return SimpleNamespace(adapter=adapter, original_source_capture=capture, ready=(1,)), now

  # Unready is measured from original source clocks, never fit error/response.
  with patch('openpilot.starpilot.flm.source_plant_runtime._current_fit_pair_reason', return_value=None):
    with patch.object(cohorts, 'source_quality_keys', side_effect=AssertionError('full quality reached')) as full:
      for elapsed in (1., 20., 38.79):
        collector, now = make(elapsed)
        assert response_snapshot(collector, binding, now) is None
        assert collector._live_bootstrap_readiness['reason'] == 'fixed_cohort_clock_incomplete'
      assert full.call_count == 0
      collector, now = make(39.)
      with pytest.raises(AssertionError, match='full quality reached'):
        response_snapshot(collector, binding, now)
      assert full.call_count == 1
    # A bounded-clock preflight must not hide the original bad metadata guard.
    collector, now = make(1., ((q, Snapshot('bad-bound', ())),)+tuple((poses[0].mono_ns+i*1_000_000, Snapshot('bad-bound', ())) for i in range(1, 129)))
    with pytest.raises(ValueError, match='genuine bounded checkpoint metadata'):
      response_snapshot(collector, binding, now)


def test_old_history_bound_failure_becomes_exact_original_epoch_selection():
  binding, q, frames, poses, events, controls, epoch = source_data(seconds=61)
  cutoff = q+100_000_000_000
  # Old recorder history is structurally genuine metadata but irrelevant to this epoch.
  old = tuple((stamp, Snapshot('synthetic-availability-only', ())) for stamp in range(1, 130))
  assert old[-1][0] < q
  retained = old+epoch
  with pytest.raises(ValueError, match='genuine bounded checkpoint metadata'):
    cohorts.source_quality_keys(frames, poses, events, retained, controls)
  scoped = _epoch_checkpoints(retained, poses, cutoff)
  assert scoped == epoch
  assert cohorts.source_quality_keys(frames, poses, events, scoped, controls) == (
    cohorts.source_quality_keys(frames, poses, events, epoch, controls))
  arguments = {'binding': binding, 'epoch': 1, 'epoch_start_ns': q, 'cutoff_ns': cutoff,
    'now_ns': q+60_000_000_000, 'frames': frames, 'poses': poses, 'emissions': events, 'controls': controls}
  expected = cohorts.select_first_cohort(checkpoints=epoch, **arguments)
  actual = cohorts.select_first_cohort(checkpoints=scoped, **arguments)
  assert actual == expected
  assert actual is not None
  assert actual['start_ns'] == q+12_000_000_000
  assert actual['origin_checkpoint_ns'] == q
  assert actual['pose_counts'] == (80,)*8
  # Every candidate anchor is retained, including later original valid alternatives.
  assert tuple(stamp for stamp, _ in scoped) == tuple(stamp for stamp, _ in epoch)
  changed_response = tuple(replace(row, yaw_rate=999., roll_rad=-1.) for row in poses)
  assert _epoch_checkpoints(retained, changed_response, cutoff) == scoped


def test_129_checkpoints_inside_epoch_still_fail_original_cap():
  _, q, frames, poses, events, controls, _ = source_data(seconds=1)
  snapshot = Snapshot('synthetic-availability-only', ())
  checkpoints = ((q, snapshot),)+tuple((poses[0].mono_ns+i*1_000_000, snapshot) for i in range(1, 129))
  scoped = _epoch_checkpoints(checkpoints, poses, q+100_000_000_000)
  assert scoped == checkpoints
  assert len(scoped) == 129
  with pytest.raises(ValueError, match='genuine bounded checkpoint metadata'):
    cohorts.source_quality_keys(frames, poses, events, scoped, controls)


def test_scope_retains_preceding_anchor_and_discards_only_outside_epoch():
  _, q, _, poses, _, _, checkpoints = source_data(seconds=1)
  cutoff = q+100_000_000_000
  retained = checkpoints+((cutoff, checkpoints[0][1]), (cutoff+1, checkpoints[0][1]))
  assert _epoch_checkpoints(retained, poses, cutoff) == checkpoints
  assert _epoch_checkpoints(((1, checkpoints[0][1]),)+retained, poses, cutoff) == checkpoints


@pytest.mark.parametrize('bad', ['order', 'type'])
def test_bad_metadata_is_rejected_even_outside_epoch(bad):
  _, q, _, poses, _, _, checkpoints = source_data(seconds=1)
  outside = ((2, checkpoints[0][1]), (1, checkpoints[0][1])) if bad == 'order' else ((1, object()),)
  with pytest.raises(ValueError):
    _epoch_checkpoints(outside+checkpoints, poses, q+100_000_000_000)


def test_missing_preceding_anchor_cannot_select_any_cohort():
  binding, q, frames, poses, events, controls, checkpoints = source_data(seconds=41)
  scoped = _epoch_checkpoints(checkpoints[1:], poses, q+100_000_000_000)
  assert scoped == ()
  assert cohorts.select_first_cohort(checkpoints=scoped, frames=frames, poses=poses, emissions=events, controls=controls,
    binding=binding, epoch=1, epoch_start_ns=q, cutoff_ns=q+100_000_000_000, now_ns=q+40_000_000_000) is None
