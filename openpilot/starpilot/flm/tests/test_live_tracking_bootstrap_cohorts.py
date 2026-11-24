"""Outcome-blind availability tests, not original/native calibration proof."""
from typing import Any


from dataclasses import replace
from types import SimpleNamespace
import math

from openpilot.starpilot.flm import live_tracking_bootstrap_cohorts as cohorts
from openpilot.starpilot.flm.controller_replay import ReplayFrame, Snapshot
from openpilot.starpilot.flm.torque_emission_replay import Emission
from openpilot.starpilot.flm.tests.test_live_tracking_requalification import certificate, synthetic_pair


def source_data(*, bad_until=12, seconds=61):
  cert = certificate()
  owner = SimpleNamespace(_binding=cert.binding, original=cert)
  first, _, _ = synthetic_pair(owner, 0)
  q = first.control_ns
  controls, frames, events, poses = [], [], [], []
  for i in range(seconds * 100 + 1):
    elapsed = i / 100
    # Command-only periodic excitation, exactly equal extrema in train/holdout.
    value = 0.15 * math.sin(2 * math.pi * elapsed / 4.0)
    c, output, pose = synthetic_pair(owner, i, command=value)
    stamp = q + i * 10_000_000
    good = elapsed >= bad_until
    c = replace(c, control_ns=stamp, state_ns=stamp - 1_000_000, source_receipt_ns=stamp - 1_000_000, active=good)
    controls.append(c)
    frames.append(ReplayFrame(stamp, (), c.requested, (0.0, 0.0), healthy=good, frame_sequence=i))
    events.append(
      Emission(
        stamp,
        stamp - 1_000_000,
        stamp,
        output.projection,
        cert.binding.identity,
        cert.binding.source_epoch,
        cert.binding.baseline_revision,
        True,
        good,
        False,
        False,
        False,
        True,
        False,
        0,
        0,
        '',
      )
    )
    if i % 5 == 0:
      mono = stamp + cert.lag_ns
      poses.append(
        replace(
          pose,
          mono_ns=mono,
          source_lower_ns=mono,
          source_upper_ns=mono,
          source_timestamp_ns=mono,
          completed_ns=mono,
          observed_ns=mono,
          speed_mps=20.0 if good else 0.0,
        )
      )
  checkpoints = tuple((q + i * 1_000_000_000, Snapshot('synthetic-availability-only', ())) for i in range(seconds + 1))
  return cert.binding, q, tuple(frames), tuple(poses), tuple(events), tuple(controls), checkpoints


def choose(data, elapsed, *, cache=None):
  b, q, frames, poses, events, controls, checkpoints = data
  now = q + int(elapsed * 1e9)
  return cohorts.select_first_cohort(
    binding=b,
    epoch=1,
    epoch_start_ns=q,
    cutoff_ns=q + 100_000_000_000,
    now_ns=now,
    cache=cache,
    frames=tuple(row for row in frames if row.control_ns <= now),
    poses=tuple(row for row in poses if row.completed_ns <= now),
    emissions=tuple(row for row in events if row.apply_ns <= now),
    controls=tuple(row for row in controls if row.control_ns <= now),
    checkpoints=tuple(row for row in checkpoints if row[0] <= now),
  )


def test_initial_inactive_standstill_does_not_strand_later_good_same_epoch():
  data = source_data()
  assert choose(data, 40) is None
  selected = choose(data, 60)
  assert selected is not None
  assert selected['start_ns'] == data[1] + 12_000_000_000
  assert selected['epoch'] == 1
  assert selected['pose_counts'] == (80,) * 8


def test_response_values_never_choose_or_change_cohort():
  data = source_data()
  expected = choose(data, 60)
  changed = list(data)
  changed[3] = tuple(replace(p, yaw_rate=999.0, roll_rad=-1.0) for p in data[3])
  assert choose(tuple(changed), 60) == expected


def test_examined_anchors_are_not_rescanned_after_source_watermark(monkeypatch):
  data = source_data()
  cache = {'examined': {}, 'selected': None}
  calls = []
  original = cohorts.quality_for_start

  def counted(start, **kwargs):
    calls.append(start)
    return original(start, **kwargs)

  monkeypatch.setattr(cohorts, 'quality_for_start', counted)
  assert choose(data, 40, cache=cache) is None
  first_count = len(calls)
  assert choose(data, 40, cache=cache) is None
  assert len(calls) == first_count
  selected = choose(data, 60, cache=cache)
  assert selected is not None
  assert len(calls) == len(set(calls))
  assert choose(data, 61, cache=cache) == selected


def test_source_value_mutation_invalidates_cache_but_append_retains_it():
  keys = ((b'a',), (b'b',))
  cache = cohorts.new_schedule_cache(None, b'epoch', keys)
  cache['examined'][1] = False
  appended = cohorts.new_schedule_cache(cache, b'epoch', ((b'a', b'next'), (b'b',)))
  assert appended['examined'] == {1: False}
  assert not cohorts.new_schedule_cache(appended, b'epoch', ((b'changed',), (b'b',)))['examined']
  assert not cohorts.new_schedule_cache({'token': b'epoch'}, b'epoch', keys)['examined']


def test_interval_filter_is_source_only_and_minimum_applies_after_filter():
  data = source_data(bad_until=0)
  b, q, frames, poses, events, controls, _ = data
  selected = choose(data, 40)
  assert selected is not None
  lower = q + 2_600_000_000

  def quality(rows):
    return cohorts.quality_for_start(q, frames=frames, poses=rows, emissions=events, controls=controls, binding=b, now_ns=q + 61_000_000_000)

  inside = next(p for p in poses if p.mono_ns >= lower)
  partial = replace(inside, source_lower_ns=lower - 1)
  assert quality(tuple(partial if p is inside else p for p in poses))['pose_counts'][0] == 79
  unknown = replace(inside, source_lower_ns=0)
  assert quality(tuple(unknown if p is inside else p for p in poses)) is None
  reduced = tuple(p for p in poses if not lower <= p.mono_ns < lower + 2_050_000_000)
  assert quality(reduced) is None


def test_one_real_submission_per_epoch_even_after_response_failure(monkeypatch):
  from openpilot.starpilot.flm.live_tracking_actor import LiveTrackingActor
  from openpilot.starpilot.flm import live_tracking_bootstrap_producer
  from openpilot.starpilot.flm.tests.test_live_tracking_actor import PollProcess

  b = certificate().binding
  process = PollProcess()
  actor: Any = LiveTrackingActor(process)
  monkeypatch.setattr(actor, 'binding_for', lambda collector, source: b)
  collector = SimpleNamespace(ready=[None], original_source_capture=SimpleNamespace(epoch=1), live_tracking=None)
  calls = []

  def snapshot(*args):
    calls.append(1)
    return {'availability_scheduling_only': True}

  monkeypatch.setattr(live_tracking_bootstrap_producer, 'response_snapshot', snapshot)

  # Snapshot preparation is now asynchronous. Exercise the same epoch latch
  # across its separate completion; no source/native admission is mocked.
  class SnapshotPreparation:
    pid = None
    epoch = token = None
    reason = 'snapshot_submitted'
    pending = None

    def poll(self, *, token, now_ns):
      result, self.pending = self.pending, None
      self.pid = None
      return result

    def submit(self, collector, seed, *, token, now_ns):
      self.epoch, self.token, self.pid = collector.original_source_capture.epoch, token, 1
      self.pending = {'snapshot': live_tracking_bootstrap_producer.response_snapshot(collector, seed, now_ns)}

  actor.snapshot_process = SnapshotPreparation()
  monkeypatch.setattr(actor, '_same_bootstrap_source', lambda source, now_ns: True)
  actor.tick(collector, object(), now_ns=10, current_native=True)
  assert len(process.submissions) == 0
  actor.tick(collector, object(), now_ns=20, current_native=True)
  assert len(process.submissions) == 1
  # A real child outcome cannot authorize another heldout/cohort search.
  monkeypatch.setattr(process, 'poll', lambda **kwargs: {'production_admissible': True, 'status': 'collecting', 'reason': 'heldout_rmse'})
  actor.tick(collector, object(), now_ns=30, current_native=True)
  actor.tick(collector, object(), now_ns=40, current_native=True)
  assert len(calls) == len(process.submissions) == 1
  collector.original_source_capture.epoch = 2
  monkeypatch.setattr(process, 'poll', lambda **kwargs: None)
  actor.tick(collector, object(), now_ns=50, current_native=True)
  actor.tick(collector, object(), now_ns=60, current_native=True)
  assert len(process.submissions) == 2


def test_known_controller_quality_flags_cannot_consume_early_cohort():
  data = source_data(bad_until=0)
  for name in ('saturated', 'model_wobble', 'controller_chatter'):
    changed = list(data)
    changed[5] = tuple(replace(c, **{name: c.control_ns < data[1] + 12_000_000_000}) for c in data[5])
    assert choose(tuple(changed), 40) is None
    assert choose(tuple(changed), 60)['start_ns'] == data[1] + 12_000_000_000


def test_unknown_limit_is_not_reclassified_as_encoder_quantization():
  data = source_data(bad_until=0)
  changed = list(data)
  changed[5] = tuple(replace(c, limited=True) for c in data[5])
  # No registered raw stage/curvature hint: exact_unclipped_pair fails closed.
  assert choose(tuple(changed), 60) is None


def test_proven_exact_encoder_quantization_hint_keeps_command_support():
  data = source_data(bad_until=0)
  changed = list(data)
  changed[5] = tuple(
    replace(c, limited=True, stage_schema_revision='torque-selected-stage-snapshot-v1', replay_inputs=c.replay_inputs + (('curvature_limited', 0.0),))
    for c in data[5]
  )
  assert choose(tuple(changed), 40) is not None


def test_delayed_native_or_context_receipt_does_not_seal_failed_cohort():
  data = source_data(bad_until=0)
  q = data[1]
  # At39s the fixed38.8s cohort is geometrically complete. Later actual
  # native/control receipts may still be queued on their independent topics.
  for index in (4, 5):
    changed = list(data)
    changed[index] = tuple(row for row in data[index] if row.control_ns < q + 38_800_000_000)
    cache = {'examined': {}, 'selected': None}
    assert choose(tuple(changed), 39, cache=cache) is None
    assert q not in cache['examined']
    assert choose(data, 40, cache=cache) is not None
    assert cache['examined'][q] is True


def test_all_lag_lookback_rejects_gap_even_when_every_pose_has_lag_zero_endpoint():
  data = list(source_data(bad_until=0))
  q = data[1]
  # Lag-zero20Hz poses fall either side of this40ms application gap. Their
  # individual ages remain <=30ms, but the common all-lag tape is incomplete.
  data[4] = tuple(row for row in data[4] if not q + 3_030_000_000 <= row.apply_ns <= q + 3_050_000_000)
  b, q, frames, poses, events, controls, _ = data
  diagnostics = {}
  assert (
    cohorts.quality_for_start(q, frames=frames, poses=poses, emissions=events, controls=controls, binding=b, now_ns=q + 40_000_000_000, diagnostics=diagnostics)
    is None
  )
  assert diagnostics['reason'] == 'fixed_cohort_common_command_source_unavailable'
  selected = choose(tuple(data), 40)
  assert selected is not None and selected['start_ns'] == q + 1_000_000_000
  changed = list(data)
  changed[3] = tuple(replace(row, yaw_rate=-999.0, roll_rad=1.0) for row in data[3])
  assert choose(tuple(changed), 40) == selected


def test_any_good_lag_cannot_hide_a_different_lag_source_interval_crossing():
  data = list(source_data(bad_until=0))
  q = data[1]
  point = q + 3_020_000_000
  data[4] = tuple(replace(row, apply_ns=point + 1) if row.apply_ns == point else row for row in data[4])
  data[3] = tuple(
    replace(row, source_lower_ns=point, source_upper_ns=point + 2, completed_ns=point + 2, observed_ns=point + 2) if row.mono_ns == point else row
    for row in data[3]
  )
  b, q, frames, poses, events, controls, _ = data
  diagnostics = {}
  assert (
    cohorts.quality_for_start(q, frames=frames, poses=poses, emissions=events, controls=controls, binding=b, now_ns=q + 40_000_000_000, diagnostics=diagnostics)
    is None
  )
  assert diagnostics['reason'] == 'fixed_cohort_common_command_source_unavailable'
  selected = choose(tuple(data), 40)
  assert selected is not None and selected['start_ns'] == q + 1_000_000_000


def test_distinct_original_applications_may_repeat_one_consumed_control():
  from openpilot.starpilot.flm.live_tracking_bootstrap_producer import _epoch_rows
  import inspect

  ordering = {'order_name': 'apply_ns'} if 'order_name' in inspect.signature(_epoch_rows).parameters else {}
  data = source_data(bad_until=0)
  b, q, frames, poses, events, controls, checkpoints = data
  first = events[0]
  extra = replace(first, apply_ns=first.apply_ns + 1)
  repeated = (first, extra, *events[1:])
  assert _epoch_rows(repeated, Emission, 'control_ns', q, q + 100_000_000_000, **ordering) == repeated
  keys = cohorts.source_quality_keys(frames, poses, repeated, checkpoints, controls)
  assert len(keys[2]) == len(repeated)
  # Scheduling cannot merge, replace, restamp or renew either real application.
  assert repeated[0] is first and repeated[1] is extra
  assert first.control_ns == extra.control_ns and first.apply_ns != extra.apply_ns


def test_repeated_control_support_still_rejects_application_or_control_regression():
  import pytest
  from openpilot.starpilot.flm.live_tracking_bootstrap_producer import _epoch_rows
  import inspect

  ordering = {'order_name': 'apply_ns'} if 'order_name' in inspect.signature(_epoch_rows).parameters else {}
  data = source_data(bad_until=0)
  b, q, frames, poses, events, controls, checkpoints = data
  first, second = events[:2]
  for changed in ((first, replace(second, apply_ns=first.apply_ns)), (first, replace(second, control_ns=first.control_ns - 1))):
    with pytest.raises(ValueError):
      _epoch_rows(changed, Emission, 'control_ns', q, q + 100_000_000_000, **ordering)
    with pytest.raises(ValueError):
      cohorts.source_quality_keys(frames, poses, changed, checkpoints, controls)
  # Genuine controller ticks themselves still require strict unique clocks.
  with pytest.raises(ValueError):
    _epoch_rows((frames[0], frames[0]), ReplayFrame, 'control_ns', q, q + 100_000_000_000)


def test_source_complete_slots_exclude_unknown_lookbacks_before_any_response_is_read():
  data = source_data(bad_until=0)
  b, q, frames, poses, events, controls, _ = data
  # One actually unknown consumed command; surrounding genuine applications
  # remain in the FULL source inventory. Exclude its entire500ms lookback.
  bad = q + 5_000_000_000
  controls = tuple(replace(c, state_ns=c.control_ns - 40_000_000, source_receipt_ns=c.control_ns - 40_000_000) if c.control_ns == bad else c for c in controls)
  events = tuple(replace(e, state_ns=e.control_ns - 40_000_000) if e.control_ns == bad else e for e in events)
  kwargs = {'frames': frames, 'poses': poses, 'emissions': events, 'controls': controls, 'binding': b, 'now_ns': q + 61_000_000_000}
  assert cohorts.quality_for_start(q, **kwargs) is None
  selected = cohorts.quality_for_start(q, **kwargs, source_complete=True)
  assert selected is not None
  assert selected['start_ns'] == q
  assert all(40 <= count <= 240 for count in selected['pose_counts'])
  retained = tuple(t for group in selected['source_pose_ns'] for t in group)
  assert not any(bad <= t <= bad + 500_000_000 for t in retained)
  assert len(retained) < sum(
    1 for p in poses if any(q + 2_600_000_000 + k * 4_600_000_000 <= p.mono_ns < q + 6_600_000_000 + k * 4_600_000_000 for k in range(8))
  )
  changed = tuple(replace(p, yaw_rate=999.0, roll_rad=-1.0) for p in poses)
  assert cohorts.quality_for_start(q, **dict(kwargs, poses=changed), source_complete=True) == selected


def test_source_complete_slots_do_not_top_up_an_underfilled_original_slot():
  data = source_data(bad_until=0)
  b, q, frames, poses, events, controls, _ = data
  bad_start, bad_end = q + 3_000_000_000, q + 5_600_000_000
  controls = tuple(
    replace(c, state_ns=c.control_ns - 40_000_000, source_receipt_ns=c.control_ns - 40_000_000) if bad_start <= c.control_ns <= bad_end else c for c in controls
  )
  events = tuple(replace(e, state_ns=e.control_ns - 40_000_000) if bad_start <= e.control_ns <= bad_end else e for e in events)
  diagnostics = {}
  assert (
    cohorts.quality_for_start(
      q, frames=frames, poses=poses, emissions=events, controls=controls, binding=b, now_ns=q + 61_000_000_000, source_complete=True, diagnostics=diagnostics
    )
    is None
  )
  assert diagnostics['reason'] == 'fixed_slots_source_complete_pose_count_unavailable'
