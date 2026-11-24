"""Actual parent snapshot availability, not fabricated fit/native qualification."""

import pytest

from openpilot.starpilot.flm import source_plant_runtime as runtime
from openpilot.starpilot.flm.tests.test_compact_parent_timing import SECOND, START, parent_fixture


def subtle_parent(age_seconds):
  collector, now, exported = parent_fixture(age_seconds * SECOND)
  # A full 4.5-second small-amplitude cycle has one geometry bundle at20Hz.
  # Use actual whole_event_windows via runtime.snapshot; no geometry mock.
  for frame in collector.adapter.replay.records:
    elapsed = frame.control_ns - START
    frame.desired_physical_accel = (
      (0.15 if ((elapsed - 39 * SECOND) // 2_250_000_000) % 2 == 0 else -0.15)
      if elapsed >= 39 * SECOND else 0.
    )
  return collector, now, exported


@pytest.mark.parametrize('age_seconds,attempt,windows', [(80, False, 8), (85, True, 9)])
def test_actual_parent_waits_for_ninth_complete_subtle_cycle(age_seconds, attempt, windows):
  collector, now, exported = subtle_parent(age_seconds)
  assert runtime.snapshot(collector, now) is None
  assert bool(exported) is attempt
  assert collector.snapshot_attempts == int(attempt)
  assert collector.last_fit == (now if attempt else 0)
  if not attempt:
    readiness = collector.adapter.snapshot_readiness
    assert readiness['reason'] == 'plant_later_original_goal_geometry'
    assert readiness['candidateBundles'] == 8
    assert readiness['requiredBundles'] == 8
    assert readiness['completeGoalWindows'] == windows
    assert readiness['requiredCompleteGoalWindows'] == 9
    assert readiness['childQualification'] == 'not_yet_evaluated'
  else:
    assert exported == [(((START, now),), now)]
    # The parent-only fixture stops at export rather than inventing a packet.
    assert collector.adapter.snapshot_readiness['reason'] == 'plant_snapshot_unavailable'


def test_eighth_cycle_does_not_consume_epoch_or_cooldown():
  collector, now, exported = subtle_parent(80)
  assert runtime.snapshot(collector, now) is None
  assert not exported
  assert collector.snapshot_attempts == collector.last_fit == 0
  assert collector.adapter.source_plant_submitted_epoch == -1
  later, later_ns, _ = subtle_parent(85)
  collector.adapter.replay = later.adapter.replay
  collector.original_source_capture = later.original_source_capture
  collector.ready = later.ready
  assert runtime.snapshot(collector, later_ns) is None
  assert exported == [(((START, later_ns),), later_ns)]
  assert collector.snapshot_attempts == 1
  assert collector.last_fit == later_ns


def test_geometry_wait_does_not_enable_retry_of_submitted_epoch():
  collector, now, exported = subtle_parent(85)
  collector.adapter.source_plant_submitted_epoch = collector.original_source_capture.epoch
  assert runtime.snapshot(collector, now) is None
  assert not exported and collector.snapshot_attempts == collector.last_fit == 0
