"""Producer regression: logical crossing boundaries must name actual replay ticks."""
from unittest.mock import patch
import pytest
from openpilot.starpilot.flm.event_storage import select_whole_event_snapshot
from openpilot.starpilot.flm.tests.test_event_storage import engine, bundle


def selected(engine_value, logical_end):
  block = bundle(20_000_000, logical_end, 0)
  with patch('openpilot.starpilot.flm.event_storage.regroup_whole_events', return_value=(block,)):
    return block, select_whole_event_snapshot(engine_value, (), object())


def test_crossing_sentinel_uses_prior_real_tick_with_identical_membership():
  actual = engine(200)
  block, (blocks, windows) = selected(actual, 999_999_999)
  assert blocks == (block,)
  assert windows == ((10_000_000, 990_000_000),)
  assert tuple(f.control_ns for f in actual.records if 10_000_000 <= f.control_ns <= 999_999_999) == tuple(
    f.control_ns for f in actual.records if windows[0][0] <= f.control_ns <= windows[0][1])
  assert all(windows[0][0] <= p['control_ns'] <= windows[0][1] for p in block['samples'])


def test_existing_actual_recovery_endpoint_is_unchanged():
  actual = engine(200)
  block, (blocks, windows) = selected(actual, 1_000_000_000)
  assert blocks == (block,)
  assert windows == ((10_000_000, 1_000_000_000),)


@pytest.mark.parametrize('logical_end', (995_000_000, 999_999_998))
def test_arbitrary_missing_tick_is_not_normalized(logical_end):
  _, (blocks, windows) = selected(engine(200), logical_end)
  assert not blocks and not windows


def test_absent_crossing_does_not_invent_a_boundary_tick():
  actual = engine(200)
  actual.records = tuple(f for f in actual.records if f.control_ns != 1_000_000_000)
  _, (blocks, windows) = selected(actual, 999_999_999)
  assert not blocks and not windows


def test_truncated_source_keeps_missing_last_tick_rejected():
  actual = engine(99)
  _, (blocks, windows) = selected(actual, 999_999_999)
  assert not blocks and not windows


def test_nonconsecutive_crossing_sequence_is_rejected():
  actual = engine(200)
  actual.records[99].frame_sequence += 1
  _, (blocks, windows) = selected(actual, 999_999_999)
  assert not blocks and not windows


def test_crossing_clock_gap_is_rejected():
  actual = engine(200)
  actual.records = tuple(f for f in actual.records if not 900_000_000 <= f.control_ns < 1_000_000_000)
  # Keep sequence continuity so the clock gap independently supplies the veto.
  for sequence, frame in enumerate(actual.records, 1):
    frame.frame_sequence = sequence
  _, (blocks, windows) = selected(actual, 999_999_999)
  assert not blocks and not windows


def test_unhealthy_crossing_is_rejected():
  actual = engine(200)
  actual.records[99].healthy = False
  _, (blocks, windows) = selected(actual, 999_999_999)
  assert not blocks and not windows
