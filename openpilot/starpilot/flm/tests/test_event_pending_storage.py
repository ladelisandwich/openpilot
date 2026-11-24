"""Storage-only synthetic row identity, not physical/native qualification."""
from typing import Protocol, cast

import pytest
from openpilot.starpilot.flm.event_collector import EventFullrateCollector
from openpilot.starpilot.flm.fullrate_collector import RawPoint
from openpilot.starpilot.flm.tests.test_fullrate_pipeline import SyntheticAdapter, snapshots, BASE


class _StorageTokens(Protocol):
  """Opaque identity tokens for storage tests, not production context tuples."""
  context: tuple[str] | None
  physical_context: tuple[str] | None


def setup():
  collector = EventFullrateCollector(SyntheticAdapter())
  cast(_StorageTokens, collector).context = ('source',)
  cast(_StorageTokens, collector).physical_context = ('physical',)
  collector.cohort = 'event-storage.source'
  control, output, _ = snapshots(BASE)
  def rows(start, count):
    return tuple(RawPoint(control.control_ns, output.apply_ns, BASE + start + i,
      control, output, {'original': start + i}, ('physical',)) for i in range(count))
  return collector, rows


@pytest.mark.parametrize('first,second', ((1,39),(20,20),(39,1),(39,40),(39,80)))
def test_original_fragments_survive_only_as_storage_and_reconstruct_exact_rows(first, second):
  collector, rows = setup()
  a, b = rows(0, first), rows(first, second)
  collector.raw_points.extend(a)
  collector.active_since = BASE
  collector.last_apply = BASE + 1
  collector.split('physical-interruption')
  assert collector._pending_storage == a and not collector.blocks
  assert collector.active_since == collector.last_apply == 0 and not collector.ready
  collector.raw_points.extend(b)
  collector.split('another-interruption')
  saved = tuple(point for block in collector.blocks for point in block.samples)
  actual = saved + collector._pending_storage
  assert actual == a + b
  assert all(x is y for x, y in zip(actual, a + b, strict=True))
  assert len(collector._pending_storage) < 40
  assert all(40 <= len(block.samples) <= 80 for block in collector.blocks)
  before = actual
  collector.split('empty-interruption')
  assert tuple(point for block in collector.blocks for point in block.samples) + collector._pending_storage == before


@pytest.mark.parametrize('count', (1, 39, 40, 79, 80))
def test_hard_reset_discards_pending_current_and_sealed_history(count):
  collector, rows = setup()
  collector.raw_points.extend(rows(0, 39))
  collector.split('ordinary-break')
  collector.raw_points.extend(rows(39, count))
  collector.split('identity-boundary', reset=True)
  assert not collector._pending_storage and not collector.raw_points and not collector.blocks
  assert collector.last_qualified_response == 0 and not collector.ready


def test_accepted_target_rollover_drops_pending_and_current_storage():
  collector, rows = setup()
  collector.raw_points.extend(rows(0, 39))
  collector.split('ordinary-break')
  collector.raw_points.extend(rows(39, 20))
  collector._rollover_training()
  assert not collector._pending_storage and not collector.raw_points and not collector.blocks
  assert collector.active_since == 0 and collector.last_qualified_response == 0


def test_empty_context_cannot_park_unqualified_rows_and_packet_history_stays_bounded():
  collector, rows = setup()
  collector.context = None
  collector.raw_points.extend(rows(0, 39))
  collector.split('no-context')
  assert not collector._pending_storage and not collector.raw_points
  collector.context = ('source',)
  for i in range(70):
    collector.raw_points.extend(rows(i * 80, 80))
    collector.split('ordinary-break')
  assert len(collector.blocks) == 64 and not collector._pending_storage


@pytest.mark.parametrize('boundary', ('source', 'physical', 'target'))
def test_pending_never_crosses_source_physical_or_accepted_target_change(boundary):
  collector, rows = setup()
  old = rows(0, 39)
  collector.raw_points.extend(old)
  collector.split('ordinary-break')
  assert collector._pending_storage == old
  if boundary == 'source':
    collector.context = ('new-source',)
  elif boundary == 'physical':
    collector.physical_context = ('new-physical',)
  else:
    collector.adapter.actual_target_sha256 = 'new-accepted-target'
  from dataclasses import replace
  new = replace(rows(39, 1)[0], physical_context=collector.physical_context)
  collector.raw_points.append(new)
  collector.split('new-boundary')
  assert not collector.blocks
  assert collector._pending_storage == (new,)
  assert collector._pending_storage[0] is new


@pytest.mark.parametrize('fault', ('duplicate', 'backward-control', 'backward-apply', 'backward-response', 'foreign-physical'))
def test_duplicate_backward_and_foreign_rows_fail_closed_without_export(fault):
  from dataclasses import replace
  collector, rows = setup()
  first = rows(0, 39)
  collector.raw_points.extend(first)
  collector.split('ordinary-break')
  new = rows(39, 1)[0]
  if fault == 'duplicate':
    new = first[-1]
  elif fault == 'backward-control':
    new = replace(new, control_ns=new.control_ns - 1)
  elif fault == 'backward-apply':
    new = replace(new, apply_ns=new.apply_ns - 1)
  elif fault == 'backward-response':
    new = replace(new, mono_ns=first[-1].mono_ns - 1)
  else:
    new = replace(new, physical_context=('foreign',))
  collector.raw_points.append(new)
  collector.split('malformed-fragment')
  assert collector.training_interruptions['event_storage_lineage_or_chronology'] == 1
  assert not collector._pending_storage and not collector.blocks


def test_thirty_nine_unexportable_rows_never_become_a_packet_by_themselves():
  collector, rows = setup()
  original = rows(0, 39)
  collector.raw_points.extend(original)
  for _ in range(20):
    collector.split('ordinary-break')
    assert not collector.blocks
    assert collector._pending_storage == original



def test_late_row_cannot_precede_previously_sealed_packet_when_pending_is_empty():
  collector, rows = setup()
  collector.raw_points.extend(rows(0, 40))
  collector.split('first-packet')
  assert len(collector.blocks) == 1 and not collector._pending_storage
  collector.raw_points.append(rows(39, 1)[0])
  collector.split('late-duplicate')
  assert not collector.blocks and not collector._pending_storage
  assert collector.training_interruptions['event_storage_lineage_or_chronology'] == 1



def snapshot_ready_collector(result):
  from unittest.mock import Mock
  collector, rows = setup()
  for i in range(8):
    collector.raw_points.extend(rows(i * 40, 40))
    collector.split('storage-boundary')
  control, output, _ = snapshots(BASE)
  collector.ready.append((control, output))
  collector.adapter.fit_job_snapshot = Mock(return_value=result)
  return collector


def test_none_snapshot_selection_is_bounded_by_original_sixty_second_cadence():
  collector = snapshot_ready_collector(None)
  assert collector.job_snapshot(BASE) is None
  for elapsed in (0, 1, 50_000_000, 59_999_999_999):
    assert collector.job_snapshot(BASE + elapsed) is None
  assert collector.adapter.fit_job_snapshot.call_count == 1
  assert collector.job_snapshot(BASE + 60_000_000_000) is None
  assert collector.adapter.fit_job_snapshot.call_count == 2
  assert collector.sequence == 2
  assert collector.last_fit == BASE + 60_000_000_000
  assert collector.fit_block_sequence == 0  # No actual submission fabricated.


def test_snapshot_result_still_requires_original_explicit_submission_marker():
  sentinel = {'storage_test_only': True}
  collector = snapshot_ready_collector(sentinel)
  assert collector.job_snapshot(BASE) is sentinel
  assert collector.fit_block_sequence == 0
  collector.mark_job_submitted(BASE)
  assert collector.fit_block_sequence == collector.block_sequence
  assert collector.last_fit == BASE
  assert collector.job_snapshot(BASE + 60_000_000_000) is None
  assert collector.adapter.fit_job_snapshot.call_count == 1
