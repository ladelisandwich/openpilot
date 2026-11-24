"""Bounded actual wire/collector transport units; no road or learned-fit claim."""
from collections import deque
from dataclasses import replace
from types import SimpleNamespace

import pytest

from openpilot.starpilot.flm.fullrate_collector import (
  FullrateCollector, MAX_DRAIN, MAX_SNAPSHOT_WIRE_BYTES, MAX_WIRE_BYTES, drain, drain_pair,
)
from openpilot.starpilot.flm.learning_wire import read_applied, read_control
from openpilot.starpilot.flm.tests.test_fullrate_pipeline import SyntheticAdapter, snapshots, serialized_control, serialized_output

BASE = 10_000_000_000


def transport(controls=(), outputs=()):
  sockets = object(), object()
  queues = {sockets[0]: deque(controls), sockets[1]: deque(outputs)}
  reads = dict.fromkeys(sockets, 0)

  def receive(socket):
    reads[socket] += 1
    return queues[socket].popleft() if queues[socket] else None

  return sockets, receive, reads


def exercise(controls, outputs, now, *, paired=True):
  collector = FullrateCollector(SyntheticAdapter())
  sockets, receive, reads = transport(controls, outputs)
  seen = []

  def control(event, current, stamp):
    row = read_control(event)
    accepted = current.control(row, stamp)
    seen.append(('control', row.control_ns, accepted))

  def output(event, current, stamp):
    row = read_applied(event)
    accepted = current.output(row, stamp)
    seen.append(('output', row.apply_ns, accepted))

  if paired:
    result = drain_pair(sockets[0], control, sockets[1], output, receive, collector, now)
  else:
    result = drain(sockets[0], control, receive, collector, now)
    result = drain(sockets[1], output, receive, collector, now) and result
  return result, collector, seen, reads


@pytest.mark.parametrize('offsets,expected', [((0, 150_000_000), 1), ((140_000_000, 150_000_000), 2), ((0, 10_000_000), 0)])
def test_original_mixed_fresh_and_stale_rows_keep_same_rejections(offsets, expected):
  pairs = [snapshots(BASE + offset)[:2] for offset in offsets]
  controls = [serialized_control(c) for c, _ in pairs]
  outputs = [serialized_output(o) for _, o in pairs]
  result, collector, seen, _ = exercise(controls, outputs, BASE + 160_000_000)
  old_result, old, old_seen, _ = exercise(controls, outputs, BASE + 160_000_000, paired=False)
  assert result and old_result
  assert len(collector.ready) == expected
  assert sorted(seen) == sorted(old_seen)  # Neither stale original row was admitted.
  assert collector.training_interruptions == old.training_interruptions
  assert all(row.control_ns >= BASE + 60_000_000 for row, _ in collector.ready)
  if offsets == (0, 150_000_000):
    assert not old.ready  # The old output used to erase the fresh queued control.


def small_event(stamp, *, size=8, kind='fixture', snapshot=(), ford_rows=(), ford_version=0):
  return SimpleNamespace(logMonoTime=stamp, as_builder=lambda: SimpleNamespace(to_bytes=lambda: bytes(size)),
    which=lambda: kind,
    starpilotLateralState=SimpleNamespace(learningEvidence=SimpleNamespace(replaySnapshot=snapshot)),
    starpilotCarState=SimpleNamespace(fordLearningEvidence=SimpleNamespace(version=ford_version, rows=ford_rows)))


def ordered(controls, outputs):
  collector = FullrateCollector(SyntheticAdapter())
  sockets, receive, reads = transport(controls, outputs)
  seen = []

  def callback(kind):
    return lambda event, _collector, now: seen.append((kind, event, now))

  result = drain_pair(sockets[0], callback('control'), sockets[1], callback('output'), receive, collector, BASE)
  return result, collector, seen, sockets, reads


def test_equal_publisher_headers_choose_control_first_without_reordering_either_fifo():
  controls = [small_event(BASE), small_event(BASE)]
  outputs = [small_event(BASE), small_event(BASE)]
  result, _, seen, _, _ = ordered(controls, outputs)
  assert result
  assert [(kind, event) for kind, event, _ in seen] == [('control', e) for e in controls] + [('output', e) for e in outputs]
  assert all(now == BASE for _, _, now in seen)


def test_only_queue_heads_compete_never_sort_a_producers_regression_away():
  controls = [small_event(BASE + 30), small_event(BASE + 10)]
  outputs = [small_event(BASE + 20), small_event(BASE + 40)]
  result, _, seen, _, _ = ordered(controls, outputs)
  assert result
  assert [(kind, event.logMonoTime) for kind, event, _ in seen] == [
    ('output', BASE + 20), ('control', BASE + 30), ('control', BASE + 10), ('output', BASE + 40)]


def test_real_control_sequence_regression_still_rejects_and_clears():
  controls = [serialized_control(snapshots(BASE + offset)[0]) for offset in (20_000_000, 10_000_000)]
  result, collector, seen, _ = exercise(controls, [], BASE + 30_000_000)
  assert result and [accepted for _, _, accepted in seen] == [True, False]
  assert collector.reason == 'reordered_control' and not collector.controls and not collector.ready


def test_original_publisher_header_not_repeated_consumed_command_clock_is_ordering_key():
  control, output, _ = snapshots(BASE)
  event = serialized_control(control).as_builder()
  event.logMonoTime = BASE + 5_000_000  # Card can publish before Controls finishes its feedback.
  applied = serialized_output(output).as_builder()
  applied.logMonoTime = BASE + 2_000_000
  result, _, seen, _, _ = ordered([event.as_reader()], [applied.as_reader()])
  assert result and [kind for kind, _, _ in seen] == ['output', 'control']
  # The transport never restamps or replaces the embedded original clocks.
  assert read_control(seen[1][1]).control_ns == BASE
  assert read_applied(seen[0][1]).apply_ns == output.apply_ns


@pytest.mark.parametrize('absent', ('control', 'output', 'both'))
def test_one_producer_absent_is_nonblocking_and_does_not_create_evidence(absent):
  controls = [] if absent in ('control', 'both') else [small_event(BASE)]
  outputs = [] if absent in ('output', 'both') else [small_event(BASE)]
  result, collector, seen, sockets, reads = ordered(controls, outputs)
  assert result and len(seen) == len(controls) + len(outputs)
  assert reads[sockets[0]] == len(controls) + 1 and reads[sockets[1]] == len(outputs) + 1
  assert not collector.ready and not collector.controls and not collector.applied


@pytest.mark.parametrize('field,value', [('logMonoTime', 0), ('logMonoTime', -1), ('logMonoTime', True), ('logMonoTime', 1.0)])
def test_invalid_ordering_header_is_malformed_not_a_freshness_substitute(field, value):
  event = small_event(BASE)
  setattr(event, field, value)
  result, collector, _, _, _ = ordered([event], [])
  assert not result and collector.reason == 'malformed_evidence'


@pytest.mark.parametrize('side', (0, 1))
@pytest.mark.parametrize('size,kind,snapshot,ford_rows,ford_version,allowed', [
  (MAX_WIRE_BYTES, 'fixture', (), (), 0, True),
  (MAX_WIRE_BYTES + 1, 'fixture', (), (), 0, False),
  (MAX_WIRE_BYTES + 1, 'starpilotLateralState', (), (), 0, False),
  (MAX_SNAPSHOT_WIRE_BYTES, 'starpilotLateralState', (1,), (), 0, True),
  (MAX_SNAPSHOT_WIRE_BYTES + 1, 'starpilotLateralState', (1,), (), 0, False),
  (MAX_WIRE_BYTES + 1, 'starpilotCarState', (), (1,), 1, True),
  (MAX_WIRE_BYTES + 1, 'starpilotCarState', (), (1,), 0, False),
  (MAX_WIRE_BYTES + 1, 'starpilotCarState', (), (1,) * 5, 1, False),
])
def test_exact_existing_wire_bounds_on_both_streams(side, size, kind, snapshot, ford_rows, ford_version, allowed):
  rows = [[], []]
  rows[side] = [small_event(BASE, size=size, kind=kind, snapshot=snapshot, ford_rows=ford_rows, ford_version=ford_version)]
  result, collector, seen, _, _ = ordered(*rows)
  assert result is allowed
  assert len(seen) == int(allowed)
  if not allowed:
    assert collector.reason == 'malformed_evidence' and not collector.ready


@pytest.mark.parametrize('side', (0, 1))
@pytest.mark.parametrize('error', (AttributeError, KeyError, TypeError, ValueError, OverflowError))
def test_malformed_callback_keeps_original_reset_boundary(side, error):
  collector = FullrateCollector(SyntheticAdapter())
  c, o, _ = snapshots(BASE)
  collector.control(c, BASE + 1_000_000)
  collector.output(o, BASE + 1_000_000)
  assert collector.ready
  rows = [[], []]
  rows[side] = [small_event(BASE)]
  sockets, receive, _ = transport(*rows)

  def malformed(*_args):
    raise error('declared malformed fixture')

  assert not drain_pair(sockets[0], malformed, sockets[1], malformed, receive, collector, BASE)
  assert collector.reason == 'malformed_evidence' and not collector.ready and not collector.blocks


@pytest.mark.parametrize('side', (0, 1))
@pytest.mark.parametrize('count', (MAX_DRAIN, MAX_DRAIN + 1, MAX_DRAIN + 2))
def test_per_stream_bound_and_single_lookahead_are_not_silently_conflated(side, count):
  rows = [[], []]
  rows[side] = [small_event(BASE + i) for i in range(count)]
  result, collector, seen, sockets, reads = ordered(*rows)
  assert len(seen) == MAX_DRAIN
  assert reads[sockets[side]] == MAX_DRAIN + 1
  assert reads[sockets[1 - side]] == 1
  assert result is (count == MAX_DRAIN)
  assert collector.dropped == int(count > MAX_DRAIN)
  if count > MAX_DRAIN:
    assert collector.reason == 'fullrate_overrun' and not collector.ready


def test_both_streams_have_independent_bounds():
  controls = [small_event(BASE + i * 2) for i in range(MAX_DRAIN)]
  outputs = [small_event(BASE + i * 2 + 1) for i in range(MAX_DRAIN)]
  result, _, seen, sockets, reads = ordered(controls, outputs)
  assert result and len(seen) == 2 * MAX_DRAIN
  assert list(reads.values()) == [MAX_DRAIN + 1] * 2
  assert [event.logMonoTime for _, event, _ in seen] == list(range(BASE, BASE + 2 * MAX_DRAIN))


def test_output_native_denial_is_not_bypassed_by_fresh_cross_queue_join():
  c, o, _ = snapshots(BASE)
  result, collector, _, _ = exercise([serialized_control(c)], [serialized_output(replace(o, native_allowed=False))], BASE + 2_000_000)
  assert result and not collector.ready and not collector.points


def test_monotonic_publisher_headers_cannot_hide_regressed_control_after_an_intervening_join():
  new_c, new_o, _ = snapshots(BASE + 20_000_000)
  old_c, old_o, _ = snapshots(BASE + 10_000_000)

  def header(event, stamp):
    message = event.as_builder()
    message.logMonoTime = stamp
    return message.as_reader()

  controls = [serialized_control(new_c), header(serialized_control(old_c), BASE + 30_000_000)]
  outputs = [serialized_output(new_o), header(serialized_output(old_o), BASE + 31_000_000)]
  result, collector, seen, _ = exercise(controls, outputs, BASE + 40_000_000)
  assert result
  assert seen[0][0] == 'control' and seen[1][0] == 'output'
  assert seen[2] == ('control', old_c.control_ns, False)
  assert collector.training_interruptions['reordered_control'] == 1
  assert not collector.ready and old_c.control_ns not in collector.recent_controls


@pytest.mark.parametrize('mutated', (False, True))
def test_repeated_applied_header_and_duplicate_body_are_not_rewritten(mutated):
  control, output, _ = snapshots(BASE)
  duplicate = replace(output, applied=output.applied + 0.1) if mutated else output
  result, collector, seen, _ = exercise([serialized_control(control)],
    [serialized_output(output), serialized_output(duplicate)], BASE + 2_000_000)
  assert result
  assert seen[-1] == ('output', output.apply_ns, not mutated)
  assert len(collector.ready) == int(not mutated)
  if mutated:
    assert collector.reason == 'changed_duplicate_output'


def test_applied_before_feedback_can_join_without_rewriting_original_clocks():
  collector = FullrateCollector(SyntheticAdapter())
  seed = snapshots(BASE - 10_000_000)[0]
  assert collector.control(seed, BASE + 10_000_000)
  control, output, _ = snapshots(BASE)
  feedback = serialized_control(control).as_builder()
  feedback.logMonoTime = BASE + 5_000_000
  sockets, receive, _ = transport([feedback.as_reader()], [serialized_output(output)])
  order = []

  def consume_control(event, current, stamp):
    order.append('control')
    assert current.control(read_control(event), stamp)

  def consume_output(event, current, stamp):
    order.append('output')
    assert current.output(read_applied(event), stamp)
    assert not current.ready

  assert drain_pair(sockets[0], consume_control, sockets[1], consume_output, receive, collector, BASE + 10_000_000)
  assert order == ['output', 'control']
  assert list(collector.ready) == [(control, output)]


@pytest.mark.parametrize('source_offset,header_offset,reason', [
  (0, 1_000_000_000, 'stale_control'),
  (200_000_000, 10_000_000, 'invalid_control'),
])
def test_publisher_ordering_header_cannot_replace_stale_or_future_original_source_clock(source_offset, header_offset, reason):
  control = snapshots(BASE + source_offset)[0]
  event = serialized_control(control).as_builder()
  event.logMonoTime = BASE + header_offset
  result, collector, seen, _ = exercise([event.as_reader()], [], BASE + 160_000_000)
  assert result and seen == [('control', control.control_ns, False)]
  assert collector.reason == reason and not collector.ready and not collector.controls


@pytest.mark.parametrize('overrun_side', (0, 1))
def test_mixed_queue_overrun_remains_whole_maneuver_reset(overrun_side):
  collector = FullrateCollector(SyntheticAdapter())
  c, o, _ = snapshots(BASE)
  assert collector.control(c, BASE + 1_000_000)
  assert collector.output(o, BASE + 1_000_000) and collector.ready
  rows = [[small_event(BASE + 2 * i + side) for i in range(MAX_DRAIN + 1 if side == overrun_side else 8)] for side in (0, 1)]
  sockets, receive, reads = transport(*rows)
  counts = [0, 0]

  def callback(side):
    def consume(_event, _current, _now):
      counts[side] += 1
    return consume

  assert not drain_pair(sockets[0], callback(0), sockets[1], callback(1), receive, collector, BASE)
  assert counts[overrun_side] == MAX_DRAIN and counts[1 - overrun_side] == 8
  assert reads[sockets[overrun_side]] == MAX_DRAIN + 1 and reads[sockets[1 - overrun_side]] == 9
  assert collector.dropped == 1 and collector.reason == 'fullrate_overrun'
  assert not collector.controls and not collector.applied and not collector.ready and not collector.points
