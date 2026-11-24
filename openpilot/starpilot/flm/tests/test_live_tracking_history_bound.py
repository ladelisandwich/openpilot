from collections import OrderedDict, deque
from types import SimpleNamespace

import pytest

from openpilot.starpilot.flm.live_tracking_actor import LiveTrackingActor


def history(*, source_tracking, identified=False, identity='controller', pending=()):
  records = deque(SimpleNamespace(control_ns=1_000_000_000 + i * 10_000_000) for i in range(1000))
  events = OrderedDict((r.control_ns, SimpleNamespace(control_ns=r.control_ns, witness=SimpleNamespace(frame=i))) for i, r in enumerate(records))
  emissions = SimpleNamespace(
    events=events,
    by_control={r.control_ns: r for r in events.values()},
    decode_errors=dict.fromkeys(events),
    decode_queue=deque(1_000_000_000 + i * 10_000_000 for i in pending),
    decoder=SimpleNamespace(verified=dict.fromkeys(events)),
    carriers=OrderedDict((i, object()) for i in range(1000)),
  )
  replay = SimpleNamespace(
    raw_mode=True,
    identity=identity,
    baseline_revision='baseline',
    records=records,
    normalized=dict.fromkeys(r.control_ns for r in records),
    checkpoints=deque((r.control_ns, object()) for i, r in enumerate(records) if i % 100 == 0),
    trace_archive=None,
    emissions=emissions,
  )
  context = SimpleNamespace(controls=OrderedDict((r.control_ns, object()) for r in records), outputs=OrderedDict((r.control_ns, r) for r in records))
  archive = SimpleNamespace(
    publications=OrderedDict((r.control_ns, object()) for r in records), consumptions=OrderedDict((r.control_ns, object()) for r in records)
  )
  collector = SimpleNamespace(adapter=SimpleNamespace(replay=replay, original_context_receipts=context, original_feedback_archive=archive))
  binding = SimpleNamespace(identity='controller', baseline_revision='baseline')
  owner = SimpleNamespace(
    source_tracking=source_tracking,
    seed=binding,
    ingress=object() if identified else None,
    certificate=SimpleNamespace(binding=binding) if identified else None,
  )
  return owner, collector


def retire(owner, collector):
  for _ in range(40):
    LiveTrackingActor._retire_identified_history(owner, collector)


@pytest.mark.parametrize('source_tracking,identified,expected', [(True, False, 500), (False, False, 1000), (True, True, 500), (False, True, 500)])
def test_only_bound_source_or_identified_history(source_tracking, identified, expected):
  owner, collector = history(source_tracking=source_tracking, identified=identified)
  replay = collector.adapter.replay
  original = tuple(replay.records)
  retire(owner, collector)
  assert tuple(replay.records) == original[-expected:]
  assert {id(row) for row in replay.records} == {id(row) for row in original[-expected:]}
  assert replay.checkpoints[0][0] == replay.records[0].control_ns
  assert len(replay.normalized) == len(replay.emissions.events) == expected
  assert owner.ingress is None if not identified else owner.ingress is not None
  assert owner.certificate is None if not identified else owner.certificate is not None


@pytest.mark.parametrize('veto', ['unknown_source', 'wrong_identity', 'trace_archive', 'no_checkpoint', 'not_raw'])
def test_missing_exact_source_or_opaque_archive_does_not_retire(veto):
  owner, collector = history(source_tracking=True)
  replay = collector.adapter.replay
  if veto == 'unknown_source':
    owner.seed = None
  elif veto == 'wrong_identity':
    replay.identity = 'another'
  elif veto == 'trace_archive':
    replay.trace_archive = object()
  elif veto == 'no_checkpoint':
    replay.checkpoints.clear()
  elif veto == 'not_raw':
    replay.raw_mode = False
  original = tuple(replay.records)
  retire(owner, collector)
  assert tuple(replay.records) == original
  assert len(replay.emissions.events) == 1000


def test_unconsumed_can_and_its_original_context_are_never_skipped():
  owner, collector = history(source_tracking=True, pending=(100, 101))
  replay = collector.adapter.replay
  cutoff = 1_000_000_000 + 100 * 10_000_000
  original_event = replay.emissions.events[cutoff]
  retire(owner, collector)
  assert replay.emissions.decode_queue == deque((cutoff, cutoff + 10_000_000))
  assert replay.emissions.events[cutoff] is original_event
  assert next(iter(replay.emissions.events)) == cutoff
  assert next(iter(collector.adapter.original_context_receipts.controls)) == cutoff
  assert next(iter(collector.adapter.original_context_receipts.outputs)) == cutoff
  assert replay.emissions.decoder.verified[cutoff] is None
  assert len(replay.records) == 500


def test_each_tick_keeps_original_bounded_retirement_work():
  owner, collector = history(source_tracking=True)
  replay = collector.adapter.replay
  LiveTrackingActor._retire_identified_history(owner, collector)
  assert len(replay.records) == 968
  assert len(replay.emissions.events) == 968
  assert len(collector.adapter.original_context_receipts.controls) == 968
  assert len(collector.adapter.original_feedback_archive.publications) == 968
  assert owner.ingress is owner.certificate is None
