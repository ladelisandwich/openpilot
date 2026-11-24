"""Whole event selection retains complete source trajectories before budgets."""
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from openpilot.starpilot.flm.event_storage import select_whole_event_snapshot, source_windows


def engine(count=4000):
  frames = tuple(SimpleNamespace(control_ns=(index + 1) * 10_000_000, frame_sequence=index + 1, healthy=True)
                 for index in range(count))
  return SimpleNamespace(records=frames, replay_windows=(), _frame_context=lambda frame: frame.healthy,
                         checkpoints=tuple((frame.control_ns, None) for frame in frames[::100]),
                         identity='a' * 64, baseline_revision='baseline')


def bundle(start, end, index):
  # Source boundaries deliberately precede and follow admitted physical points.
  return {'block_id': str(index), 'event_windows': ((start, end, 'subtle'),),
          'samples': tuple({'control_ns': stamp, 'apply_ns': stamp, 'mono_ns': stamp + 100_000_000}
                           for stamp in range(start + 100_000_000, end - 90_000_000, 10_000_000))}


def test_complete_events_precede_bounded_storage_export():
  actual = engine()
  blocks = tuple(bundle((index * 100 + 2) * 10_000_000, (index * 100 + 100) * 10_000_000, index) for index in range(32))
  with patch('openpilot.starpilot.flm.event_storage.regroup_whole_events', return_value=blocks) as regroup:
    selected, windows = select_whole_event_snapshot(actual, ('all_original_packets',), object())
  assert len(selected) == 24
  assert selected == blocks[-24:]
  assert windows[0][0] < selected[0]['samples'][0]['control_ns']
  assert any(b > selected[-1]['samples'][-1]['control_ns'] for _, b in windows)
  # Fit windows end at actual complete episode recovery; the unrelated latest
  # caller cannot make a qualified historical fit disappear. Fresh authority
  # is a separate parent current_admission/current_effect_probe obligation.
  assert windows[-1][1] == selected[-1]['event_windows'][-1][1]
  assert regroup.call_args.args[1] == ('all_original_packets',)
  assert selected[0]['samples'] is blocks[-24]['samples']


def test_budget_discards_whole_bundle_never_truncates_it():
  actual = engine()
  blocks = tuple(bundle((index * 100 + 2) * 10_000_000, (index * 100 + 100) * 10_000_000, index) for index in range(32))
  with patch('openpilot.starpilot.flm.event_storage.regroup_whole_events', return_value=blocks):
    selected, windows = select_whole_event_snapshot(actual, (), object(), max_frames=400)
  assert 1 <= len(selected) <= 4
  assert all(block in blocks for block in selected)
  assert sum(sum(a <= frame.control_ns <= b for frame in actual.records) for a, b in windows) <= 400


def test_missing_source_and_health_split_independence():
  actual = engine(20)
  actual.records[5].healthy = False
  actual.records[10].frame_sequence += 5
  windows = source_windows(actual)
  assert len(windows) == 4
  assert not any(a <= actual.records[5].control_ns <= b for a, b in windows)
  assert not any(a < actual.records[10].control_ns < b for a, b in windows)


@pytest.mark.parametrize('bound', [0, 12001, 1.5, True])
def test_explicit_budget_bound(bound):
  with pytest.raises(ValueError):
    select_whole_event_snapshot(engine(), (), object(), max_frames=bound)
