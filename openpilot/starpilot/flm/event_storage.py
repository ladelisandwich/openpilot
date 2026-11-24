"""Choose complete physical events before applying bounded child replay budgets."""
from bisect import bisect_left, bisect_right
from types import SimpleNamespace

from openpilot.starpilot.flm.controller_replay import MAX_FRAME_GAP_NS
from openpilot.starpilot.flm.event_windows import regroup_whole_events


def source_windows(engine):
  if engine.replay_windows:
    return tuple(engine.replay_windows)
  windows = []
  start = previous = sequence = 0
  for frame in engine.records:
    healthy = engine._frame_context(frame)
    contiguous = previous and 0 < frame.control_ns - previous <= MAX_FRAME_GAP_NS and frame.frame_sequence == sequence + 1
    if not healthy or not contiguous or not start:
      if start:
        windows.append((start, previous))
      start = frame.control_ns if healthy else 0
    previous, sequence = frame.control_ns, frame.frame_sequence
  if start:
    windows.append((start, previous))
  return tuple(windows)


def select_whole_event_snapshot(engine, packets, objective, *, max_frames=12000, diagnostics=None):
  """Parent selection is a storage optimization, never child admission proof.

  All points of a chosen whole bundle and every actual controller frame from
  its genuine checkpoint through final neutral recovery remain present. No
  episode is shortened to meet a budget, and no missing source gap is filled.
  """
  if type(max_frames) is not int or not 1 <= max_frames <= 12000:
    raise ValueError('event complete replay budget')
  if diagnostics is not None:
    diagnostics.clear()
    diagnostics.update(sourceWindows=0, wholeBlocks=0, missingCheckpoint=0, frameBudget=0, windowBudget=0, selectedBlocks=0)
  windows = source_windows(engine)
  view = SimpleNamespace(records=engine.records, replay_windows=windows, _frame_context=engine._frame_context,
                         identity=engine.identity, baseline_revision=engine.baseline_revision)
  blocks = regroup_whole_events(view, packets, objective)
  if diagnostics is not None:
    diagnostics.update(sourceWindows=len(windows), wholeBlocks=len(blocks))
  frame_rows = tuple(engine.records)
  frame_stamps = tuple(frame.control_ns for frame in frame_rows)
  checkpoints = tuple(stamp for stamp, _ in engine.checkpoints)

  def complete_windows(chosen):
    result = []
    for block in chosen:
      boundaries = block['event_windows']
      first, last = boundaries[0][0], boundaries[-1][1]
      source = next(((a, b) for a, b in windows if a <= first <= last <= b), None)
      if source is None:
        raise ValueError('event source window missing')
      candidates = [stamp for stamp in checkpoints if source[0] <= stamp <= first]
      if not candidates:
        raise ValueError('event complete checkpoint missing')
      start = candidates[-1]
      # Event lineage uses crossing_ns - 1 for disjoint inclusive membership.
      # Replay endpoints must instead name the last actual tick in that set.
      index = bisect_right(frame_stamps, last) - 1
      if index < 0 or frame_stamps[index] < first:
        raise ValueError('event complete endpoint missing')
      if frame_stamps[index] != last:
        if index + 1 >= len(frame_rows):
          raise ValueError('event complete crossing missing')
        previous, crossing = frame_rows[index], frame_rows[index + 1]
        if (crossing.control_ns != last + 1 or crossing.frame_sequence != previous.frame_sequence + 1 or
            not 0 < crossing.control_ns - previous.control_ns <= MAX_FRAME_GAP_NS or
            not source[0] <= previous.control_ns < crossing.control_ns <= source[1] or
            not engine._frame_context(previous) or not engine._frame_context(crossing)):
          raise ValueError('event complete crossing discontinuity')
        last = previous.control_ns
      if result and start <= result[-1][1]:
        result[-1] = result[-1][0], max(result[-1][1], last)
      else:
        result.append((start, last))
    return tuple(result)

  selected = []
  selected_windows = ()
  for block in reversed(blocks):
    trial = [block, *selected]
    try:
      trial_windows = complete_windows(trial)
    except ValueError:
      if diagnostics is not None:
        diagnostics['missingCheckpoint'] += 1
      continue
    count = sum(bisect_right(frame_stamps, b) - bisect_left(frame_stamps, a) for a, b in trial_windows)
    if len(trial_windows) > 24 or count > max_frames:
      if diagnostics is not None:
        diagnostics['windowBudget' if len(trial_windows) > 24 else 'frameBudget'] += 1
      continue
    selected, selected_windows = trial, trial_windows
    if len(selected) == 24:
      break
  if diagnostics is not None:
    diagnostics['selectedBlocks'] = len(selected)
  return tuple(selected), selected_windows
