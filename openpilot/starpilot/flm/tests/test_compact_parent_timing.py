"""Parent availability/attempt policy only; no fit or native proof is invented."""
from dataclasses import replace
from types import SimpleNamespace

import pytest

from openpilot.starpilot.flm import source_plant_runtime as runtime
from openpilot.starpilot.flm.original_source_capture import OriginalSourceCapture, MAX_CAPTURE_SPAN_NS
from openpilot.starpilot.flm.source_plant_fit import FIRST_SLOT_NS, BLOCK_NS, PURGE_NS
from openpilot.starpilot.flm.tests.test_fullrate_pipeline import snapshots

SECOND = 1_000_000_000
START = 200 * SECOND


def parent_fixture(age_ns, *, pose_hz=20, with_goals=True, last_attempt=0):
  now = START + age_ns
  control, output, _ = snapshots(now - 1_000_000)
  # Existing ordinary-car evidence contract: no native ACK is manufactured.
  control = replace(control, native_required=False, native_allowed=False, native_identity='',
                    native_observed_ns=0, native_expires_ns=0)
  records = []
  for tick in range(age_ns // 10_000_000 + 1):
    elapsed = tick * 10_000_000
    goal = (0.3 if ((elapsed - 39 * SECOND) // (2 * SECOND)) % 2 == 0 else -0.3) if with_goals and elapsed >= 39 * SECOND else 0.
    records.append(SimpleNamespace(control_ns=START + elapsed, frame_sequence=tick + 1,
                                   desired_physical_accel=goal))
  capture = SimpleNamespace(epoch=0, epoch_start_ns=START, epoch_cutoff_ns=START + MAX_CAPTURE_SPAN_NS,
    rows=tuple(SimpleNamespace(mono_ns=stamp) for stamp in range(START, now + 1, SECOND // pose_hz)))
  exported = []

  def original_source_packet(collector, windows, observed_ns):
    exported.append((windows, observed_ns))
    # A parent-selection fixture must never fabricate a qualified child packet.
    raise ValueError('parent-only export boundary reached')

  adapter = SimpleNamespace(envelope=object(), source_plant_submitted_epoch=-1, identity=control.identity,
    baseline_revision=control.baseline_revision, snapshot_readiness=None,
    replay=SimpleNamespace(records=tuple(records), checkpoints=((START, object()),), _frame_context=lambda frame: True),
    original_source_packet=original_source_packet)
  collector = SimpleNamespace(adapter=adapter, original_source_capture=capture, ready=[(control, output)],
    last_fit=last_attempt, snapshot_attempts=0)
  assert runtime._current_fit_pair_reason(collector, now) is None
  return collector, now, exported


def test_predeclared_constants_and_plant_design_remain_separate():
  assert runtime.MIN_NORMAL_JOB_AGE_NS == 60 * SECOND
  assert runtime.NORMAL_JOB_ATTEMPT_SPACING_NS == 80 * SECOND
  assert MAX_CAPTURE_SPAN_NS == 100 * SECOND
  assert FIRST_SLOT_NS + 7 * (BLOCK_NS + PURGE_NS) + BLOCK_NS == 38_800_000_000


@pytest.mark.parametrize('age_ns,attempt', [(60 * SECOND - 1, False), (60 * SECOND, True)])
def test_complete_parent_geometry_first_attempt_boundary(age_ns, attempt):
  collector, now, exported = parent_fixture(age_ns)
  assert runtime.snapshot(collector, now) is None
  assert bool(exported) is attempt
  assert collector.snapshot_attempts == int(attempt)
  assert collector.last_fit == (now if attempt else 0)
  if attempt:
    assert exported == [(((START, now - age_ns % 10_000_000),), now)]
    assert collector.adapter.snapshot_readiness['reason'] == 'plant_snapshot_unavailable'
  else:
    assert collector.adapter.snapshot_readiness['reason'] == 'plant_fixed_cohort_still_collecting'


@pytest.mark.parametrize('pose_hz,goals', [(20, False), (10, True)])
def test_incomplete_geometry_never_consumes_attempt_or_cooldown(pose_hz, goals):
  collector, now, exported = parent_fixture(60 * SECOND, pose_hz=pose_hz, with_goals=goals)
  assert runtime.snapshot(collector, now) is None
  assert exported == []
  assert collector.snapshot_attempts == collector.last_fit == 0
  assert collector.adapter.snapshot_readiness['reason'] == 'plant_later_original_goal_geometry'
  assert collector.adapter.snapshot_readiness['candidateBundles'] < 8


@pytest.mark.parametrize('elapsed,attempt', [(80 * SECOND - 1, False), (80 * SECOND, True)])
def test_actual_attempt_spacing_boundary(elapsed, attempt):
  collector, now, exported = parent_fixture(90 * SECOND)
  collector.last_fit = now - elapsed
  previous = collector.last_fit
  assert runtime.snapshot(collector, now) is None
  assert bool(exported) is attempt
  assert collector.snapshot_attempts == int(attempt)
  assert collector.last_fit == (now if attempt else previous)


def test_submitted_epoch_still_forbids_another_attempt():
  collector, now, exported = parent_fixture(90 * SECOND)
  collector.adapter.source_plant_submitted_epoch = collector.original_source_capture.epoch
  assert runtime.snapshot(collector, now) is None
  assert not exported and collector.snapshot_attempts == 0


def test_real_capture_keeps_fixed_hundred_second_epoch_and_no_reselection():
  capture = OriginalSourceCapture()
  receipt = {'epoch_id': 'first', 'start_ns': START, 'cutoff_ns': START + 100 * SECOND,
    'identity': '1' * 64, 'source_epoch': '2' * 64, 'baseline_revision': 'baseline'}
  capture.begin_epoch(receipt)
  ordinal = capture.epoch
  with pytest.raises(ValueError, match='cannot reselect'):
    capture.begin_epoch({**receipt, 'epoch_id': 'overlap', 'start_ns': START + 60 * SECOND,
                         'cutoff_ns': START + 160 * SECOND})
  assert capture.epoch == ordinal and capture.epoch_cutoff_ns == START + 100 * SECOND
  capture.begin_epoch({**receipt, 'epoch_id': 'second', 'start_ns': START + 100 * SECOND,
                       'cutoff_ns': START + 200 * SECOND})
  assert capture.epoch == ordinal + 1
  assert capture.epoch_cutoff_ns - capture.epoch_start_ns == 100 * SECOND
