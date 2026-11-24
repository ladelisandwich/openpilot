"""Timing rejection and inert study recovery; numerical fixtures, no road proof."""
from dataclasses import replace
from types import SimpleNamespace

import pytest

from openpilot.starpilot.flm.fullrate_collector import FullrateCollector
from openpilot.starpilot.flm.tests.test_fullrate_pipeline import SyntheticAdapter, snapshots
from openpilot.starpilot.flm.tests.test_source_study_gaps import active_study
from openpilot.starpilot.flm.tests.test_tracking_source_context import refresh, source_ack


def subject(age=43_305_053, *, pause=True):
  owner, now = active_study()
  if pause:
    owner.interrupt('expired_response_witness')
  control, _, _ = snapshots(now)
  control = replace(control, domain='normalized_torque', state_ns=now-age, native_required=True,
    native_allowed=True, native_identity='4'*64, native_observed_ns=now-50_000_000,
    native_expires_ns=now+150_000_000)
  collector = FullrateCollector(SyntheticAdapter())
  collector.adapter.command_domain = 'normalized_torque'
  collector.context = tuple(getattr(control, name) for name in (
    'identity', 'generation', 'domain', 'stage_revision', 'response_revision', 'source_epoch',
    'controller_mode', 'policy', 'baseline_revision', 'controller_revision', 'turn_assist'))
  collector.live_tracking = SimpleNamespace(interrupt=owner.interrupt)
  return collector, owner, control, now+37_720_875


@pytest.mark.parametrize('age', (30_000_001, 43_305_053))
@pytest.mark.parametrize('pause', (False, True))
def test_normalized_source_clock_gap_rejects_row_without_erasing_numerical_study(age, pause):
  collector, owner, control, now = subject(age, pause=pause)
  original = (owner.paused_intent[0] if pause else owner.offer_state).copy()
  before, future = owner.paused_intent[1:] if pause else (owner.before, tuple(owner.future))
  assert not collector.control(control, now)
  assert collector.reason == 'stale_control'
  assert not collector.controls and not collector.ready and not collector.recent_controls
  assert collector.last_qualified_response == collector.last_apply == 0
  assert owner.offer_state['phase'] == 'rollback' and owner.settled_ns is None
  assert owner.paused_intent is not None
  assert owner.paused_intent[1:] == (before, future)
  assert not owner.before and not owner.future and not any(owner.accepted.values())
  refresh(owner, now+200_000_000)
  rollback = owner.offer(now+200_000_000)
  assert rollback is not None and rollback.phase == 'rollback'
  assert source_ack(owner, rollback, now+210_000_000)
  resumed = owner.offer_state
  assert resumed['phase'] == 'trial' and resumed['trial_id'] != original['trial_id']
  assert resumed['target_sequence'] > original['target_sequence'] and resumed['published'] is False
  for name in ('started_ns', 'deadline_ns', 'alpha', 'required_episodes', 'look_quotas', 'target', 'baseline'):
    assert resumed[name] == original[name]
  assert (owner.before, tuple(owner.future)) == (before, future) and owner.settled_ns is None


@pytest.mark.parametrize('change,reason', (
  ({'identity': 'bad'}, 'invalid_control'),
  ({'requested': float('nan')}, 'invalid_control'),
  ({'valid': False}, 'ineligible_control'),
  ({'fault': True}, 'ineligible_control'),
  ({'override': True}, 'ineligible_control'),
  ({'manual': True}, 'ineligible_control'),
  ({'active': False}, 'ineligible_control'),
  ({'saturated': True}, 'ineligible_control'),
  ({'native_allowed': False}, 'original_native_admission_unavailable'),
  ({'native_expires_ns': 1}, 'original_native_admission_unavailable'),
  ({'source_receipt_ns': 1}, 'invalid_control'),
  ({'domain': 'torque'}, 'invalid_control'),
))
def test_clock_gap_never_masks_fault_native_structural_or_other_domain_failure(change, reason):
  collector, owner, control, now = subject()
  assert owner.paused_intent is not None
  assert not collector.control(replace(control, **change), now)
  assert collector.reason == reason and owner.paused_intent is None
  assert not collector.ready and not collector.controls and not any(owner.accepted.values())


@pytest.mark.parametrize('change', ({'identity': '5'*64}, {'source_epoch': '6'*64}, {'baseline_revision': '7'*64}))
def test_clock_gap_with_changed_context_cannot_preserve_old_study(change):
  collector, owner, control, now = subject()
  assert not collector.control(replace(control, **change), now)
  assert collector.training_interruptions['actual_controller_identity_changed'] == 1
  assert owner.paused_intent is None and not collector.ready


def test_original_deadline_still_expires_after_source_clock_gap():
  collector, owner, control, now = subject()
  deadline = owner.paused_intent[0]['deadline_ns']
  assert not collector.control(control, now)
  refresh(owner, deadline+1)
  rollback = owner.offer(deadline+1)
  assert rollback is not None and source_ack(owner, rollback, deadline+10_000_001)
  assert owner.paused_intent is None and owner.offer_state['phase'] == 'rollback'
  assert owner.offer_state['accepted'] and not any(owner.accepted.values())
