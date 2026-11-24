"""Informational recovery only; synthetic numerical/protocol evidence."""
from dataclasses import replace

import pytest

from openpilot.starpilot.flm.live_trial_owner import TrackingTrialOwner
from openpilot.starpilot.flm.live_trial_contract import MAX_STEP
from openpilot.starpilot.flm.tests.test_live_tracking import optimizer as _protocol_optimizer
from openpilot.starpilot.flm.tests.test_live_tracking_rollout import NS, HOOK, _episode, _refresh, _ack, _optimizer


def interrupted_owner():
  # Existing clean protocol certificate isolates lifecycle assertions.
  # Original noisy rollout certificates, seeds and floors are unchanged.
  owner = _protocol_optimizer()
  for index in range(1, 61):
    now = index*8*NS
    _refresh(owner, now)
    owner.observe_episode(_episode(owner, index, now, owner.accepted, .6, .35, 22.), now_ns=now)
    if owner.offer_state is not None:
      break
  offer = owner.offer(now)
  assert offer is not None and offer.phase == 'trial'
  _ack(owner, offer, now+NS)
  before = tuple(owner.before)
  _refresh(owner, now+8*NS)
  owner.observe_episode(_episode(owner, 90, now+8*NS, dict(offer.target), .6, .325, 22.), now_ns=now+8*NS)
  assert len(owner.future) == 1
  future = tuple(owner.future)
  assert owner.selector_hint is not None
  hint = owner.selector_hint.copy()
  owner.interrupt('temporary_current_source_or_native_unavailable')
  assert owner.before == () and owner.future == [] and owner.comparison == []
  _refresh(owner, now+10*NS)
  rollback = owner.offer(now+10*NS)
  assert rollback is not None and rollback.phase == 'rollback' and rollback.target == rollback.baseline
  # Even a fresh numerical rollback lease cannot bypass current permission.
  actual_owner = TrackingTrialOwner(owner.binding)
  assert not actual_owner.prepare(rollback, now_ns=now+10*NS, permission=False, binding=owner.binding)
  _ack(owner, rollback, now+11*NS)
  return owner, offer, hint, before, future, now+16*NS


def test_recovery_uses_only_information_and_new_probe_revalidation_then_new_uncensored_rows():
  owner, old, hint, before, future, now = interrupted_owner()
  assert set(hint) == {'group', 'name', 'delta', 'demand', 'binding', 'profile_sha256', 'model_revision',
    'controller_domain', 'selected_ns', 'expires_ns', 'source_episode_ids', 'quota', 'minimum_episodes'}
  assert hint['minimum_episodes'] == len(before)
  assert owner.preferred_probe_group(now) == hint['group']
  _refresh(owner, now)
  revalidation = _episode(owner, 100, now, owner.accepted, .6, .35, 22.)
  revalidation = replace(revalidation, sensitivity=(replace(revalidation.sensitivity[0], error_feature_sum=-999.),))
  owner.observe_episode(revalidation, now_ns=now)
  assert owner.discovery is not None and not owner.comparison
  assert owner.discovery['expires_ns'] == hint['expires_ns']
  quota = owner.discovery['quota']
  compared = []
  for index in range(quota):
    now += 8*NS
    _refresh(owner, now)
    row = _episode(owner, 101+index, now, owner.accepted, .6, .35, 22.)
    row = replace(row, sensitivity=(replace(row.sensitivity[0], error_feature_sum=-999.),))
    compared.append(row)
    owner.observe_episode(row, now_ns=now)
  new = owner.offer(now)
  assert new is not None and new.phase == 'trial' and new.trial_id != old.trial_id
  assert new.target_sequence == old.target_sequence+1 and new.started_ns == now
  assert new.deadline_ns == now+120*NS and new.deadline_ns != old.deadline_ns
  assert dict(new.target)[HOOK] > dict(new.baseline)[HOOK]  # No direction selection from the new negative correlations.
  assert owner.before == tuple(compared) and all(row.error_rms == .35*row.goal_rms for row in owner.before)
  spent = {row.episode_id for row in before+future+(revalidation,)}
  assert spent.isdisjoint(row.episode_id for row in owner.before)
  assert owner.selector_hint is not None and owner.offer_state is not None
  assert owner.selector_hint['expires_ns'] == hint['expires_ns']
  assert owner.offer_state['alpha'] == .05/(2*3)  # No aborted-attempt alpha refund.
  assert owner.preferred_probe_group(now) is None  # Round-robin resumes during the actual trial.


def test_zero_error_recovery_rows_are_uncensored_but_do_not_force_an_underpowered_trial():
  owner, old, hint, _before, _future, now = interrupted_owner()
  _refresh(owner, now)
  revalidation = _episode(owner, 100, now, owner.accepted, .6, .35, 22.)
  owner.observe_episode(revalidation, now_ns=now)
  assert owner.discovery is not None and owner.comparison == []
  quota = owner.discovery['quota']
  compared = []
  for index in range(quota):
    now += 8*NS
    _refresh(owner, now)
    row = _episode(owner, 101+index, now, owner.accepted, .6, 0., 22.)
    compared.append(row)
    owner.observe_episode(row, now_ns=now)
    if index+1 < quota:
      assert owner.comparison == compared  # Every zero row counts; none is replaced.
  assert all(row.episode_id in owner.seen for row in compared)
  assert all(row in owner.planning_history for row in compared)
  assert owner.discovery is None and owner.comparison == []
  # A declared minimum pilot is now allowed; zero baseline error supplies no
  # evidence of improvement. Exercise the unchanged independent final refusal.
  offer = owner.offer(now)
  assert offer is not None and offer.phase == 'trial' and offer.trial_id != old.trial_id
  assert owner.sequence == old.target_sequence+1 and owner.before == tuple(compared)
  assert owner.selector_hint is not None
  assert all(row.error_rms == 0. for row in owner.before)
  _ack(owner, offer, now+NS)
  assert owner.offer_state is not None
  maximum = max(owner.offer_state['look_quotas'])
  for index in range(maximum):
    now += 8*NS
    _refresh(owner, now)
    row = _episode(owner, 1000+index, now, dict(offer.target), .6, 0., 22.)
    assert owner.observe_episode(row, now_ns=now)
  rollback = owner.offer(now)
  assert rollback is not None and rollback.phase == 'rollback' and rollback.trial_id == offer.trial_id
  assert rollback.target == rollback.baseline
  assert owner.reason == 'independent_future_tracking_not_improved'
  assert owner.accepted == dict(old.baseline) and owner.selector_hint is None
  assert len(owner.future) == maximum



@pytest.mark.parametrize('bad', ('missing', 'invisible', 'energy', 'physical'))
def test_retained_information_cannot_replace_fresh_native_visibility(bad):
  owner, old, _hint, _before, _future, now = interrupted_owner()
  _refresh(owner, now)
  row = _episode(owner, 100, now, owner.accepted, .6, .35, 22.)
  if bad == 'missing':
    row = replace(row, sensitivity=())
  else:
    feature = row.sensitivity[0]
    if bad == 'invisible':
      feature = replace(feature, native_separated=0)
    elif bad == 'energy':
      feature = replace(feature, feature_energy=0.)
    else:
      feature = replace(feature, max_step_physical_rms=0.)
    row = replace(row, sensitivity=(feature,))
  owner.observe_episode(row, now_ns=now)
  assert owner.discovery is None and owner.comparison == []
  assert owner.offer_state is not None
  assert owner.offer_state['trial_id'] == old.trial_id and owner.offer_state['phase'] == 'rollback'
  assert owner.reason == 'fresh_selected_native_probe_required'


def test_recovery_requires_current_response_and_current_source_binding():
  owner, old, _hint, _before, _future, now = interrupted_owner()
  row = _episode(owner, 100, now, owner.accepted, .6, .35, 22.)
  owner.observe_episode(row, now_ns=now)  # Previous pose is stale; no refresh.
  assert owner.discovery is None and owner.reason == 'fresh_selector_response_required'
  _refresh(owner, now+8*NS)
  wrong = _episode(owner, 101, now+8*NS, owner.accepted, .6, .35, 22.)
  wrong = replace(wrong, binding=replace(wrong.binding, source_epoch='9'*64))
  assert not owner.observe_episode(wrong, now_ns=now+8*NS)
  assert owner.offer_state is not None
  assert owner.discovery is None and owner.offer_state['trial_id'] == old.trial_id


@pytest.mark.parametrize('change', ('binding', 'profile', 'model', 'domain', 'context', 'expiry'))
def test_informational_selector_invalidates_without_stale_group_priority(change):
  owner, _old, hint, _before, _future, now = interrupted_owner()
  if change == 'binding':
    owner.binding = replace(owner.binding, source_epoch='9'*64)
  elif change == 'profile':
    owner.accepted[HOOK] = .001
  elif change == 'model':
    owner.certificate = replace(owner.certificate, model_revision='9'*64)
  elif change == 'domain':
    owner.domain = 'torque_pid'
  elif change == 'context':
    owner.interrupt('changed-physical-context', context_changed=True)
  else:
    now = hint['expires_ns']+1
  assert owner.preferred_probe_group(now) is None
  assert owner.selector_hint is None and not owner.selector_pending and owner.discovery is None


def test_numerical_failed_future_cannot_become_a_recovery_hint():
  owner, _old, _hint, _before, _future, _now = interrupted_owner()
  owner._rollback('independent_future_tracking_not_improved')
  assert owner.selector_hint is None and not owner.selector_pending


def test_joint_allocation_uses_more_fresh_baseline_to_shorten_active_exposure():
  owner = _optimizer()  # Original noisy .005 model floor, unchanged.
  history = tuple(_episode(owner, index, index*8*NS, owner.accepted, .6, .35, 22.)
    for index in range(1, 10))
  owner.planning_history.extend(history)
  target = dict(owner.accepted)
  target[HOOK] = .005
  allocation = owner._comparison_allocation(history[-1], .023, target)
  assert allocation is not None
  before_count, future_count = allocation
  assert 3 < before_count <= 24 and future_count == 3
  # With the SAME whole-episode noise floor, a three-row baseline needs more
  # target exposure. The independently collected larger baseline needs three.
  short = owner._trial_plan(history[:3], .023, target)
  assert short is not None and short[0] > future_count
  fresh = tuple(_episode(owner, 100+index, (10+index)*8*NS, owner.accepted, .6, .35, 22.)
    for index in range(before_count))
  actual = owner._trial_plan(fresh, .023, target)
  assert actual is not None and actual[0] == future_count
  assert owner.sequence == 0 and owner.before == () and owner.future == []


def test_cold_selection_predeclares_more_clean_baseline_for_shorter_target_exposure():
  owner = _optimizer()  # Same .005 whole-episode model floor as the noisy route.
  # This is the neighboring allocation test's .023 normalized-effect boundary,
  # not a fixed-derivative plant fixture. Keep the current fitted proposal size.
  expected_normalized_effect, goal, fitted_estimate = .023, .6, .06
  planned_delta = min(MAX_STEP, .1 * fitted_estimate)
  physical_rms_at_cap = expected_normalized_effect * goal * MAX_STEP / planned_delta
  selected = []
  for index in range(1, 4):
    now = index*8*NS
    _refresh(owner, now)
    row = _episode(owner, index, now, owner.accepted, .6, .35, 22.)
    row = replace(row, sensitivity=(replace(row.sensitivity[0], max_step_physical_rms=physical_rms_at_cap),))
    selected.append(row)
    owner.observe_episode(row, now_ns=now)
  assert owner.offer_state is None and owner.discovery is not None
  assert owner.discovery['expected'] == pytest.approx(expected_normalized_effect)
  assert owner.discovery['target'][HOOK] - owner.accepted[HOOK] == pytest.approx(planned_delta)
  quota = owner.discovery['quota']
  assert 3 < quota <= 24 and owner.comparison == []
  assert not owner.discovery['recovery']
  compared = []
  for index in range(quota):
    now += 8*NS
    _refresh(owner, now)
    row = _episode(owner, 100+index, now, owner.accepted, .6, .35, 22.)
    row = replace(row, sensitivity=(replace(row.sensitivity[0], error_feature_sum=-999.),))
    compared.append(row)
    owner.observe_episode(row, now_ns=now)
    if index+1 < quota:
      assert owner.offer_state is None and owner.discovery is not None
      assert owner.discovery['quota'] == quota and owner.comparison == compared
  offer = owner.offer(now)
  assert offer is not None and offer.phase == 'trial'
  assert owner.offer_state is not None and owner.offer_state['required_episodes'] == 3
  assert owner.before == tuple(compared) and dict(offer.target)[HOOK] > dict(offer.baseline)[HOOK]
  assert {row.episode_id for row in selected}.isdisjoint(row.episode_id for row in owner.before)
  assert owner.sequence == 1 and owner.offer_state['alpha'] == .025


def _planned_unissued(owner, now):
  _refresh(owner,now)
  rows=tuple(_episode(owner,100+i,now-(2-i)*8*NS,owner.accepted,.6,.35,22.)for i in range(3))
  target=dict(owner.accepted)
  target[HOOK]+=.005
  assert owner._launch('ff.left',HOOK,target,rows,(3,2.0,.025),now)
  preview=owner.offer(now,publication=False)
  assert preview is not None and owner.offer_state['published'] is False
  return preview


def test_unissued_proposal_withdrawal_has_no_phantom_consumer_rollback_or_budget_refund():
  owner=_protocol_optimizer()
  preview=_planned_unissued(owner,30*NS)
  owner.interrupt('fresh_response_witness_unavailable')
  assert owner.offer_state is None and owner.before==() and owner.future==[]
  assert owner.sequence==preview.target_sequence==1 and owner.last_attempt==30*NS
  _refresh(owner,31*NS)
  assert owner.offer(31*NS,publication=False) is None


def test_before_send_publication_marker_preserves_real_rollback_and_original_deadline():
  owner=_protocol_optimizer()
  preview=_planned_unissued(owner,30*NS)
  owner.mark_published(preview)
  owner.interrupt('fresh_response_witness_unavailable')
  _refresh(owner,31*NS)
  rollback=owner.offer(31*NS,publication=False)
  assert rollback.phase=='rollback' and rollback.trial_id==preview.trial_id
  assert rollback.target_sequence==preview.target_sequence and rollback.target==preview.baseline
  assert rollback.started_ns==preview.started_ns and rollback.deadline_ns==preview.deadline_ns


def test_abandoned_unissued_attempt_preserves_prior_accepted_sequence_and_target():
  owner=_protocol_optimizer()
  prior=_planned_unissued(owner,30*NS)
  # Synthetic preexisting accepted fixture; this is a sequence/lifecycle unit,
  # never evidence that this test fitted or physically accepted a correction.
  state=owner.offer_state.copy()
  state.update(phase='confirmed',accepted=True,published=True)
  state.pop('prior_accepted',None)
  owner.offer_state=state
  owner.accepted=dict(prior.target)
  proposal=_planned_unissued(owner,60*NS)
  assert proposal.target_sequence==2
  owner.interrupt('fresh_response_witness_unavailable')
  _refresh(owner,61*NS)
  retained=owner.offer(61*NS,publication=False)
  assert retained.trial_id==prior.trial_id and retained.target==prior.target
  assert retained.target_sequence==1 and owner.sequence==2
  assert owner.acknowledge_application(trial_id=retained.trial_id,target_sequence=1,phase='confirmed',
    target_sha256=retained.target_sha256,applied_full30=retained.target,control_ns=61*NS,
    binding=retained.binding,now_ns=61*NS,status='applied',confirmed_target_sha256=retained.target_sha256)
  assert owner.report(61*NS)['applicationAcknowledged'] is True


def test_unissued_proposal_timeout_does_not_start_a_later_deadline():
  owner=_protocol_optimizer()
  proposal=_planned_unissued(owner,30*NS)
  _refresh(owner,proposal.deadline_ns+1)
  assert owner.offer(proposal.deadline_ns+1,publication=False) is None
  assert owner.offer_state is None and owner.sequence==1
