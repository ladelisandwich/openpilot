"""Planning-only numerical regressions; no controller/native authority proof."""
from dataclasses import replace
import math

import pytest

from openpilot.starpilot.flm import live_tracking as tracking
from openpilot.starpilot.flm.tests.test_live_tracking_rollout import NS, HOOK, _optimizer, _episode, _refresh, _ack


def setup_plan():
  owner = _optimizer()
  rows = tuple(_episode(owner, index, (1000+index*8)*NS, owner.accepted, .6, .35, 22.) for index in range(20, 28))
  target = dict(owner.accepted)
  target[HOOK] = .005
  reference = tuple(_episode(owner, index, (1000+index*8)*NS, owner.accepted, .6, score, 22.)
    for index, score in enumerate((.31, .31, .34, .34, .37, .37, .34, .34, .31, .31, .34, .34), 1))
  return owner, rows, target, reference


def test_past_comparable_noise_can_only_increase_forecast_quota_or_require_more_baseline():
  owner, rows, target, reference = setup_plan()
  original = owner._trial_plan(rows, .025, target)
  assert original is not None
  owner.planning_history.extend(reference)
  forecast = owner._trial_plan(rows, .025, target)
  assert forecast is None or forecast[0] > original[0]
  # Planning never replaces or populates the actual independent score baseline.
  assert owner.before == () and owner.future == []


def test_larger_future_variance_cannot_admit_an_underpowered_trial_by_raising_welch_degrees_of_freedom():
  owner = _optimizer()
  rows = tuple(_episode(owner, index, (1000+index*8)*NS, owner.accepted, .6, score, 22.)
    for index, score in enumerate((.34, .35, .36), 20))
  target = dict(owner.accepted)
  target[HOOK] = .005
  reference = tuple(_episode(owner, index, (1000+index*8)*NS,
    owner.accepted, .6, score, 22.) for index, score in
    enumerate((.35-math.sqrt(.00014), .35, .35+math.sqrt(.00014)), 1))
  variance, inflation, before_df = tracking._noise(rows, owner.rmse)
  historical_variance, historical_inflation, _ = tracking._noise(reference, owner.rmse)
  cadence = rows[1].end_ns-rows[0].end_ns
  feasible = min(tracking.MAX_EPISODES,
    math.floor((tracking.TRIAL_TIMEOUT_NS-NS-owner.lag_ns)/cadence))
  left = variance*inflation/len(rows)

  def nominal_rhs(quota, future_variance, future_inflation):
    right = future_variance*future_inflation/quota
    df = tracking._welch_df(left, before_df, right, max(1, math.floor(quota/future_inflation)-1))
    return (tracking._critical(df, .025)+.8416212335729143)*math.sqrt(left+right)

  # Inspect EVERY feasible original quota, including quota6 (which an earlier
  # hand calculation missed). Construct the strict counterexample interval
  # from actual numerical helpers rather than guessing the original quota.
  original_best = min(nominal_rhs(q, variance, inflation) for q in range(3, feasible+1))
  naive_noisier = nominal_rhs(9, max(variance, historical_variance), max(inflation, historical_inflation))
  assert naive_noisier < original_best
  expected = (original_best+naive_noisier)/2
  assert naive_noisier < expected < original_best
  original = owner._trial_plan(rows, expected, target)
  assert original is not None and original[0] == 3  # Fixed minimum pilot, not a powered quota9 claim.
  owner.planning_history.extend(reference)
  # Without the original-critical envelope, quota9 becomes admissible solely
  # because its larger forecast variance raises Welch df and lowers t.
  forecast = owner._trial_plan(rows, expected, target)
  assert forecast is not None and forecast[0] == original[0] == 3
  assert forecast[1] >= original[1] and forecast[2] == original[2] == .025
  assert owner.before == () and owner.future == []
  # Nominal planning did not become easier. The bounded pilot is still not
  # acceptance: equal independently observed tracking must actually roll back.
  now = rows[-1].end_ns+8*NS
  _refresh(owner, now)
  baseline = owner.accepted.copy()
  assert owner._launch('ff.left', HOOK, target, rows, forecast, now)
  offer = owner.offer(now)
  assert offer is not None and offer.phase == 'trial'
  _ack(owner, offer, now+NS)
  assert owner.offer_state is not None
  maximum = max(owner.offer_state['look_quotas'])
  for index in range(maximum):
    now += 8*NS
    _refresh(owner, now)
    future = _episode(owner, 1000+index, now, dict(offer.target), .6, .35, 22.)
    assert owner.observe_episode(future, now_ns=now)
  rollback = owner.offer(now)
  assert rollback is not None and rollback.phase == 'rollback' and rollback.target == rollback.baseline
  assert owner.reason == 'independent_future_tracking_not_improved' and owner.accepted == baseline


@pytest.mark.parametrize('mismatch', ('binding', 'profile', 'shape', 'age'))
def test_forecast_never_borrows_different_binding_profile_shape_or_stale_history(mismatch):
  owner, rows, target, reference = setup_plan()
  original = owner._trial_plan(rows, .025, target)
  if mismatch == 'binding':
    reference = tuple(replace(row, binding=replace(row.binding, reset_generation='other-drive')) for row in reference)
  elif mismatch == 'profile':
    reference = tuple(replace(row, profile_sha256='0'*64) for row in reference)
  elif mismatch == 'shape':
    reference = tuple(replace(row, goal_shape=(2.,)*8) for row in reference)
  else:
    reference = tuple(replace(row, start_ns=row.start_ns-900*NS, end_ns=row.end_ns-900*NS) for row in reference)
  owner.planning_history.extend(reference)
  assert owner._trial_plan(rows, .025, target) == original


def test_history_keeps_zero_error_clean_geometry_but_not_trial_future_outcomes():
  owner = _optimizer()
  zero = _episode(owner, 0, 8*NS, owner.accepted, .6, 0., 22.)
  _refresh(owner, zero.end_ns)
  owner.observe_episode(zero, now_ns=zero.end_ns)
  assert list(owner.planning_history) == [zero]
  owner.interrupt('new-physical-context', context_changed=True)
  assert not owner.planning_history
  for index in range(1, 49):
    now = (8+index*8)*NS
    _refresh(owner, now)
    row = _episode(owner, index, now, owner.accepted, .6, .35, 22.)
    owner.observe_episode(row, now_ns=now)
    if owner.offer_state is not None:
      break
  offer = owner.offer(now)
  assert offer is not None and offer.phase == 'trial'
  _ack(owner, offer, now+NS)
  history = tuple(owner.planning_history)
  now += 8*NS
  _refresh(owner, now)
  future = _episode(owner, 100, now, dict(offer.target), .6, .325, 22.)
  owner.observe_episode(future, now_ns=now)
  assert tuple(owner.planning_history) == history
  assert all(row.profile_sha256 == owner._hash(owner.accepted) for row in history)
