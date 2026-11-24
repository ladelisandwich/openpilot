"""Numerical state-machine fixtures, NOT actual pose/model/native admission.

Real controller probes, route joins, owner application and bootstrap remain
independent qualification. Synthetic certificate contents test the protocol.
"""
from typing import Any


from dataclasses import replace
import hashlib
import pytest

from openpilot.starpilot.flm.canonical_surface_adapter import NAMES
from openpilot.starpilot.flm.live_trial_contract import LiveTrialBinding, MAX_STEP
from openpilot.starpilot.flm.live_tracking import LiveTrackingOptimizer, TrackingEpisode, HookSensitivity
from openpilot.starpilot.flm.live_tracking_response import ResponseCertificate, ResponseWitness, VERSION, _revision, validate_certificate


def fixture_certificate():
  binding = LiveTrialBinding(
    'a' * 64, 'b' * 64, 'actual-controller-registry-v2:' + 'c' * 64, 'd' * 64, 'e' * 64, 'reset-1', 'f' * 64, 'a' * 64, '', '0' * 64, 'b' * 64, False
  )
  raw = ResponseCertificate(
    VERSION,
    binding,
    '0' * 64,
    170_000_000,
    3.0,
    0.0,
    -0.3,
    0.3,
    0.001,
    ('train0', 'train1', 'train2', 'train3'),
    ('hold0', 'hold1', 'hold2', 'hold3'),
    'c' * 64,
    'd' * 64,
    1,
  )
  revision = _revision(raw)
  return validate_certificate(replace(raw, model_revision=revision, binding=replace(binding, physical_model_revision=revision)))


def optimizer(domain='torque'):
  cert = fixture_certificate()
  return LiveTrackingOptimizer(cert.binding, tuple((name, 0.0) for name in NAMES), controller_domain=domain, response_certificate=cert)


def refresh(owner, now):
  raw = ResponseWitness(owner.certificate, owner.binding, now - 180_000_000, now - 180_000_000, now, now - 350_000_000, 8, 0.001, now + 100_000_000)
  assert owner.response_current(raw, now_ns=now)


# Preserve the fixture derivative (0.01 physical delta per 0.005 coordinate).
FIXTURE_PHYSICAL_DERIVATIVE = 2.0


def episode(owner, index, name, *, kind='large', profile=None, error=0.2, reversal=0.1, native=4, effect=FIXTURE_PHYSICAL_DERIVATIVE * MAX_STEP, start=None):
  end = (index + 1) * 1_000_000_000 if start is None else start + 500_000_000
  begin = end - 500_000_000
  direction = -1 if kind == 'large' and name.endswith('_right') else 1
  return TrackingEpisode(
    owner.binding,
    profile or owner._hash(owner.accepted),
    hashlib.sha256(str(index).encode()).hexdigest(),
    begin,
    end,
    kind,
    direction,
    20.0,
    21.0,
    0.5 if kind == 'large' else 0.1,
    error,
    reversal,
    20,
    10_000_000,
    (HookSensitivity(name, 0.05, 1.0, native, 20, effect, native_gradient_complete=True),),
    (float(direction),) * 8,
    native_gradient_complete=True,
  )


def ready(owner, name='ff_gain_left', *, native=4, effect=FIXTURE_PHYSICAL_DERIVATIVE * MAX_STEP):
  kind = 'subtle' if name.startswith('center_') or name == 'highway_center_taper_max' else 'large'
  count = 9 if kind == 'subtle' else 3
  discovery_ids = set()
  comparison_ids = set()
  for index in range(2 * count):
    row = episode(owner, index, name, kind=kind, native=native, effect=effect)
    refresh(owner, row.end_ns)
    owner.observe_episode(row, now_ns=row.end_ns)
    (discovery_ids if index < count else comparison_ids).add(row.episode_id)
    if index == count - 1:
      # Geometry/probes select a candidate, never an immediate live offer.
      assert owner.offer_state is None
  if owner.offer_state is not None:
    assert {row.episode_id for row in owner.before} == comparison_ids
    assert discovery_ids.isdisjoint(row.episode_id for row in owner.before)
  return owner.offer_state, row.end_ns


def acknowledge(owner, offer, *, phase=None, stamp=None):
  stamp = stamp or offer.observed_ns + 10_000_000
  refresh(owner, stamp)
  return owner.acknowledge_application(
    trial_id=offer.trial_id,
    target_sequence=offer.target_sequence,
    phase=phase or offer.phase,
    target_sha256=offer.target_sha256,
    applied_full30=offer.target,
    control_ns=stamp,
    binding=offer.binding,
    now_ns=stamp,
    status='applied',
    confirmed_target_sha256=offer.target_sha256 if (phase or offer.phase) == 'confirmed' else '',
  )


@pytest.mark.parametrize('name', NAMES)
def test_all_thirty_torque_hooks_are_available_without_model_specific_tune(name):
  owner = optimizer()
  state, now = ready(owner, name)
  assert state is not None and state['names'] == (name,)
  offer = owner.offer(now)
  assert offer is not None and offer.phase == 'trial'
  assert len(offer.baseline) == len(offer.target) == 30
  changes = [(key, value - dict(offer.baseline)[key]) for key, value in offer.target if value != dict(offer.baseline)[key]]
  assert len(changes) == 1 and changes[0][0] == name and 0 < changes[0][1] <= MAX_STEP
  assert changes[0][1] == pytest.approx(0.005, rel=0, abs=1e-18)


@pytest.mark.parametrize(
  'name', ('turn_in_threshold_reduction_left', 'turn_in_threshold_reduction_right', 'unwind_threshold_increase_left', 'unwind_threshold_increase_right')
)
def test_pid_does_not_invent_four_unavailable_friction_hooks(name):
  owner = optimizer('torque_pid')
  state, _ = ready(owner, name)
  assert state is None


@pytest.mark.parametrize('native,effect', [(0, 0.01), (1, 0.01), (4, 0.0)])
def test_invisible_native_or_noise_trial_is_not_offered(native, effect):
  owner = optimizer()
  state, now = ready(owner, native=native, effect=effect)
  assert state is None and owner.offer(now) is None


def test_future_independent_improvement_confirms_only_after_actual_target_ack():
  owner = optimizer()
  _, now = ready(owner)
  offer = owner.offer(now)
  assert offer is not None
  assert owner.accepted['ff_gain_left'] == 0.0
  # A future point without real application acknowledgement cannot count.
  row = episode(owner, 10, 'ff_gain_left', profile=offer.target_sha256, error=0.001, start=now + 300_000_000)
  refresh(owner, row.end_ns)
  assert not owner.observe_episode(row, now_ns=row.end_ns)
  assert not owner.future
  assert acknowledge(owner, offer, stamp=now + 20_000_000)
  for index in range(3):
    row = episode(owner, 20 + index, 'ff_gain_left', profile=offer.target_sha256, error=0.001, start=now + 1_000_000_000 + index * 1_000_000_000)
    refresh(owner, row.end_ns)
    assert owner.observe_episode(row, now_ns=row.end_ns)
  confirmed = owner.offer(row.end_ns)
  assert confirmed is not None and confirmed.phase == 'confirmed'
  assert owner.accepted['ff_gain_left'] == 0.0  # Future decision is not owner application/persistence.
  assert acknowledge(owner, confirmed)
  assert owner.accepted['ff_gain_left'] == dict(offer.target)['ff_gain_left']
  # Confirmed lease renewal preserves immutable trial baseline/target identity.
  renewed = owner.offer(row.end_ns + 20_000_000)
  assert renewed is not None and renewed.baseline == offer.baseline and renewed.target == offer.target
  assert renewed.target_sequence == offer.target_sequence and renewed.sequence > offer.sequence


def test_worse_oscillation_rolls_back_without_persisting_trial():
  owner = optimizer()
  _, now = ready(owner)
  offer = owner.offer(now)
  assert acknowledge(owner, offer)
  row = episode(owner, 30, 'ff_gain_left', profile=offer.target_sha256, reversal=2.0, start=now + 1_000_000_000)
  refresh(owner, row.end_ns)
  assert not owner.observe_episode(row, now_ns=row.end_ns)
  rollback = owner.offer(row.end_ns)
  assert rollback is not None and rollback.phase == 'rollback' and rollback.target == rollback.baseline
  assert owner.accepted['ff_gain_left'] == 0.0


def test_original_deadline_and_quota_cannot_be_renewed():
  owner = optimizer()
  _, now = ready(owner)
  first = owner.offer(now)
  refresh(owner, now + 50_000_000)
  second = owner.offer(now + 50_000_000)
  assert second.deadline_ns == first.deadline_ns == first.started_ns + 120_000_000_000
  assert second.target_sequence == first.target_sequence and second.sequence > first.sequence
  assert owner.offer_state['required_episodes'] == 3
  refresh(owner, first.deadline_ns + 1)
  expired = owner.offer(first.deadline_ns + 1)
  assert expired is not None and expired.phase == 'rollback'


def test_context_or_pose_freshness_withdraws_trial_instead_of_renewing_old_pose():
  owner = optimizer()
  _, now = ready(owner)
  assert owner.offer(now) is not None
  assert owner.offer(now + 100_000_001) is None
  assert owner.offer_state['phase'] == 'rollback'
  assert owner.accepted['ff_gain_left'] == 0.0


def test_duplicate_and_custom_episode_identity_cannot_supply_independence():
  owner = optimizer()
  row = episode(owner, 0, 'ff_gain_left')
  refresh(owner, row.end_ns)
  owner.observe_episode(row, now_ns=row.end_ns)
  assert not owner.observe_episode(row, now_ns=row.end_ns)
  assert len(owner.baseline) == 1
  with pytest.raises(ValueError):
    replace(row, episode_id=object()).validate()


@pytest.mark.parametrize('future_error,expected_worse', ((0.21, True), (0.2, False), (0.19, False)))
def test_independent_closed_loop_loss_changes_next_direction_without_reusing_scores(future_error, expected_worse):
  owner = optimizer()
  _, now = ready(owner)
  offer = owner.offer(now)
  assert offer is not None and acknowledge(owner, offer, stamp=now + 20_000_000)
  original_before = {row.episode_id for row in owner.before}
  future_ids = set()
  for index in range(3):
    row = episode(owner, 200 + index, 'ff_gain_left', profile=offer.target_sha256, error=future_error, start=now + 1_000_000_000 + index * 1_000_000_000)
    future_ids.add(row.episode_id)
    refresh(owner, row.end_ns)
    owner.observe_episode(row, now_ns=row.end_ns)
  assert bool(owner.closed_loop_worse) is expected_worse
  terminal = owner.offer(row.end_ns)
  assert terminal is not None
  if future_error < 0.2:
    assert terminal.phase == 'confirmed'
    return
  assert terminal.phase == 'rollback' and owner.accepted['ff_gain_left'] == 0.0
  assert acknowledge(owner, terminal, stamp=row.end_ns + 10_000_000)
  for index in range(6):
    row = episode(owner, 300 + index, 'ff_gain_left', start=now + 40_000_000_000 + index * 1_000_000_000)
    refresh(owner, row.end_ns)
    owner.observe_episode(row, now_ns=row.end_ns)
  next_offer = owner.offer(row.end_ns)
  assert next_offer is not None and next_offer.phase == 'trial'
  delta = dict(next_offer.target)['ff_gain_left']
  assert (-MAX_STEP <= delta < 0) if expected_worse else (0 < delta <= MAX_STEP)
  assert abs(delta) == pytest.approx(0.005, rel=0, abs=1e-18)
  assert next_offer.target_sequence == offer.target_sequence + 1
  assert owner.offer_state['alpha'] == 0.05 / (2 * 3)
  assert {row.episode_id for row in owner.before}.isdisjoint(original_before | future_ids)


@pytest.mark.parametrize('outcome', ('improve', 'null', 'worse', 'deadline'))
def test_small_error_followup_looks_are_predeclared_and_split_original_budget(outcome):
  owner = optimizer()
  state, now = ready(owner, 'center_taper_max')
  assert state is not None and state['look_quotas'] == (9, 18)
  deadline, alpha = state['deadline_ns'], state['alpha']
  assert alpha == 0.05 / (1 * 2)
  assert sum(alpha / len(state['look_quotas']) for _ in state['look_quotas']) == alpha
  offer = owner.offer(now)
  assert offer is not None and acknowledge(owner, offer, stamp=now + 20_000_000)
  first_error = 0.202 if outcome == 'worse' else 0.2 if outcome == 'null' else 0.1995
  for index in range(9):
    row = episode(
      owner, 500 + index, 'center_taper_max', kind='subtle', profile=offer.target_sha256, error=first_error, start=now + 1_000_000_000 + index * 1_000_000_000
    )
    refresh(owner, row.end_ns)
    owner.observe_episode(row, now_ns=row.end_ns)
  if outcome == 'worse':
    assert owner.offer_state['phase'] == 'rollback'
    return
  assert owner.offer_state['phase'] == 'trial' and len(owner.future) == 9
  assert owner.offer_state['required_episodes'] == 18
  assert owner.offer_state['look_quotas'] == (9, 18)
  assert owner.offer_state['deadline_ns'] == deadline and owner.offer_state['alpha'] == alpha
  assert owner.accepted['center_taper_max'] == 0.0
  if outcome == 'deadline':
    refresh(owner, deadline + 1)
    terminal = owner.offer(deadline + 1)
    assert terminal is not None and terminal.phase == 'rollback' and terminal.deadline_ns == deadline
    return
  for index in range(9, 18):
    row = episode(
      owner,
      500 + index,
      'center_taper_max',
      kind='subtle',
      profile=offer.target_sha256,
      error=0.198 if outcome == 'improve' else 0.2,
      start=now + 1_000_000_000 + index * 1_000_000_000,
    )
    refresh(owner, row.end_ns)
    owner.observe_episode(row, now_ns=row.end_ns)
  assert owner.offer_state['phase'] == ('confirmed' if outcome == 'improve' else 'rollback')
  assert owner.offer_state['deadline_ns'] == deadline and owner.offer_state['alpha'] == alpha
  assert owner.accepted['center_taper_max'] == 0.0  # Confirmation still needs actual owner ACK.


@pytest.mark.parametrize('future_rates,expected', [((1.4, 0.46, 0.46), 'confirmed'), ((1.4, 1.4, 1.4), 'rollback')])
def test_oscillation_uses_baseline_envelope_then_independent_batch_limit(future_rates, expected):
  owner = optimizer()
  _, now = ready(owner)
  # Same average/range relationship demonstrated by original native-cycle
  # episodes: a single future turn is inside existing baseline variation.
  owner.before = tuple(replace(row, reversal_hz=rate) for row, rate in zip(owner.before, (0.46, 0.46, 2.32), strict=True))
  offer = owner.offer(now)
  assert acknowledge(owner, offer)
  for index, rate in enumerate(future_rates):
    row = episode(owner, 40 + index, 'ff_gain_left', profile=offer.target_sha256, error=0.001, reversal=rate, start=now + 1_000_000_000 + index * 1_000_000_000)
    refresh(owner, row.end_ns)
    owner.observe_episode(row, now_ns=row.end_ns)
    if index == 0:
      assert owner.offer_state['phase'] == 'trial'
  assert owner.offer_state['phase'] == expected
  if expected == 'rollback':
    assert owner.reason == 'trial_oscillation_batch_worse'
  assert owner.accepted['ff_gain_left'] == 0.0


@pytest.mark.parametrize('discovery,expected', [(None, False), ({'recovery': False}, False), ({'recovery': True}, True)])
def test_recovery_comparison_keeps_its_required_fresh_native_probes(discovery, expected):
  from openpilot.starpilot.flm.live_tracking import discovery_needs_probes

  assert discovery_needs_probes(discovery) is expected


@pytest.mark.parametrize('recovery', (False, True))
def test_actual_probe_coordinator_only_suspends_work_for_measurement_only_discovery(recovery):
  from types import SimpleNamespace
  from openpilot.starpilot.flm.live_tracking_actor import LiveTrackingActor

  calls = []
  probe = SimpleNamespace(child=None, poll_probe=lambda **kwargs: None, cancel=lambda reason: calls.append(('cancel_probe', reason)))
  actor: Any = LiveTrackingActor(probe_process=probe)
  owner = optimizer()
  owner.discovery = {'recovery': recovery}
  actor.ingress = SimpleNamespace(optimizer=owner)
  actor.certificate = owner.certificate
  actor.probe_preparation = SimpleNamespace(
    pid=None, poll=lambda **kwargs: calls.append(('poll_preparation', kwargs)) or None, cancel=lambda reason: calls.append(('cancel_preparation', reason))
  )
  actor._probe_tick(SimpleNamespace(live_response_pair=None), None, now_ns=1)
  assert any(name == 'poll_preparation' for name, _ in calls) is recovery
  assert any(name == 'cancel_probe' for name, _ in calls) is not recovery


@pytest.mark.parametrize('recovery', (False, True))
def test_actual_ingress_retains_required_native_endpoint_work_during_recovery(recovery):
  from types import SimpleNamespace
  from openpilot.starpilot.flm.live_tracking_ingress import LiveTrackingIngress

  # Scheduling seam only: no source/native/controller admission is asserted.
  obj: Any = object.__new__(LiveTrackingIngress)
  obj.last_pose_ns = 0
  obj.binding = fixture_certificate().binding
  obj.replay = None
  obj.scoring_enabled = True
  obj.optimizer = SimpleNamespace(discovery={'recovery': recovery}, offer_state=None, response_current=lambda *a, **kw: True, offer=lambda *a, **kw: None)
  obj.monitor = SimpleNamespace(observe=lambda *a, **kw: True, witness=lambda *a: object())
  released = []
  obj.episodes = SimpleNamespace(
    defer_unprobed=False, release_measured_only=lambda: released.append(True) or True, observe_pose=lambda *a, **kw: None, drain=lambda **kw: ()
  )
  obj.observe_pose(SimpleNamespace(mono_ns=1), None, None, now_ns=1, current_native=False)
  assert obj.episodes.defer_unprobed is recovery
  assert bool(released) is not recovery


def test_fast_confidence_preserves_original_student_bounds():
  from openpilot.starpilot.flm.live_tracking import _critical, _student_survival

  def original(df, alpha):
    low, high = 0.0, 1.0
    while _student_survival(high, df) > alpha and high < 1e8:
      high *= 2.0
    if _student_survival(high, df) > alpha:
      raise ValueError('Student confidence outside numerical range')
    for _ in range(64):
      middle = (low + high) / 2.0
      if _student_survival(middle, df) > alpha:
        low = middle
      else:
        high = middle
    return high

  for sequence in (0, 1, 2, 5, 20, 100):
    for looks in (1, 2):
      alpha = 0.05 / ((sequence + 1) * (sequence + 2)) / looks
      for df in range(1, 48):
        expected, actual = original(df, alpha), _critical(df, alpha)
        assert actual >= expected
        assert actual - expected <= 5e-11 * max(1.0, expected)
        assert _student_survival(actual, df) <= alpha


def test_shared_planning_grid_preserves_original_allocation_loops_and_is_bounded():
  import math
  from openpilot.starpilot.flm.live_tracking import _comparison_thresholds, _critical, _welch_df, _look_quotas

  _comparison_thresholds.cache_clear()
  for variance, inflation, feasible, minimum, alpha in ((0.001, 1.0, 19, 9, 0.025), (0.02, 1.4, 15, 3, 0.008333333333333333)):
    grid = _comparison_thresholds(variance, inflation, feasible, minimum, alpha)
    for expected in (0.0002, 0.002, 0.01, 0.03, 0.1, 0.3, 1.0):
      old, new = [], []
      for before in range(minimum, 25):
        left = variance * inflation / before
        left_df = max(1, math.floor(before / inflation) - 1)
        for future in range(minimum, feasible + 1):
          right = variance * inflation / future
          right_df = max(1, math.floor(future / inflation) - 1)
          critical = _critical(_welch_df(left, left_df, right, right_df), alpha / len(_look_quotas(future, feasible)))
          if expected > (critical + 0.8416212335729143) * math.sqrt(left + right):
            old.append((future, before + future, before))
            break
      for before, futures in grid:
        for future, threshold in futures:
          if expected > threshold:
            new.append((future, before + future, before))
            break
      assert old == new
    assert _comparison_thresholds(variance, inflation, feasible, minimum, alpha) is grid
  info = _comparison_thresholds.cache_info()
  assert info.maxsize == 32 and info.currsize == 2 and info.hits == 2
