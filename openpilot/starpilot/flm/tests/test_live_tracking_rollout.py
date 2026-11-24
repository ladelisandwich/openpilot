"""Known-plant numerical trials; NO source/controller/native authority evidence.

These tests synthesize certificates, physical residual witnesses and target
acknowledgments to exercise the optional learner's decisions. Coordinate
quantization below is a declared model assumption, not a real CAN encoder.
Root must run these inside the separately qualified candidate closure.
"""
from dataclasses import replace
import hashlib
import json
import math
import random
import statistics

import pytest

from openpilot.starpilot.flm.canonical_surface_adapter import NAMES
from openpilot.starpilot.flm.live_tracking import HookSensitivity, LiveTrackingOptimizer, TrackingEpisode, _critical
from openpilot.starpilot.flm.live_tracking_response import ResponseCertificate, ResponseWitness, VERSION, _revision, validate_certificate
from openpilot.starpilot.flm.live_trial_contract import LiveTrialBinding, MAX_STEP

NS = 1_000_000_000
HOOK = 'ff_gain_left'


def _optimizer():
  binding = LiveTrialBinding('a'*64, 'b'*64, 'actual-controller-registry-v2:'+'c'*64, 'd'*64, 'e'*64,
    'numerical-reset', 'f'*64, 'a'*64, '', '0'*64, 'b'*64, False)
  raw = ResponseCertificate(VERSION, binding, '0'*64, 170_000_000, 3., 0., -.5, .5, .005,
    ('train0', 'train1', 'train2', 'train3'), ('hold0', 'hold1', 'hold2', 'hold3'), 'c'*64, 'd'*64, 1)
  revision = _revision(raw)
  certificate = validate_certificate(replace(raw, model_revision=revision,
    binding=replace(binding, physical_model_revision=revision)))
  return LiveTrackingOptimizer(certificate.binding, tuple((name, 0.) for name in NAMES),
    controller_domain='torque', response_certificate=certificate)


def _refresh(owner, now):
  witness = ResponseWitness(owner.certificate, owner.binding, now-180_000_000, now-180_000_000,
    now, now-350_000_000, 8, .005, now+100_000_000)
  assert owner.response_current(witness, now_ns=now)


def _ack(owner, offer, now):
  _refresh(owner, now)
  assert owner.acknowledge_application(trial_id=offer.trial_id, target_sequence=offer.target_sequence,
    phase=offer.phase, target_sha256=offer.target_sha256, applied_full30=offer.target,
    control_ns=now, binding=offer.binding, now_ns=now, status='applied',
    confirmed_target_sha256=offer.target_sha256 if offer.phase == 'confirmed' else '')


# Same synthetic plant slope; physical RMS is measured at the current trial cap.
FIXTURE_PHYSICAL_DERIVATIVE = 3.0


def _episode(owner, index, now, profile, goal, score, speed):
  # A physically visible algorithmic probe can still have the wrong causal
  # direction. Future observations, rather than this probe, must decide keep.
  sensitivity = HookSensitivity(HOOK, .06, 1., 20, 80, FIXTURE_PHYSICAL_DERIVATIVE * MAX_STEP, native_gradient_complete=True)
  return TrackingEpisode(owner.binding, owner._hash(profile), hashlib.sha256(f'rollout-{index}'.encode()).hexdigest(),
    now-4*NS, now, 'large', 1, speed, speed*1.02, goal, score*goal, .1,
    80, 10_000_000, (sensitivity,), (1.,)*8, native_gradient_complete=True)


def rollout(*, seed, gain=5., noise_sd=.008, correlation=.65, demand_change=False, interrupt_every=0):
  """Twenty minutes, one eligible 4s maneuver every8s, AR1 road/noise.

  gain=0 is the causal null; gain<0 is an adverse controller change. Probes
  remain optimistic in both cases to force the independent future test to
  reject false starts. The desired magnitude and speed vary independently.
  """
  owner, rng = _optimizer(), random.Random(seed)
  first_trial = first_confirmation = None
  confirmations = rollbacks = interruptions = 0
  noise = 0.
  attempts = []
  decisions = []
  confirmed_steps = []
  interrupted_ids = set()
  last_interrupt_ns = None
  rolled_back = set()
  scores = []
  for index in range(1, 151):
    now = index*8*NS
    _refresh(owner, now)
    state = owner.offer_state
    active = state is not None and state['phase'] == 'trial' and not state.get('accepted', False)
    profile = state['target'] if active else owner.accepted
    # Explicit model-domain quantization: 0.0001 coordinate resolution.
    encoded_coordinate = round(profile[HOOK]/.0001)*.0001
    noise = correlation*noise+math.sqrt(1-correlation**2)*rng.gauss(0., noise_sd)
    goal = (.48 if active else .60) if demand_change else .60*(1+rng.uniform(-.06, .06))
    speed = 22.*(1+rng.uniform(-.025, .025))
    score = max(.025, .35-gain*encoded_coordinate+noise+rng.gauss(0., noise_sd*.35))
    scores.append(score)
    row = _episode(owner, index, now, profile, goal, score, speed)
    if active and interrupt_every and index % interrupt_every == 0:
      interrupted_ids.update(item.episode_id for item in (*owner.before, *owner.future, row))
      last_interrupt_ns = now
      owner.interrupt('numerical_driver_override')
      interruptions += 1
    else:
      owner.observe_episode(row, now_ns=now)
    offer = owner.offer(now)
    if offer is None:
      continue
    if offer.phase == 'trial' and owner.settled_ns is None:
      first_trial = first_trial or now/NS
      attempts.append({'seconds': now/NS, 'quota': owner.offer_state['required_episodes'],
        'alpha': owner.offer_state['alpha'], 'baseline_count': len(owner.before),
        'critical': owner.offer_state['critical'],
        'baseline_score_sd': statistics.stdev(row.error_rms/row.goal_rms for row in owner.before)})
      # The bounded .005 step needs1s at the specified .005/s slew. This is
      # simulated receipt time, never a claim of actual owner application.
      _ack(owner, offer, now+NS)
    elif offer.phase == 'confirmed' and not owner.offer_state.get('accepted', False):
      state = owner.offer_state
      baseline_coordinate, target_coordinate = state['baseline'][HOOK], state['target'][HOOK]
      confirmed_steps.append({
        'baseline_coordinate': baseline_coordinate, 'target_coordinate': target_coordinate,
        'baseline_full30': tuple((name, state['baseline'][name]) for name in NAMES),
        'target_full30': tuple((name, state['target'][name]) for name in NAMES),
        'baseline_expected_error': max(.025, .35-gain*round(baseline_coordinate/.0001)*.0001),
        'target_expected_error': max(.025, .35-gain*round(target_coordinate/.0001)*.0001),
        'before_scores': tuple(item.error_rms/item.goal_rms for item in owner.before),
        'future_scores': tuple(item.error_rms/item.goal_rms for item in owner.future),
        'before_ids': tuple(item.episode_id for item in owner.before),
        'future_ids': tuple(item.episode_id for item in owner.future),
        'before_start_ns': tuple(item.start_ns for item in owner.before),
        'future_start_ns': tuple(item.start_ns for item in owner.future),
        'interrupted_ids': tuple(sorted(interrupted_ids)), 'last_interrupt_ns': last_interrupt_ns,
        'started_ns': state['started_ns'], 'settled_ns': owner.settled_ns, 'lag_ns': owner.lag_ns,
        'target_sequence': state['target_sequence'], 'alpha': state['alpha'],
        'required_episodes': state['required_episodes'], 'look_quotas': state['look_quotas'],
      })
      decisions.append({'seconds': now/NS, 'phase': offer.phase, 'reason': owner.reason,
        'future_count': len(owner.future), 'elapsed_seconds': (now-offer.started_ns)/NS})
      _ack(owner, offer, now+10_000_000)
      confirmations += 1
      first_confirmation = first_confirmation or (now+.01*NS)/NS
    elif offer.phase == 'rollback' and (offer.trial_id, offer.target_sequence) not in rolled_back:
      decisions.append({'seconds': now/NS, 'phase': offer.phase, 'reason': owner.reason,
        'future_count': len(owner.future), 'elapsed_seconds': (now-offer.started_ns)/NS})
      _ack(owner, offer, now+NS)
      rolled_back.add((offer.trial_id, offer.target_sequence))
      rollbacks += 1
  result = {'seed': seed, 'seconds': 1200, 'first_trial_seconds': first_trial,
    'first_confirmation_seconds': first_confirmation, 'confirmations': confirmations,
    'rollbacks': rollbacks, 'interruptions': interruptions, 'attempts': attempts, 'decisions': decisions,
    'confirmed_steps': confirmed_steps,
    'accepted_coordinate': owner.accepted[HOOK], 'initial_expected_normalized_error': .35,
    'final_expected_normalized_error': max(.025, .35-gain*round(owner.accepted[HOOK]/.0001)*.0001),
    'last_10_episode_mean_normalized_error': sum(scores[-10:])/10}
  return result


def wrong_causal_corrections(result):
  # The declared plant can have either sign. A beneficial reversal is not a
  # false acceptance; a null-gain confirmation remains a false acceptance.
  return [step for step in result['confirmed_steps']
    if not step['target_expected_error'] < step['baseline_expected_error']]


def assert_causal_confirmations(result):
  steps = result['confirmed_steps']
  assert result['confirmations'] == len(steps), result
  assert not wrong_causal_corrections(result), result
  for step in steps:
    baseline, target = dict(step['baseline_full30']), dict(step['target_full30'])
    assert tuple(baseline) == tuple(target) == NAMES, step
    assert {name for name in NAMES if baseline[name] != target[name]} == {HOOK}, step
    assert statistics.mean(step['future_scores']) < statistics.mean(step['before_scores']), step
    assert set(step['before_ids']).isdisjoint(step['future_ids']), step
    assert (set(step['before_ids']) | set(step['future_ids'])).isdisjoint(step['interrupted_ids']), step
    assert len(step['future_ids']) == step['required_episodes'], step
    assert step['required_episodes'] in step['look_quotas'], step
    assert step['alpha'] == .05/(step['target_sequence']*(step['target_sequence']+1)), step
    assert min(step['future_start_ns']) > step['settled_ns']+step['lag_ns'], step


def test_student_critical_matches_independent_published_reference_values():
  # Standard Student t quantiles: no SciPy dependency and no self-derived oracle.
  for df, alpha, expected in ((1, .025, 12.7062047364), (2, .025, 4.3026527297),
                              (8, .025, 2.3060041352), (23, .01, 2.4998667395)):
    assert _critical(df, alpha) == pytest.approx(expected, rel=2e-9)


def test_twenty_minute_correlated_noisy_known_plant_makes_multiple_useful_corrections():
  result = rollout(seed=20261019)
  print('NUMERICAL_TWENTY_MINUTE_ROLLOUT '+json.dumps(result, sort_keys=True))
  assert result['first_trial_seconds'] is not None and result['first_trial_seconds'] <= 180, result
  assert result['first_confirmation_seconds'] is not None and result['first_confirmation_seconds'] <= 300, result
  assert result['confirmations'] >= 3, result
  assert result['accepted_coordinate'] > 0, result
  assert result['final_expected_normalized_error'] <= .8*result['initial_expected_normalized_error'], result


def test_proportional_demand_change_without_controller_benefit_never_confirms():
  result = rollout(seed=1, gain=0., noise_sd=0., demand_change=True)
  assert len(result['attempts']) >= 3 and result['rollbacks'] >= 3, result
  assert result['confirmations'] == 0 and result['accepted_coordinate'] == 0., result


def test_subtle_proportional_demand_change_below_score_floor_never_confirms():
  owner = _optimizer()
  for index in range(1, 25):
    now = index*8*NS
    _refresh(owner, now)
    state = owner.offer_state
    active = state is not None and state['phase'] == 'trial'
    profile = state['target'] if active else owner.accepted
    # Both errors exceed the subtle eligibility floor. Tracking quality is
    # exactly unchanged: error/desired=1 on both sets of maneuvers.
    goal = .016 if active else .020
    row = replace(_episode(owner, index, now, profile, goal, 1., 22.), kind='subtle',
      sensitivity=(HookSensitivity('center_taper_max', .06, 1., 20, 80, FIXTURE_PHYSICAL_DERIVATIVE * MAX_STEP),))
    owner.observe_episode(row, now_ns=now)
    offer = owner.offer(now)
    if offer is None:
      continue
    assert offer.phase != 'confirmed', owner.report(now)
    if offer.phase == 'trial' and owner.settled_ns is None or offer.phase == 'rollback':
      _ack(owner, offer, now+NS)


def test_repeated_noisy_adverse_trials_revert_without_accumulating_wrong_correction():
  result = rollout(seed=20261020, gain=-5.)
  assert len(result['attempts']) >= 3 and result['rollbacks'] >= 3, result
  assert result['confirmed_steps'], result  # Exercise actual beneficial reversal, never a vacuous no-trial pass.
  assert_causal_confirmations(result)
  assert result['accepted_coordinate'] < 0., result
  assert result['final_expected_normalized_error'] < result['initial_expected_normalized_error'], result


def test_driver_interruptions_do_not_prevent_later_independent_useful_trials():
  result = rollout(seed=20261021, interrupt_every=11)
  assert result['interruptions'] >= 1 and result['rollbacks'] >= 1, result
  # The shipped stricter independent quotas leave only one completed correction
  # in this unchanged20min/88s-interruption case. Prove causal recovery and
  # spent-alpha/fresh-row boundaries instead of an obsolete correction count.
  assert result['confirmed_steps'] and result['accepted_coordinate'] > 0., result
  assert_causal_confirmations(result)
  for step in result['confirmed_steps']:
    assert step['last_interrupt_ns'] is not None, step
    assert step['started_ns'] > step['last_interrupt_ns'], step
    assert min(step['before_start_ns']) > step['last_interrupt_ns'], step
  assert result['final_expected_normalized_error'] < result['initial_expected_normalized_error'], result


def test_correlated_null_seed_sweep_reports_family_false_acceptances():
  # This finite deterministic stress suite is an empirical regression gate,
  # not a proof of a5% probability guarantee under arbitrary road correlation.
  results = [rollout(seed=seed, gain=0.) for seed in range(64)]
  false_acceptances = [row for row in results if row['confirmations']]
  summary = {'routes': len(results), 'false_acceptance_routes': len(false_acceptances),
    'attempts': sum(len(row['attempts']) for row in results), 'failures': false_acceptances}
  print('NUMERICAL_CORRELATED_NULL_SWEEP '+json.dumps(summary, sort_keys=True))
  assert all(row['attempts'] for row in results), summary
  assert len(false_acceptances) <= 3, summary
