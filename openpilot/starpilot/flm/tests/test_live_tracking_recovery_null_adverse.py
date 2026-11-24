"""Interrupted numerical null/adverse stress; no native/source authority claim.

The seed ranges, interruption cadence and acceptance gates are declared here
before execution. This suite is an empirical regression, not a probability
bound under arbitrary correlated road observations.
"""
from dataclasses import asdict, dataclass, field
import json

import pytest

from openpilot.starpilot.flm.live_tracking import LiveTrackingOptimizer, TrackingEpisode
from openpilot.starpilot.flm.tests.test_live_tracking_rollout import (
  rollout, wrong_causal_corrections, assert_causal_confirmations,
)

SEEDS = range(64)  # Same fixed size/range as the original stationary null sweep.
INTERRUPT_EVERY = 11  # Same cadence as the original interrupted progress test.


@dataclass
class RecoveryCoverage:
  allocation_calls: int = 0
  allocations: list[tuple[int, int]] = field(default_factory=list)
  recovery_launches: int = 0


def interrupted_case(monkeypatch: pytest.MonkeyPatch, *, seed: int, gain: float):
  coverage = RecoveryCoverage()
  original_allocation = LiveTrackingOptimizer._comparison_allocation
  original_observe = LiveTrackingOptimizer.observe_episode

  def allocation(owner: LiveTrackingOptimizer, episode: TrackingEpisode, expected: float,
                 target: dict[str, float], *, minimum_episodes: int | None = None) -> tuple[int, int] | None:
    result = original_allocation(owner, episode, expected, target, minimum_episodes=minimum_episodes)
    coverage.allocation_calls += 1
    if result is not None:
      coverage.allocations.append(result)
    return result

  def observe(owner: LiveTrackingOptimizer, episode: TrackingEpisode, *, now_ns: int) -> bool:
    selection = owner.discovery
    prior = owner.offer_state
    result = original_observe(owner, episode, now_ns=now_ns)
    state = owner.offer_state
    if (selection is not None and selection['recovery'] and state is not None and state['phase'] == 'trial' and
        (prior is None or prior['trial_id'] != state['trial_id'])):
      coverage.recovery_launches += 1
    return result

  # The observer returns the original decisions unchanged. The context always
  # restores both methods, including when the original helper raises.
  with monkeypatch.context() as patch:
    patch.setattr(LiveTrackingOptimizer, '_comparison_allocation', allocation)
    patch.setattr(LiveTrackingOptimizer, 'observe_episode', observe)
    result = rollout(seed=seed, gain=gain, interrupt_every=INTERRUPT_EVERY)
  return result, coverage


def sweep(monkeypatch: pytest.MonkeyPatch, *, gain: float):
  cases = [interrupted_case(monkeypatch, seed=seed, gain=gain) for seed in SEEDS]
  results = [result for result, _ in cases]
  coverage = [item for _, item in cases]
  failures = [row for row in results if wrong_causal_corrections(row)]
  summary = {'gain': gain, 'seeds': [SEEDS.start, SEEDS.stop], 'routes': len(results),
    'interrupt_every': INTERRUPT_EVERY, 'false_acceptance_routes': len(failures),
    'attempts': sum(len(row['attempts']) for row in results),
    'interruptions': sum(row['interruptions'] for row in results),
    'allocation_calls': sum(item.allocation_calls for item in coverage),
    'successful_allocations': sum(len(item.allocations) for item in coverage),
    'recovery_launches': sum(item.recovery_launches for item in coverage),
    'route_coverage': [dict(seed=seed, **asdict(item)) for seed, item in zip(SEEDS, coverage, strict=True)],
    'failures': failures}
  print('NUMERICAL_INTERRUPTED_RECOVERY_SWEEP '+json.dumps(summary, sort_keys=True))
  # False-acceptance checks must actually exercise the newly allocated path,
  # rather than passing through a learner that never creates a trial.
  assert all(row['attempts'] for row in results), summary
  assert any(row['interruptions'] for row in results) and any(item.allocation_calls for item in coverage), summary
  assert any(item.allocations for item in coverage) and any(item.recovery_launches for item in coverage), summary
  return results, summary


def test_interrupted_correlated_null_preserves_original_family_false_acceptance_gate(monkeypatch: pytest.MonkeyPatch):
  results, summary = sweep(monkeypatch, gain=0.)
  false_acceptances = [row for row in results if row['confirmations']]
  assert len(false_acceptances) <= 3, summary  # Original stationary null criterion.


def test_interrupted_correlated_adverse_trials_never_accumulate_wrong_correction(monkeypatch: pytest.MonkeyPatch):
  results, summary = sweep(monkeypatch, gain=-5.)
  assert any(row['confirmed_steps'] for row in results), summary  # Actual beneficial direction-search is exercised.
  for row in results:
    assert_causal_confirmations(row)
    assert row['accepted_coordinate'] <= 0., summary
    assert row['final_expected_normalized_error'] <= row['initial_expected_normalized_error'], summary
  assert summary['false_acceptance_routes'] == 0, summary
