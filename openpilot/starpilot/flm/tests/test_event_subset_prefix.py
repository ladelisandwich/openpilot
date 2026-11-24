"""Subset controller prefix only: no native-result/token/admission transfer."""
from dataclasses import replace
import pickle
from types import MappingProxyType
from unittest import mock

import pytest

from openpilot.starpilot.flm.canonical_surface_adapter import GROUPS
from openpilot.starpilot.flm.controller_replay import _run
from openpilot.starpilot.flm.event_controller_replay import EventControllerReplay
from openpilot.starpilot.flm.event_objective import EffectObjective
from openpilot.starpilot.flm.tests.test_event_controller_prefix import fixture, observed


def full_and_subset(domain='torque', nonzero=True, epsilon=.01):
  original = fixture(domain, nonzero=nonzero)
  envelope = original.records[0].effect_trace.envelope.for_trial(GROUPS['ff.left'])
  objective = EffectObjective('ff.left', envelope.trial_names, envelope.trial_anchor, envelope.context_key)
  full = EventControllerReplay(original.engine, objective, envelope.replay_anchor(), activation_plan=original.activation_plan)
  full.rebuild_causal_features(epsilon=epsilon)
  small = envelope.for_trial(('ff_gain_left',))
  selected = EffectObjective('ff.left', small.trial_names, small.trial_anchor, small.context_key)
  subset = EventControllerReplay(original.engine, selected, small.replay_anchor(), activation_plan=original.activation_plan)
  return full, subset


@pytest.mark.parametrize('domain', ('torque', 'starpilot', 'pid'))
@pytest.mark.parametrize('nonzero', (False, True))
@pytest.mark.parametrize('shadow,delta', ((False, .000000001), (True, .01), (True, -.01)))
def test_actual_full_to_subset_prefix_matches_independent_tail_state_logs_maps_and_native(domain, nonzero, shadow, delta):
  full, subset = full_and_subset(domain, nonzero=nonzero)
  source = full._EventControllerReplay__controller_prefix
  assert source is not None and len(source[1]) == 30
  independent = EventControllerReplay(subset.engine, subset.objective, subset.original_anchor, activation_plan=subset.activation_plan)
  assert subset.reuse_subset_causal_features(full) is True
  rebound = subset._EventControllerReplay__controller_prefix
  assert rebound is not None and rebound[0] == subset._causal_input_key(.01)
  assert rebound[0] != source[0] and rebound[1:] == source[1:]
  assert all(type(part) is MappingProxyType and part is not old for part, old in zip(rebound[1:], source[1:], strict=True))
  assert subset._EventControllerReplay__strict_rollouts is None
  assert subset._EventControllerReplay__identification_token is not full._EventControllerReplay__identification_token
  with pytest.raises(TypeError):
    rebound[1][0] = 1.
  coordinates = (subset.reference[0] + delta,)
  expected, before, native_before = observed(independent, coordinates, shadow=shadow)
  actual, after, native_after = observed(subset, coordinates, shadow=shadow)
  assert pickle.dumps(actual, protocol=5) == pickle.dumps(expected, protocol=5)
  assert len(before) == 60 and len(after) == 30 and after == before[30:]
  assert native_after == native_before


@pytest.mark.parametrize('defect', ('missing', 'stale', 'mutable', 'bad_raw_type', 'bad_snapshot', 'nondefault_epsilon'))
def test_missing_stale_unknown_prefix_does_not_change_valid_fd_transfer(defect):
  epsilon = .02 if defect == 'nondefault_epsilon' else .01
  full, subset = full_and_subset(epsilon=epsilon)
  source = full._EventControllerReplay__controller_prefix
  if defect == 'missing':
    full._EventControllerReplay__controller_prefix = None
  elif defect == 'stale':
    full._EventControllerReplay__controller_prefix = (b'not-current', *source[1:])
  elif defect == 'mutable':
    full._EventControllerReplay__controller_prefix = (source[0], dict(source[1]), *source[2:])
  elif defect == 'bad_raw_type':
    raw = dict(source[1])
    raw[next(iter(raw))] = 0  # Must not alias raw Float64 output.
    full._EventControllerReplay__controller_prefix = (source[0], MappingProxyType(raw), *source[2:])
  elif defect == 'bad_snapshot':
    snapshots = dict(source[3])
    snapshots[next(iter(snapshots))] = object()
    full._EventControllerReplay__controller_prefix = (*source[:3], MappingProxyType(snapshots))
  assert subset.reuse_subset_causal_features(full, epsilon=epsilon) is True
  assert subset._EventControllerReplay__controller_prefix is None
  assert subset.projected and subset._EventControllerReplay__strict_rollouts is None
  with mock.patch('openpilot.starpilot.flm.event_controller_replay._run', wraps=_run) as run:
    subset._rollout((subset.reference[0] + .000000001,),
      _identification_token=subset._EventControllerReplay__identification_token)
    assert run.call_count == 60


@pytest.mark.parametrize('change', ('prior', 'foreign_engine', 'plan', 'late_fault', 'native'))
def test_existing_fd_guard_failure_never_transfers_controller_certificate(change):
  full, subset = full_and_subset()
  if change == 'prior':
    subset.prior_full['ff_gain_right'] = .002
  elif change == 'foreign_engine':
    other, _ = full_and_subset()
    subset.engine = other.engine
  elif change == 'plan':
    subset.activation_plan = replace(subset.activation_plan, physical_context='c' * 64)
  elif change == 'late_fault':
    full.records[-1] = replace(full.records[-1], healthy=False)
  else:
    stamp, event = next(iter(full.emissions.events.items()))
    full.emissions.events[stamp] = replace(event, witness=event.witness._replace(final_raw=event.witness.final_raw + 1))
  assert subset.reuse_subset_causal_features(full) is False
  assert subset._EventControllerReplay__controller_prefix is None
  assert subset._EventControllerReplay__strict_rollouts is None


def test_source_shadow_token_cannot_become_destination_permission():
  full, subset = full_and_subset()
  assert subset.reuse_subset_causal_features(full)
  with pytest.raises(ValueError, match='identification scope'):
    subset._rollout((.01,), _identification_token=full._EventControllerReplay__identification_token)
