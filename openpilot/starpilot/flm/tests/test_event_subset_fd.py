"""Subset FD equivalence; no manufactured fit, model or current authority."""
from dataclasses import asdict, replace
from types import SimpleNamespace
from unittest import mock

import pytest

from openpilot.starpilot.flm.canonical_surface_adapter import GROUPS, NAMES
from openpilot.starpilot.flm.clipping_quality import ZERO_REFERENCE_QUALITY, NATIVE_SHADOW_IDENTIFICATION
from openpilot.starpilot.flm.event_controller_replay import EventControllerReplay
from openpilot.starpilot.flm.event_fit_factory import fit_event_snapshot
from openpilot.starpilot.flm.event_objective import EffectObjective
from openpilot.starpilot.flm.selected_effects import EffectTrial, EffectReplayTransition
from openpilot.starpilot.flm.tests.test_full_surface_profile import base
from openpilot.starpilot.flm.tests.test_event_lazy_native import replay_fixture
from openpilot.starpilot.flm.torque_adapter import Objective


def full_and_subset(domain='torque'):
  original = replay_fixture(domain).engine
  envelope = base(names=GROUPS['ff.left'])
  objective = EffectObjective('ff.left', envelope.trial_names, envelope.trial_anchor, envelope.context_key)
  full = EventControllerReplay(original, objective, envelope.replay_anchor())
  full.rebuild_causal_features()
  envelope = envelope.for_trial(('ff_gain_left',))
  objective = EffectObjective('ff.left', envelope.trial_names, envelope.trial_anchor, envelope.context_key)
  return full, EventControllerReplay(original, objective, envelope.replay_anchor())


@pytest.mark.parametrize('domain', ('torque', 'starpilot', 'pid'))
def test_actual_full_fd_columns_equal_independent_selected_controller_and_native_rebuild(domain):
  full, subset = full_and_subset(domain)
  with mock.patch.object(subset, '_rollout', side_effect=AssertionError('transfer cannot replay or mint a native result')):
    assert subset.reuse_subset_causal_features(full)
  copied = dict(subset.projected)
  assert copied == {stamp: (row[0], (row[1][0],)) for stamp, row in full.projected.items()}
  assert subset._EventControllerReplay__strict_rollouts is None
  frozen = subset._EventControllerReplay__causal_features[1]
  with pytest.raises(TypeError):
    frozen[0] = (0., (1.,))
  subset.projected.clear()
  with mock.patch.object(subset, 'rebuild_causal_features', side_effect=AssertionError('fresh bound key must hit')):
    assert subset.ensure_causal_features() == len(copied)
  assert subset.projected == copied
  independent = EventControllerReplay(subset.engine, subset.objective, subset.original_anchor)
  independent.rebuild_causal_features()
  assert independent.projected == copied


@pytest.mark.parametrize('change', ('context', 'frame', 'native', 'prior_bits', 'mutable_anchor', 'plan',
                                   'factory', 'engine', 'epsilon', 'missing_cache', 'nonfloat_prior'))
def test_changed_or_unknown_owned_input_cannot_rebind_full_feature_proof(change):
  full, subset = full_and_subset()
  epsilon = .01
  if change == 'context':
    subset.objective = replace(subset.objective, context_key='1' * 64)
  elif change == 'frame':
    full.records[0] = replace(full.records[0], selected_command=-0.)
  elif change == 'native':
    stamp, event = next(iter(full.emissions.events.items()))
    full.emissions.events[stamp] = replace(event, witness=event.witness._replace(final_raw=event.witness.final_raw + 1))
  elif change == 'prior_bits':
    subset.prior_full['ff_gain_left'] = -0.
  elif change == 'mutable_anchor':
    subset.original_anchor['accumulated'] = list(subset.original_anchor['accumulated'])
  elif change == 'plan':
    subset.activation_plan = SimpleNamespace(version='unknown')
  elif change == 'factory':
    full.factory.manual_source_epoch = 'changed-factory'
  elif change == 'engine':
    subset.engine = SimpleNamespace(**vars(full.engine))
  elif change == 'epsilon':
    epsilon = .02
  elif change == 'missing_cache':
    full._EventControllerReplay__causal_features = None
  else:
    full.prior_full['ff_gain_left'] = 0
  assert not subset.reuse_subset_causal_features(full, epsilon=epsilon)
  assert subset._EventControllerReplay__causal_features is None and not subset.projected


def test_nonzero_float_prior_full30_targets_and_source_geometry_are_subset_invariant():
  prior = dict(base(names=GROUPS['ff.left']).prior_coordinates)
  prior['turn_in_boost_left'] = .002
  prior['center_taper_max'] = .003
  prior['ff_gain_right'] = -0.
  envelope = base(names=GROUPS['ff.left'])
  from openpilot.starpilot.flm.canonical_surface_adapter import full_profile_sha256
  coordinates = tuple((name, prior[name]) for name in NAMES)
  digest = full_profile_sha256(envelope.selected_baseline_revision, coordinates)
  envelope = replace(envelope, prior_coordinates=coordinates, coordinates=coordinates,
    prior_full_profile_sha256=digest, full_profile_sha256=digest).validate()
  for direction in (-1, 1):
    small = envelope.for_trial(('ff_gain_left',))
    left_objective = EffectObjective('ff.left', envelope.trial_names, envelope.trial_anchor, envelope.context_key)
    right_objective = EffectObjective('ff.left', small.trial_names, small.trial_anchor, small.context_key)
    full = tuple(prior[name] + direction * .01 if name == 'ff_gain_left' else prior[name] for name in envelope.trial_names)
    left = EffectTrial(left_objective, full, envelope.selected_baseline_revision, envelope.replay_anchor())
    right = EffectTrial(right_objective, (direction * .01,), envelope.selected_baseline_revision, small.replay_anchor())
    assert EffectReplayTransition(left).target == EffectReplayTransition(right).target
    assert left.source().values == right.source().values
    assert left.source().full_profile_sha256 == right.source().full_profile_sha256


@pytest.mark.parametrize('reuse', (True, False, 'untrusted-true'))
def test_actual_factory_uses_transfer_only_for_selected_subset_and_falls_back_exactly(reuse):
  envelope = base(names=GROUPS['ff.left'])
  objective = EffectObjective('ff.left', envelope.trial_names, envelope.trial_anchor, envelope.context_key)
  payload = {'event_version': 3, 'event_operation': 'trial', 'event_clipping_quality': ZERO_REFERENCE_QUALITY,
    'event_identification_policy': NATIVE_SHADOW_IDENTIFICATION, 'effect_envelope': asdict(envelope),
    'event_objective': asdict(objective), 'objective': asdict(Objective('feedforward', 'high', 'steady')),
    'physical_context': ('unit-boundary',)}
  # Boundary stubs expose routing only; they return no model/profile/validation.
  first, full, subset = mock.Mock(), mock.Mock(), mock.Mock()
  subset.reuse_subset_causal_features.return_value = reuse
  expected = {'status': 'collecting', 'reason': 'unit-boundary-only'}
  with mock.patch('openpilot.starpilot.flm.event_fit_factory.ControllerReplay.from_snapshot') as original, \
       mock.patch('openpilot.starpilot.flm.event_fit_factory.EventControllerReplay', side_effect=(first, full, subset)), \
       mock.patch('openpilot.starpilot.flm.event_raw_fit.prepare_fixed_event_calibration', return_value=(object(), 1, (), ())), \
       mock.patch('openpilot.starpilot.flm.event_activation.build_activation_plan', return_value=object()), \
       mock.patch('openpilot.starpilot.flm.runtime_adapter_registry.canonical_physical_context', return_value='unit'), \
       mock.patch('openpilot.starpilot.flm.event_fit_factory._identified', return_value=('ff_gain_left',)), \
       mock.patch('openpilot.starpilot.flm.event_raw_fit.fit_event_torque', return_value=expected) as fit:
    assert fit_event_snapshot(payload, object(), {'baseline_revision': envelope.selected_baseline_revision}) == expected
    subset.reuse_subset_causal_features.assert_called_once_with(full)
    assert subset.rebuild_causal_features.call_count == (0 if reuse is True else 1)
    assert full.rebuild_causal_features.call_count == 1
    assert fit.call_args.args[1] is subset and original.call_args.kwargs == {'rebuild': False}


def test_unowned_legacy_shells_do_not_proxy_recurse_or_gain_readiness():
  source = EventControllerReplay.__new__(EventControllerReplay)
  destination = EventControllerReplay.__new__(EventControllerReplay)
  assert not destination.reuse_subset_causal_features(source)
  assert not destination.reuse_subset_causal_features(object())
