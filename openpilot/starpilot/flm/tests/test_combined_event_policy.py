"""Policy/proof boundaries; primitive lineage is not a qualified replay."""
from dataclasses import asdict, replace
from types import SimpleNamespace
from unittest import mock

import pytest

from openpilot.starpilot.flm.clipping_quality import (
  STATE_ALIGNED_RELIEF, NATIVE_SHADOW_IDENTIFICATION, STRICT_IDENTIFICATION,
  event_clipping_quality, native_relief_is_admitted,
)
from openpilot.starpilot.flm.event_activation import (
  COMBINED_ACTIVATION_VERSION, activation_matches_model, activation_model_from_dict, bind_activation_model,
)
from openpilot.starpilot.flm.event_controller_replay import EventControllerReplay
from openpilot.starpilot.flm.tests.test_event_activation import fitted_model, primitive_plan
from openpilot.starpilot.flm.tests.test_event_fd_boundary import engine


def test_exact_unwind_witnesses_and_partial_order_negatives():
  assert native_relief_is_admitted(-76, -76, -75, -80, -79, -79, 4, 3, 4)
  assert native_relief_is_admitted(-19, -19, -18, -28, -28, -28, 9, 9, 10)
  valid: list[float] = [-76, -76, -75, -80, -79, -79, 4, 3, 4]
  for index, value in ((2, -77), (2, 0), (2, 1), (5, -81), (5, 79), (6, -4), (7, -3), (8, -4), (8, 0), (1, float('nan'))):
    changed = valid.copy()
    changed[index] = value
    assert not native_relief_is_admitted(*changed)


def test_explicit_policy_pair_and_model_revision_cannot_downgrade():
  payload = {'event_version': 3, 'event_clipping_quality': STATE_ALIGNED_RELIEF,
             'event_identification_policy': NATIVE_SHADOW_IDENTIFICATION}
  assert event_clipping_quality(payload) == STATE_ALIGNED_RELIEF
  for change in ({'event_identification_policy': STRICT_IDENTIFICATION}, {'event_identification_policy': None},
                 {'event_version': 2}, {'event_clipping_quality': 'unknown'}):
    with pytest.raises(ValueError):
      event_clipping_quality({**payload, **change})
  with pytest.raises(ValueError):
    event_clipping_quality({key: value for key, value in payload.items() if key != 'event_identification_policy'})
  model = fitted_model()
  old = primitive_plan(model)
  new = replace(old, version=COMBINED_ACTIVATION_VERSION).validate()
  assert new.identification_policy == NATIVE_SHADOW_IDENTIFICATION
  assert new.clipping_quality == STATE_ALIGNED_RELIEF
  bound = bind_activation_model(model, new)
  assert new.digest() != old.digest()
  assert not activation_matches_model(old, bound)
  assert activation_matches_model(new, activation_model_from_dict(asdict(bound)))
  assert not activation_matches_model(new, bind_activation_model(model, old))


def test_shadow_secants_are_scoped_and_final_rollout_has_no_sticky_mode():
  value = engine()
  value.activation_plan = SimpleNamespace(identification_policy=NATIVE_SHADOW_IDENTIFICATION)
  token = value._EventControllerReplay__identification_token = object()
  calls = []
  def rollout(coordinates, **kwargs):
    calls.append((coordinates, kwargs))
    if coordinates != value.reference and kwargs.get('_identification_token') is not token:
      raise ValueError('final native clipping quality')
    return {}, {1: .2 + coordinates[0], 2: .3 + coordinates[0]}, {}
  with mock.patch.object(value, '_rollout', side_effect=rollout):
    assert value.rebuild_causal_features() == 2
    # Final calls get no token; rebuilding does not mutate an engine mode.
    with pytest.raises(ValueError, match='final native clipping quality'):
      value._rollout((.01,))
  assert calls[0] == ((0.,), {})
  assert all(kwargs == {'_identification_token': token} for _, kwargs in calls[1:3])
  assert calls[-1] == ((.01,), {})
  assert value.projected[1][1] == pytest.approx((1.,))


def test_forged_identification_token_rejects_before_controller_execution():
  value = object.__new__(EventControllerReplay)
  value.engine = SimpleNamespace()  # Explicit empty delegate; no native/controller admission.
  value._EventControllerReplay__identification_token = object()
  value.activation_plan = SimpleNamespace(identification_policy=NATIVE_SHADOW_IDENTIFICATION)
  with pytest.raises(ValueError, match='scope/policy'):
    value._rollout((), _identification_token=object())
