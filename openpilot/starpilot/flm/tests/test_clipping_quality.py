"""Encoder-local quality arithmetic/version negatives, not vehicle admission."""
from dataclasses import asdict, replace
import math

from openpilot.starpilot.flm.clipping_quality import (
  RECORDED_DEBT, STATE_ALIGNED_DEBT, clipping_quality, debt_is_admitted, event_clipping_quality,
)
from openpilot.starpilot.flm.event_activation import (
  STATE_ALIGNED_ACTIVATION_VERSION, activation_matches_model, activation_model_from_dict,
  bind_activation_model, model_clipping_quality,
)
from openpilot.starpilot.flm.tests.test_event_activation import fitted_model, primitive_plan


def rejects(call):
  try:
    call()
  except ValueError:
    return
  raise AssertionError('unknown quality admitted')


def test_identical_request_with_different_previous_output_is_not_added_direct_demand():
  # Root's original−90 request: recorded previous−58→−73/debt−17.
  # BOTH requests at candidate previous−57→−72/debt−18.
  assert not debt_is_admitted(-17, -18, -18, RECORDED_DEBT)
  assert debt_is_admitted(-17, -18, -18, STATE_ALIGNED_DEBT)
  # Actually larger request−91 under that SAME history has debt−19.
  assert not debt_is_admitted(-17, -18, -19, STATE_ALIGNED_DEBT)


def test_new_or_opposite_clipping_rejects_including_recorded_direction_counterexample():
  assert not debt_is_admitted(-17, -18, 1, STATE_ALIGNED_DEBT)
  assert not debt_is_admitted(-17, 0, -1, STATE_ALIGNED_DEBT)
  assert not debt_is_admitted(0, -1, -1, STATE_ALIGNED_DEBT)
  # Recorded previous−58 gives−17; candidate previous−200 gives paired
  # original−90→−175/debt+85, candidate−91→−175/debt+84. The recorded
  # direction veto rejects despite an improved same-state magnitude84<85.
  assert not debt_is_admitted(-17, 85, 84, STATE_ALIGNED_DEBT)


def test_recorded_contract_is_exact_original_comparison_for_all_signs():
  for recorded in range(-5, 6):
    for candidate in range(-7, 8):
      old = abs(candidate) <= abs(recorded) and (not candidate or (candidate > 0) == (recorded > 0))
      assert debt_is_admitted(recorded, 999, candidate, RECORDED_DEBT) == bool(old)


def test_bad_support_versions_and_unversioned_optin_fail_closed():
  for position in range(3):
    for bad in (math.nan, math.inf, True):
      values: list[float] = [-17, -18, -18]
      values[position] = bad
      assert not debt_is_admitted(values[0], values[1], values[2], STATE_ALIGNED_DEBT)
  rejects(lambda: clipping_quality('unknown'))
  rejects(lambda: event_clipping_quality({'event_version': 2, 'event_clipping_quality': STATE_ALIGNED_DEBT}))
  rejects(lambda: event_clipping_quality({'event_version': 3, 'event_clipping_quality': 'unknown'}))
  assert event_clipping_quality({'event_version': 3}) == RECORDED_DEBT
  assert event_clipping_quality({'event_version': 3, 'event_clipping_quality': STATE_ALIGNED_DEBT}) == STATE_ALIGNED_DEBT


def test_new_quality_changes_bound_proof_and_old_proof_never_defaults_to_new_quality():
  original = fitted_model()
  old_plan = primitive_plan(original)
  new_plan = replace(old_plan, version=STATE_ALIGNED_ACTIVATION_VERSION)
  old_model, new_model = bind_activation_model(original, old_plan), bind_activation_model(original, new_plan)
  assert old_plan.digest() != new_plan.digest()
  assert old_model.activation_version != new_model.activation_version
  assert model_clipping_quality(old_model) == RECORDED_DEBT
  assert model_clipping_quality(new_model) == STATE_ALIGNED_DEBT
  assert not activation_matches_model(old_plan, new_model)
  assert not activation_matches_model(new_plan, old_model)
  assert activation_model_from_dict(asdict(new_model)) == new_model
  rejects(lambda: model_clipping_quality(replace(new_model, activation_version='unknown')))
