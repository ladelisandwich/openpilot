"""Versioned information budget, not a physical/native qualification fixture."""
from dataclasses import asdict, replace
from unittest import mock

import pytest

from openpilot.starpilot.flm.clipping_quality import (
  FIXED_MAP_IDENTIFICATION, NATIVE_SHADOW_IDENTIFICATION, QUALIFIED_EVENT_CLIPPING_QUALITY,
  RESTORE_ACTIVATION_VERSION, RESTORE_AGGREGATE_ACTIVATION_VERSION, event_clipping_quality,
)
from openpilot.starpilot.flm.event_activation import activation_matches_model, activation_model_from_dict, bind_activation_model
from openpilot.starpilot.flm.event_evaluate import _lower
from openpilot.starpilot.flm.restore_activation import restore_plan_from_dict
from openpilot.starpilot.flm.restore_support import capped_group_credits, independent_half_supported, all_event_lower
from openpilot.starpilot.flm.tests.test_declared_restore import envelope, plan_for, payload
from openpilot.starpilot.flm.tests.test_event_activation import fitted_model


def evidence(counts, *, points=120):
  return tuple((str(index), tuple((1_000_000 * (index * points + row + 1),
    1_000_000 * (index * points + row + 1) + 1, .001 if row < changed else 0.)
    for row in range(points))) for index, changed in enumerate(counts))


@pytest.mark.parametrize('counts', ((40, 40), (29, 29, 29), (20, 20, 20, 20)))
def test_same_original_budget_can_be_distributed_over_more_independent_events(counts):
  support_credits = capped_group_credits(evidence(counts))
  assert independent_half_supported(support_credits)
  assert sum(support_credits) >= 80 and len([credit for credit in support_credits if credit]) >= 2


@pytest.mark.parametrize('counts', ((80,), (79, 1), (39, 40), (0, 0, 0), (120, 0, 0)))
def test_concentrated_insufficient_or_zero_actual_encoded_support_cannot_restore(counts):
  assert not independent_half_supported(capped_group_credits(evidence(counts)))


def test_halves_never_borrow_support_and_duplicates_or_short_events_reject():
  support_credits = capped_group_credits(evidence((40, 0, 40, 0)))
  assert not independent_half_supported(support_credits[:2]) and not independent_half_supported(support_credits[2:])
  rows = evidence((40, 40))
  for malformed in ((rows[0], rows[0]), ((rows[0][0], rows[0][1]), (rows[0][0], rows[1][1])),
                    (rows[0], ('other', rows[0][1])), (('short', rows[0][1][:39]),)):
    with pytest.raises(ValueError):
      capped_group_credits(malformed)
  with pytest.raises(ValueError):
    capped_group_credits((('duplicate-inside', (rows[0][1][0],) * 80),))
  for bad in (float('nan'), float('inf'), -1.):
    changed = list(rows[0][1])
    changed[0] = (*changed[0][:2], bad)
    with pytest.raises(ValueError):
      capped_group_credits((('bad-native', tuple(changed)),))


def test_all_zero_negative_and_noisy_event_gains_are_retained_in_original_lower():
  assert all_event_lower((1., 1., 1.), _lower) == 1.
  for gains in ((1., 1., 0.), (1., 1., -1.), (.01, 1., 3.), (0., 0.), (float('nan'), 1.)):
    assert all_event_lower(gains, _lower) is None
  original = (0., -.1, 1.)
  lower = mock.Mock(return_value=-1.)
  assert all_event_lower(original, lower) is None
  lower.assert_called_once_with(original)


def test_v4_keeps_old_policy_and_v5_requires_new_exact_plan_model_descriptor():
  old = payload()
  new = {**old, 'event_version': 5, 'event_restore_policy': RESTORE_AGGREGATE_ACTIVATION_VERSION}
  assert event_clipping_quality(old) == event_clipping_quality(new) == QUALIFIED_EVENT_CLIPPING_QUALITY
  assert new['event_identification_policy'] == FIXED_MAP_IDENTIFICATION
  for values in ({**new, 'event_restore_policy': RESTORE_ACTIVATION_VERSION},
                 {**old, 'event_restore_policy': RESTORE_AGGREGATE_ACTIVATION_VERSION},
                 {**new, 'event_identification_policy': NATIVE_SHADOW_IDENTIFICATION}):
    with pytest.raises(ValueError):
      event_clipping_quality(values)
  old_plan = plan_for(envelope())
  new_plan = replace(old_plan, version=RESTORE_AGGREGATE_ACTIVATION_VERSION).validate()
  assert restore_plan_from_dict(asdict(new_plan)) == new_plan
  old_model, new_model = bind_activation_model(fitted_model(), old_plan), bind_activation_model(fitted_model(), new_plan)
  assert old_plan.digest() != new_plan.digest()
  assert activation_matches_model(new_plan, activation_model_from_dict(asdict(new_model)))
  assert not activation_matches_model(new_plan, old_model)
  assert not activation_matches_model(old_plan, new_model)
  # The prior predicate remains false for29/event; this policy is deliberately
  # versioned because80 capped support_credits across halves is a new distribution rule.
  assert all(value < 40 for value in capped_group_credits(evidence((29, 29, 29))))


def test_actual_v5_outer_factory_dispatch_and_parent_version_separation():
  from openpilot.starpilot.flm.tests import test_declared_restore as existing
  original = payload()
  row = {**original, 'event_version': 5, 'event_restore_policy': RESTORE_AGGREGATE_ACTIVATION_VERSION}
  # Reuse the real outer child-dispatch unit, only its explicit primitive input
  # version changes. No native/result/validation acceptance is manufactured.
  with mock.patch.object(existing, 'payload', return_value=row):
    existing.test_actual_child_and_outer_factory_dispatch_restore_v4_without_fd()
  existing.test_actual_parent_rejects_restore_report_from_other_configured_version(5, 4)
  existing.test_actual_parent_rejects_restore_report_from_other_configured_version(4, 5)
