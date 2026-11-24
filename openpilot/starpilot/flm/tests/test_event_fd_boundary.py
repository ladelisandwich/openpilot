"""Rejected nonzero native probe never becomes an invented fitting feature."""
from types import SimpleNamespace
from unittest import mock

import pytest

from openpilot.starpilot.flm.event_controller_replay import EventControllerReplay


def engine():
  value = object.__new__(EventControllerReplay)
  value.engine = SimpleNamespace()  # Explicit empty delegate for this policy-only shell.
  value.restore_envelope = None
  value.objective = SimpleNamespace(parameters=(SimpleNamespace(low=0., high=.25),))
  value.reference = (0.,)
  value.projected = {}
  return value


def test_neutral_only_boundary_has_zero_identifiability():
  value = engine()
  def rollout(coordinates):
    if coordinates != (0.,):
      raise ValueError('actual native interval rejects nonzero candidate')
    return {}, {1: .2, 2: .3}, {}
  with mock.patch.object(value, '_rollout', side_effect=rollout):
    assert value.rebuild_causal_features() == 2
  assert value.projected == {1: (.2, (0.,)), 2: (.3, (0.,))}


def test_changed_native_emission_support_still_rejects():
  value = engine()
  def rollout(coordinates):
    return {}, ({1: .2, 2: .3} if coordinates == (0.,) else {1: .21}), {}
  with mock.patch.object(value, '_rollout', side_effect=rollout), pytest.raises(ValueError, match='emission support changed'):
    value.rebuild_causal_features()


def test_admitted_one_sided_native_probe_keeps_real_derivative():
  value = engine()
  def rollout(coordinates):
    return {}, {1: .2 + 2 * coordinates[0], 2: .3 + coordinates[0]}, {}
  with mock.patch.object(value, '_rollout', side_effect=rollout):
    assert value.rebuild_causal_features() == 2
  assert value.projected[1][1] == pytest.approx((2.,))
  assert value.projected[2][1] == pytest.approx((1.,))
