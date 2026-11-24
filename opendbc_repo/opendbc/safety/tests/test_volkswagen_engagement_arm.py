"""Matching native policy regression; requires root's rebuilt libsafety."""
import pytest
from opendbc.safety.tests import test_volkswagen_stock_aol as stock_aol_tests


@pytest.mark.parametrize('word', (8, 24))
def test_fresh_engagement_arms_without_main_cycle_and_fault_needs_new_edge(word):
  owner = stock_aol_tests.TestVolkswagenStockAol()
  try:
    owner.configure(word)
    for _ in range(12):
      owner.feed()
    assert not owner.safety.get_controls_allowed()
    assert owner.safety.aol_get_permission_mask() == 1  # Fresh default MAIN-on may request lateral before SET.
    owner.feed(engaged=True)
    owner.feed(brake=True)
    assert not owner.safety.get_controls_allowed()
    assert owner.safety.aol_get_permission_mask() == 1
    owner.feed(gear=False)
    assert owner.safety.aol_get_permission_mask() == 0
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 1
    owner.feed(engaged=True, permanent=True)
    assert owner.safety.aol_get_permission_mask() == 0
    owner.feed(engaged=True)
    assert owner.safety.aol_get_permission_mask() == 0
    owner.feed(engaged=False)
    owner.feed(engaged=True)
    assert owner.safety.aol_get_permission_mask() == 1
  finally:
    owner.tearDown()
