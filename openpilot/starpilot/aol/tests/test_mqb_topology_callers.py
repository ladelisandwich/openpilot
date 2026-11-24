"""New topology/fixed contexts through the existing actual caller harness.

Requires source-matched native preloading; no compiler or native permission stub.
"""
import pytest
from openpilot.starpilot.aol.tests import test_mqb_actual_callers as callers


@pytest.mark.parametrize('gearbox', ('automatic', 'manual'))
@pytest.mark.parametrize('alpha', (False, True))
@pytest.mark.parametrize('offset,fixed', ((0, True), (4, False), (4, True)))
@pytest.mark.parametrize('engage_on_set', (False, True))
def test_topology_fixed_actual_callers(gearbox, alpha, offset, fixed, engage_on_set):
  callers.test_mqb_full_engagement_brake_reverse_drive_actual_callers(gearbox, alpha, offset, fixed, engage_on_set)
