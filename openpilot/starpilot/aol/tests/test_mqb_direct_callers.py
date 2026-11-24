"""Direct-drive parser/Card/SD/Controls/native lifecycle contexts."""
import pytest
from openpilot.starpilot.aol.tests import test_mqb_actual_callers as callers


@pytest.mark.parametrize('alpha', (False, True))
@pytest.mark.parametrize('offset,fixed', ((0, False), (0, True), (4, False), (4, True)))
@pytest.mark.parametrize('engage_on_set', (False, True))
def test_direct_actual_callers(alpha, offset, fixed, engage_on_set):
  callers.test_mqb_full_engagement_brake_reverse_drive_actual_callers('direct', alpha, offset, fixed, engage_on_set)
