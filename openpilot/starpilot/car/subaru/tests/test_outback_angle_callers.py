"""Actual automatic/manual factory, Card, SelfdriveD, Controls and native Outback angle joins."""
import pytest
from opendbc.car.subaru.values import CAR
from openpilot.starpilot.car.subaru.tests.native_fixture import subaru_native_pair as subaru_native_pair


@pytest.mark.parametrize('aol', [False, True])
@pytest.mark.parametrize('manual', [False, True])
def test_actual_outback_full_lifecycle(aol, manual, subaru_native_pair, tmp_path):
  from openpilot.starpilot.car.subaru.tests.test_stock_aol_callers import test_actual_full_engage_brake_reverse_temporary_recovery_mainoff
  test_actual_full_engage_brake_reverse_temporary_recovery_mainoff(CAR.SUBARU_OUTBACK_2023, subaru_native_pair, tmp_path,
                                                                 aol=aol, manual=manual)
