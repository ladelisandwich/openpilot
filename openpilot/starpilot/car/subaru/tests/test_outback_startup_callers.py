"""Actual Outback startup receiver topology, default-off and one-shot request."""
import pytest
from opendbc.car.subaru.values import CAR
from openpilot.starpilot.car.subaru.tests.native_fixture import subaru_native_pair as subaru_native_pair


@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.parametrize('manual', [False, True])
@pytest.mark.parametrize('aol', [False, True])
def test_actual_outback_startup(enabled, manual, aol, subaru_native_pair, tmp_path):
  from openpilot.starpilot.car.subaru.tests.test_stop_start_callers import test_actual_startup_factory_card_controls_native
  test_actual_startup_factory_card_controls_native(enabled, manual, aol, subaru_native_pair, tmp_path,
                                                 candidate=CAR.SUBARU_OUTBACK_2023)
