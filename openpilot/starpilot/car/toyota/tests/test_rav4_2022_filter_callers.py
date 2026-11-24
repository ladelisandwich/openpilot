"""2022 torque RAV4 exact filter owner, with saved AOL ON/OFF."""
import pytest
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.car.toyota.tests import test_rav4_angle_filter_callers as helpers

source_matched_native = helpers.source_matched_native
daemon_gc_scope = helpers.daemon_gc_scope

@pytest.mark.parametrize('alpha,disable,radar_mode', [(False, False, 0), (True, False, 0), (True, False, 1), (True, False, 2), (True, True, 1)])
@pytest.mark.parametrize('aol', [False, True])
def test_2022_actual_callers(alpha, disable, radar_mode, aol, tmp_path):
  helpers.run_case(CAR.TOYOTA_RAV4_TSS2_2022, alpha, disable, radar_mode, tmp_path, aol=aol)
