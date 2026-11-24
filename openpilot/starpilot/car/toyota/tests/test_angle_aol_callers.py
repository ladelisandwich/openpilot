import pytest
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.car.toyota.tests import test_rav4_angle_filter_callers as helpers
from openpilot.starpilot.car.toyota.tests.test_toyota_cpp_transport import host_heartbeat_bridge as host_heartbeat_bridge

source_matched_native = helpers.source_matched_native
daemon_gc_scope = helpers.daemon_gc_scope


@pytest.mark.parametrize('filtered', [False, True])
@pytest.mark.parametrize('alpha,disable', [(False, False), (True, False), (True, True)])
@pytest.mark.parametrize('aol', [False, True])
def test_angle_real_callers_cpp_native(filtered, alpha, disable, aol, tmp_path, host_heartbeat_bridge):
  helpers.run_case(CAR.TOYOTA_RAV4_TSS2_2023, alpha, disable, 0, tmp_path,
                   aol=aol, has_filter=filtered, bridge=host_heartbeat_bridge)
