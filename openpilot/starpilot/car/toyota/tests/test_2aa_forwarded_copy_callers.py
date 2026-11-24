import pytest
from opendbc.car.toyota import radar_filter
from openpilot.starpilot.car.toyota.tests import test_rav4_angle_filter_callers as helpers
from openpilot.starpilot.car.toyota.tests.test_toyota_cpp_transport import host_heartbeat_bridge as host_heartbeat_bridge

source_matched_native = helpers.source_matched_native
daemon_gc_scope = helpers.daemon_gc_scope


@pytest.mark.parametrize('candidate', sorted(radar_filter.PROFILES))
@pytest.mark.parametrize('alpha,disable', [(False, False), (True, False), (True, True)])
@pytest.mark.parametrize('aol', [False, True])
def test_forwarded_advertisement_actual_callers(candidate, alpha, disable, aol, tmp_path, host_heartbeat_bridge):
  advertisements = [(0x2AA, bytes(8), bus) for bus in (0, 2)]
  helpers.run_case(candidate, alpha, disable, 0, tmp_path, aol=aol, bridge=host_heartbeat_bridge, filter_advertisements=advertisements)
