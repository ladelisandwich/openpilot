import pytest
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.car.toyota.tests import test_rav4_angle_filter_callers as helpers
from openpilot.starpilot.car.toyota.tests.test_toyota_cpp_transport import host_heartbeat_bridge as host_heartbeat_bridge

source_matched_native=helpers.source_matched_native
daemon_gc_scope=helpers.daemon_gc_scope


@pytest.mark.parametrize('candidate',[CAR.TOYOTA_RAV4_TSS2_2022,CAR.TOYOTA_CAMRY,CAR.TOYOTA_CHR,CAR.TOYOTA_CHR_TSS2])
@pytest.mark.parametrize('alpha,disable',[(False,False),(True,False),(True,True)])
@pytest.mark.parametrize('aol',[False,True])
def test_other_filter_variants_full_callers(candidate,alpha,disable,aol,tmp_path,host_heartbeat_bridge):
  advertisements=[(0x2FF,bytes.fromhex('00000409'),bus) for bus in (0,2)]
  helpers.run_case(candidate,alpha,disable,0,tmp_path,aol=aol,bridge=host_heartbeat_bridge,filter_advertisements=advertisements)


@pytest.mark.parametrize('alpha,disable', [(False, False), (True, False), (True, True)])
def test_camry_cluster_bus_advertisement_actual_callers(alpha, disable, tmp_path, host_heartbeat_bridge):
  from opendbc.car.toyota.cluster_radar import CLUSTERS
  advertisements = [(0x2FF, bytes.fromhex('00000409'), bus) for bus in (0, 2)]
  advertisements += [(0x2FF, bytes.fromhex('100068010000b5ff'), 1)]
  advertisements += [(address, bytes(8), 1) for address in CLUSTERS]
  helpers.run_case(CAR.TOYOTA_CAMRY, alpha, disable, 0, tmp_path, aol=True,
                   bridge=host_heartbeat_bridge, filter_advertisements=advertisements)
