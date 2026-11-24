import pytest
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.car.toyota.tests.test_rav4_2025_filter_advert import observed
from openpilot.starpilot.car.toyota.tests import test_rav4_angle_filter_callers as helpers
from openpilot.starpilot.car.toyota.tests.test_toyota_cpp_transport import host_heartbeat_bridge as host_heartbeat_bridge

source_matched_native=helpers.source_matched_native
daemon_gc_scope=helpers.daemon_gc_scope


@pytest.mark.parametrize('alpha,disable,radar_mode',[(False,False,0),(True,False,0),(True,False,1),(True,False,2),(True,True,1)])
@pytest.mark.parametrize('aol',[False,True])
def test_recorded_2025_filter_actual_callers(alpha,disable,radar_mode,aol,tmp_path,host_heartbeat_bridge):
  fw,_=observed()
  advertisements=[(0x2FF,bytes.fromhex('00000409'),bus) for bus in (0,2)]
  helpers.run_case(CAR.TOYOTA_RAV4_TSS2_2023,alpha,disable,radar_mode,tmp_path,aol=aol,
    bridge=host_heartbeat_bridge,filter_advertisements=advertisements,recorded_firmware=fw)
