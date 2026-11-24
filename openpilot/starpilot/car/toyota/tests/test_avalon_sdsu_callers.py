import pytest
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.car.toyota.tests import test_rav4h_caller_native as helpers
from openpilot.starpilot.car.toyota.tests.test_toyota_cpp_transport import host_heartbeat_bridge as host_heartbeat_bridge

source_matched_native=helpers.source_matched_native
daemon_gc_scope=helpers.daemon_gc_scope


@pytest.mark.parametrize('stock',[False,True])
@pytest.mark.parametrize('scenario',['physical','faults','bad_dlc','off','cold_low'])
def test_avalon_sdsu_actual_callers(stock,scenario,host_heartbeat_bridge):
  # Allow the 10Hz safety acknowledgement plus a producer cycle after entering the brake stop.
  helpers.run_case(stock,scenario,candidate=CAR.TOYOTA_AVALON,bridge=host_heartbeat_bridge,input_clock=True,brake_stop_check_start=355)


@pytest.mark.parametrize('stock',[False,True])
def test_avalon_ordinary_cruise_without_aol(stock,host_heartbeat_bridge):
  helpers.run_case(stock,candidate=CAR.TOYOTA_AVALON,aol=False,bridge=host_heartbeat_bridge,input_clock=True,brake_stop_check_start=355)
