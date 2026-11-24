import pytest
from opendbc.car.toyota import eps8_sdsu, rav4h_filter, legacy_sdsu
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.car.toyota.tests import test_rav4h_caller_native as helpers
from openpilot.starpilot.car.toyota.tests import test_eps8_sdsu_callers as eps8
from openpilot.starpilot.car.toyota.tests import test_legacy_sdsu_callers as legacy
from openpilot.starpilot.car.toyota.tests.test_toyota_cpp_transport import host_heartbeat_bridge as host_heartbeat_bridge

source_matched_native = helpers.source_matched_native
daemon_gc_scope = helpers.daemon_gc_scope
CARS = sorted(eps8_sdsu.CARS | rav4h_filter.EPS5_SDSU_CARS | legacy_sdsu.PROFILES.keys())


def run(candidate, stock, bridge, scenario='nominal'):
  if candidate in legacy_sdsu.PROFILES:
    legacy.run(candidate, stock, bridge, scenario, camera_messages=True)
  elif candidate in eps8_sdsu.CARS:
    eps8.run(candidate, stock, bridge, scenario, camera_messages=True)
  else:
    helpers.run_case(stock, scenario, candidate=candidate, bridge=bridge, input_clock=True, brake_stop_check_start=355, camera_messages=True)


@pytest.mark.parametrize('candidate', CARS)
@pytest.mark.parametrize('stock', [False, True])
def test_camera_messages_preserve_each_smartdsu_caller(candidate, stock, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge)


@pytest.mark.parametrize('candidate', [CAR.TOYOTA_RAV4H, CAR.TOYOTA_CAMRY, CAR.TOYOTA_COROLLA_TSS2, CAR.TOYOTA_RAV4_TSS2_2023, CAR.LEXUS_GS_F])
@pytest.mark.parametrize('stock', [False, True])
@pytest.mark.parametrize('scenario', ['faults', 'bad_dlc', 'off'])
def test_camera_messages_do_not_mask_status_loss_or_saved_off(candidate, stock, scenario, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge, scenario)
