import pytest
from opendbc.car.toyota import eps8_sdsu, rav4h_filter
from opendbc.car import gen_empty_fingerprint
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.starpilot.car.toyota.tests import test_eps8_sdsu_callers as eps8
from openpilot.starpilot.car.toyota.tests import test_rav4h_caller_native as helpers
from openpilot.starpilot.car.toyota.tests.test_toyota_cpp_transport import host_heartbeat_bridge as host_heartbeat_bridge

source_matched_native = helpers.source_matched_native
daemon_gc_scope = helpers.daemon_gc_scope
CARS = sorted(eps8_sdsu.CARS | rav4h_filter.EPS5_SDSU_CARS)


def run(candidate, stock, bridge, scenario='nominal'):
  if candidate in eps8_sdsu.CARS:
    eps8.run(candidate, stock, bridge, scenario, sdsu_mirror=True)
  else:
    helpers.run_case(stock, scenario, candidate=candidate, bridge=bridge, input_clock=True, sdsu_mirror=True)


@pytest.mark.parametrize('candidate', CARS)
@pytest.mark.parametrize('stock', [False, True])
def test_each_forwarded_sdsu_full_caller(candidate, stock, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge)


@pytest.mark.parametrize('candidate', [CAR.TOYOTA_RAV4H, CAR.TOYOTA_CAMRY, CAR.TOYOTA_COROLLA_TSS2, CAR.TOYOTA_RAV4_TSS2_2023])
@pytest.mark.parametrize('stock', [False, True])
@pytest.mark.parametrize('scenario', ['faults', 'off'])
def test_copy_cannot_mask_bus0_source_loss_or_saved_off(candidate, stock, scenario, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge, scenario)


@pytest.mark.parametrize('candidate', CARS)
@pytest.mark.parametrize('stock', [False, True])
def test_native_forwarding_keeps_existing_owner(candidate, stock):
  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = fp[2][0x2FF] = 8
  cp = CarInterface.get_params(candidate, fp, helpers.firmware(candidate), True, False, False)
  if stock:
    assert rav4h_filter.prepare_stock(cp)
  safety = libsafety_py.libsafety
  safety.init_tests()
  assert safety.set_safety_hooks(cp.safetyConfigs[0].safetyModel.raw, cp.safetyConfigs[0].safetyParam) == 0
  for bus, target in ((0, 2), (2, 0)):
    for address in (0x2FF, 0x343):
      assert safety.safety_fwd_hook(bus, address) == target
