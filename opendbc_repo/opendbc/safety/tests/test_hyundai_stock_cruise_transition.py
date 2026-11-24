import pytest

from opendbc.safety.tests import test_hyundai_angle_cancel as cancel_fixture
from opendbc.safety.tests import test_hyundai_stock_angle_aol as stock_fixture


@pytest.mark.parametrize('word', cancel_fixture.WORDS)
@pytest.mark.parametrize('gesture', ['cancel', 'brake'])
def test_real_stock_cruise_transition_preserves_claimed_lateral_only(word, gesture):
  case = stock_fixture.TestHyundaiStockAngleAol(methodName='runTest')
  case.setUp()
  case.mode(word)
  case.sources(word)
  case.gesture(word)
  case.safety.aol_set_host_request(1)
  assert case.safety.aol_get_permission_mask() == 1
  pt = 1 if word & 16 else 0
  button = 'CRUISE_BUTTONS_ALT' if word & 32 else 'CRUISE_BUTTONS'
  for counter, values in ((9, {'ADAPTIVE_CRUISE_MAIN_BTN': 1}),
                          (10, {}), (11, {'CRUISE_BUTTONS': 2}), (12, {})):
    case.rx(button, pt, dict(values, COUNTER=counter))
  scc_bus = 2 if word & 8 else pt
  case.rx('SCC_CONTROL', scc_bus, {'COUNTER': 7, 'MainMode_ACC': 1, 'ACCMode': 1, 'VSetDis': 90})
  assert case.safety.get_controls_allowed()
  assert case.safety.aol_get_permission_mask() == 1
  if gesture == 'cancel':
    case.rx(button, pt, {'COUNTER': 13, 'CRUISE_BUTTONS': 4})
  else:
    case.rx('TCS', pt, {'COUNTER': 7, 'DriverBraking': 1})
  assert bool(case.safety.get_controls_allowed()) == (gesture == 'cancel')
  assert case.safety.aol_get_permission_mask() == 1
  assert case.safety.safety_tx_hook(case.command(word, 1))
  case.rx('SCC_CONTROL', scc_bus, {'COUNTER': 8, 'MainMode_ACC': 1, 'ACCMode': 0, 'VSetDis': 90})
  assert not case.safety.get_controls_allowed()
  assert case.safety.aol_get_permission_mask() == 1
  assert case.safety.safety_tx_hook(case.command(word, 2))
