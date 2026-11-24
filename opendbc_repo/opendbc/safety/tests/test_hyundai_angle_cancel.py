import pytest

from opendbc.safety.tests import test_hyundai_stock_angle_aol as stock_fixture

WORDS = (*stock_fixture.WORDS, 0x4428, 0x4C28, 0x7829)


def cancel(case, word):
  case.rx('CRUISE_BUTTONS_ALT' if word & 32 else 'CRUISE_BUTTONS', 1 if word & 16 else 0,
          {'COUNTER': 9, 'CRUISE_BUTTONS': 4})


@pytest.mark.parametrize('word', WORDS)
def test_cancel_keeps_only_claimed_independent_angle_token(word):
  case = stock_fixture.TestHyundaiStockAngleAol(methodName='runTest')
  case.setUp()
  case.mode(word)
  case.sources(word)
  case.gesture(word)
  assert case.safety.aol_get_permission_mask() == 1
  cancel(case, word)
  assert not case.safety.get_controls_allowed()
  assert case.safety.aol_get_permission_mask() == 1
  assert case.safety.safety_tx_hook(case.command(word, 1))


@pytest.mark.parametrize('word', WORDS)
@pytest.mark.parametrize('prior', ['unclaimed', 'withdrawn'])
def test_cancel_cannot_claim_or_restore_angle_authority(word, prior):
  case = stock_fixture.TestHyundaiStockAngleAol(methodName='runTest')
  case.setUp()
  case.mode(word)
  case.sources(word)
  if prior == 'withdrawn':
    case.gesture(word)
    assert case.safety.aol_get_permission_mask() == 1
    case.safety.aol_set_host_request(0)
    case.safety.set_aol_test_heartbeat(False)
    assert case.safety.aol_get_permission_mask() == 0
  else:
    bus, name = (1 if word & 16 else 0), ('CRUISE_BUTTONS_ALT' if word & 32 else 'CRUISE_BUTTONS')
    case.rx(name, bus, {'COUNTER': 7, 'LDA_BTN': 1})
    case.rx(name, bus, {'COUNTER': 8, 'LDA_BTN': 0})
  cancel(case, word)
  case.safety.set_aol_test_heartbeat(True)
  case.safety.aol_set_host_request(1)
  assert case.safety.aol_get_permission_mask() == 0
  assert not case.safety.safety_tx_hook(case.command(word, 1))


@pytest.mark.parametrize('word', WORDS)
def test_cancel_cannot_renew_expired_physical_angle_sources(word):
  case = stock_fixture.TestHyundaiStockAngleAol(methodName='runTest')
  case.setUp()
  case.mode(word)
  case.sources(word)
  case.gesture(word)
  assert case.safety.aol_get_permission_mask() == 1
  case.safety.set_timer(1_300_001)
  assert case.safety.aol_get_permission_mask() == 0
  cancel(case, word)
  case.safety.set_aol_test_heartbeat(True)
  case.safety.aol_set_host_request(1)
  assert case.safety.aol_get_permission_mask() == 0
  assert not case.safety.safety_tx_hook(case.command(word, 1))
