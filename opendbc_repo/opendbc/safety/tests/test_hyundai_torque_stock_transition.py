import pytest

from opendbc.car.hyundai.canfd_stock_aol import STOCK_AOL_WORDS
from opendbc.safety.tests import test_hyundai_shared_canfd_stock_aol as fixture


def rx(case, name, bus, counter, values):
  maximum = 16 if name == 'CRUISE_BUTTONS' else 256
  frame = case.packer.make_can_msg(name, bus, dict(values, COUNTER=counter % maximum))
  assert case.safety.safety_rx_hook(case.packet(frame))


def steering(case, word, counter):
  name = 'LKAS_ALT' if word & 0x80 else 'LKAS' if word & 0x10 else 'LFA'
  frame = case.packer.make_can_msg(name, 0, {'COUNTER': counter, 'StrTqReqVal': 1,
                                           'ActToiSta': 1, 'LKA_OptUsmSta': 2, 'Damping_Gain': 100})
  return case.packet(frame)


@pytest.mark.parametrize('word', sorted(STOCK_AOL_WORDS))
@pytest.mark.parametrize('gesture', ['cancel', 'brake'])
@pytest.mark.parametrize('veto', ['host_withdrawal', 'bad_source_checksum'])
def test_actual_stock_torque_brake_and_cancel_keep_only_existing_lateral_permission(word, gesture, veto):
  case = fixture.TestHyundaiSharedCanfdStockAol(methodName='runTest')
  case.setUp()
  case.mode(word)
  case.healthy(word)
  case.safety.set_aol_test_heartbeat(True)
  case.safety.aol_set_host_request(1)
  assert not case.safety.get_controls_allowed()
  assert case.safety.aol_get_permission_mask() == 0
  assert not case.safety.safety_tx_hook(steering(case, word, 0))
  case.gesture(word)
  case.safety.aol_set_host_request(1)
  assert case.safety.aol_get_permission_mask() == 1
  pt = 1 if word & 0x10 else 0
  buttons = 'CRUISE_BUTTONS_ALT' if word & 0x20 else 'CRUISE_BUTTONS'
  for counter, values in ((9, {'ADAPTIVE_CRUISE_MAIN_BTN': 1}),
                          (10, {}), (11, {'CRUISE_BUTTONS': 2}), (12, {})):
    rx(case, buttons, pt, counter, values)
  scc = 2 if word & 8 else pt
  rx(case, 'SCC_CONTROL', scc, 7, {'MainMode_ACC': 1, 'ACCMode': 1, 'VSetDis': 90})
  assert case.safety.get_controls_allowed()
  case.safety.aol_set_host_request(3)
  assert case.safety.aol_get_permission_mask() == 1
  case.safety.aol_set_host_request(1)
  if gesture == 'cancel':
    rx(case, buttons, pt, 13, {'CRUISE_BUTTONS': 4})
  else:
    rx(case, 'TCS', pt, 7, {'DriverBraking': 1})
  assert bool(case.safety.get_controls_allowed()) == (gesture == 'cancel')
  assert case.safety.aol_get_permission_mask() == 1
  assert case.safety.safety_tx_hook(steering(case, word, 1))
  rx(case, 'SCC_CONTROL', scc, 8, {'MainMode_ACC': 1, 'ACCMode': 0, 'VSetDis': 90})
  assert not case.safety.get_controls_allowed()
  assert case.safety.aol_get_permission_mask() == 1
  assert case.safety.safety_tx_hook(steering(case, word, 2))
  assert not case.safety.safety_tx_hook(case.packet(case.packer.make_can_msg('SCC_CONTROL', pt, {'COUNTER': 9})))
  if veto == 'host_withdrawal':
    case.safety.aol_set_host_request(0)
  else:
    address, raw, bus = case.packer.make_can_msg('TCS', pt, {'COUNTER': 8, 'DriverBraking': int(gesture == 'brake')})
    corrupt = bytearray(raw)
    corrupt[0] ^= 1
    assert not case.safety.safety_rx_hook(case.packet((address, bytes(corrupt), bus)))
  assert case.safety.aol_get_permission_mask() == 0
  assert not case.safety.safety_tx_hook(steering(case, word, 3))
