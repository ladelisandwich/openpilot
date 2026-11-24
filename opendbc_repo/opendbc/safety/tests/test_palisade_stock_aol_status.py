"""Palisade route01 stock E-CAN status never owns the isolated steering bus."""
import pytest
from opendbc.car.structs import CarParams
from opendbc.safety.tests import common
from opendbc.safety.tests.libsafety import libsafety_py


@pytest.mark.parametrize('experience', [0, 32])
@pytest.mark.parametrize('steering_address,length', [(0x50, 16), (0x2A4, 24)])
def test_captured_ecan_status_cannot_fault_stock_or_arm_lateral(experience, steering_address, length):
  safety = libsafety_py.libsafety
  safety.set_alternative_experience(experience)
  try:
    assert safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0x2010) == 0
    safety.init_tests()
    safety.set_timer(2_000_000)
    # Exact route01 status bytes are ordinary traffic on the classic PT bus.
    for payload in ('5840020000000000', '5030020000000000'):
      safety.safety_rx_hook(common.make_msg(1, 0x485, dat=bytes.fromhex(payload)))
      assert not safety.get_relay_malfunction()
      assert not safety.get_controls_allowed()
      assert safety.aol_get_permission_mask() == 0
    # The isolated steering/suppression bus retains its real relay shutdown.
    safety.safety_rx_hook(common.make_msg(0, steering_address, length))
    assert safety.get_relay_malfunction()
    assert not safety.safety_tx_hook(common.make_msg(0, 0x50, 16))
  finally:
    safety.set_alternative_experience(0)
