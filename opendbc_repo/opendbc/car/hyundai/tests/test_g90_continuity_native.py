from types import SimpleNamespace
from unittest.mock import patch

import pytest

from opendbc.car import Bus, make_tester_present_msg
from opendbc.car.hyundai.tests.test_g90_continuity import prepared
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, DBC
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.test_hyundai import checksum
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.selfdrive.car.card import Car
from openpilot.starpilot.vehicle_startup import VehicleStartupOwner


@pytest.mark.parametrize('address,data', [(0x420, '501420c8b84b6a00'), (0x421, '000000ffe37f2db1')])
@pytest.mark.parametrize('bus', [0, 2])
def test_recorded_returned_stock_scc_keeps_native_relay_protection(address, data, bus):
  safety = libsafety_py.libsafety
  safety.set_alternative_experience(32)
  assert safety.set_safety_hooks(8, 0xC04) == 0
  safety.init_tests()
  safety.set_alternative_experience(32)
  packer = CANPackerSafety(DBC[CAR.GENESIS_G90][Bus.pt])
  for name, values, fix in (
    ('WHL_SPD11', {'WHL_SPD_AliveCounter_LSB': 0, 'WHL_SPD_AliveCounter_MSB': 0}, checksum),
    ('TCS13', {'AliveCounterTCS': 0}, checksum),
    ('MDPS12', {}, None),
    ('CLU11', {'CF_Clu_AliveCnt1': 0}, None),
    ('EMS16', {'CRUISE_LAMP_S': 0, 'AliveCounter': 0}, checksum),
    ('BCM_PO_11', {'LDA_BTN': 0}, None),
  ):
    assert safety.safety_rx_hook(packer.make_can_msg_safety(name, 0, values, fix_checksum=fix)), name
  safety.set_timer(1000)
  safety.safety_tick()
  assert safety.safety_config_valid()
  assert not safety.get_relay_malfunction()
  safety.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, bytes.fromhex(data)))
  assert bool(safety.get_relay_malfunction()) == (bus == 0)
  assert not safety.get_controls_allowed()
  assert safety.aol_get_permission_mask() == 0
  if bus == 0:
    tester = make_tester_present_msg(0x7d0, 0, suppress_response=True)
    assert not safety.safety_tx_hook(libsafety_py.make_CANPacket(tester[0], tester[2], tester[1]))


@pytest.mark.parametrize('receipt', ['valid', 'invalid', 'exception'])
def test_current_card_publisher_transfers_continuity_only_after_success(receipt):
  owner, now, sent = prepared()
  owner.published = True
  delivered = []
  card = Car.__new__(Car)
  card.CI = CarInterface(owner.cp)
  card.CP = owner.cp
  card.vehicle_startup = VehicleStartupOwner(owner)

  def send(service, packet):
    assert service == 'sendcan'
    assert not owner.handed_off
    if receipt == 'exception':
      raise OSError('Actual publication failed')
    delivered.append(packet)

  try:
    with patch.object(card, 'pm', SimpleNamespace(send=send), create=True), patch.object(card, 'volt_startup_keepalive', return_value=False):
      frames = [make_tester_present_msg(0x7d0, 0, suppress_response=True)]
      if receipt == 'exception':
        with pytest.raises(OSError, match='publication failed'):
          card.publish_sendcan(frames)
      else:
        card.publish_sendcan(frames, valid=receipt == 'valid')
      assert bool(delivered) == (receipt != 'exception')
      assert owner.handed_off == (receipt == 'valid')
      count = len(sent)
      now[0] += 1.0
      owner.keeper.tick()
      assert len(sent) == count + (receipt != 'valid')
  finally:
    owner.close()
