"""Modified output stops on known invalid RX before the periodic safety tick."""
import pytest
from opendbc.car import structs
from opendbc.car.volvo.tests.test_cma import CMA, interface, packet
from opendbc.car.volvo.tests.test_cma_integrity import feed


@pytest.mark.parametrize('identity', CMA)
@pytest.mark.parametrize('address,checksum_byte', [(0x69, 6), (0x69, 2), (0x67, 2), (0x17, 0), (0x17, 3)])
def test_current_checksum_failure_withdraws_modified_commands_before_tick(identity, address, checksum_byte):
  lib, incoming = feed(identity)
  _, ci = interface(identity)
  ci.update([(1_000_000_000, incoming)])
  cc = structs.CarControl.new_message()
  cc.latActive = True
  _, output = ci.apply(cc.as_reader(), 1_000_000_000)
  command = next(msg for msg in output if msg[0] == 0x58)
  assert lib.get_controls_allowed()
  assert lib.safety_tx_hook(packet(command))
  address, payload, bus = next(msg for msg in incoming if msg[0] == address)
  changed = bytearray(payload)
  changed[checksum_byte] ^= 1
  assert not lib.safety_rx_hook(packet((address, bytes(changed), bus)))
  # Current upstream is_msg_valid withdraws permission immediately on invalid
  # RX. Require that safer contract without safety_tick or a forced flag.
  assert not lib.get_controls_allowed()
  assert not lib.safety_tx_hook(packet(command))
  original = next(msg for msg in incoming if msg[0] == 0x58)
  assert lib.safety_tx_hook(packet((original[0], original[1], 2)))
