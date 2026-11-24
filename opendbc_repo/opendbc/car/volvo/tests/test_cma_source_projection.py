"""Native source binding and opaque-bit roundtrip through the selected CI."""
import pytest
from opendbc.car import structs
from opendbc.car.volvo.tests.test_cma import CMA, interface, sources, packet
from opendbc.car.volvo.tests.test_cma_integrity import feed
from opendbc.car.volvo.helpers import (checksum_lca_2_message, checksum_2_0x69_message,
  checksum_lca_5_message, checksum_1_pscm_related_message, checksum_2_pscm_related_message)
from opendbc.safety.tests.libsafety import libsafety_py


def checksums(address, d):
  if address == 0x69:
    d[6] = checksum_lca_2_message(d[0], d[5])
    d[2] = checksum_2_0x69_message(d[0], d[1], d[3], d[4])
  elif address == 0x67:
    d[2] = checksum_lca_5_message(d[0], d[1], d[3], d[4], d[5])
  elif address == 0x17:
    d[0] = checksum_1_pscm_related_message(d[1], d[2])
    d[3] = checksum_2_pscm_related_message(d[2])


@pytest.mark.parametrize('identity', CMA)
@pytest.mark.parametrize('address,byte,mask', [(0x58, 6, 1), (0x57, 2, 1), (0x69, 5, 0x40),
  (0x90, 4, 1), (0x67, 0, 1), (0x97, 0, 1), (0x92, 0, 1), (0x16, 7, 1), (0x17, 6, 1)])
def test_recomputed_checksums_do_not_authorize_fabricated_stock_fields(identity, address, byte, mask):
  lib, incoming = feed(identity)
  address, data, source_bus = next(msg for msg in incoming if msg[0] == address)
  bus = 2 if source_bus == 0 else 0
  assert lib.safety_tx_hook(packet((address, data, bus)))
  changed = bytearray(data)
  changed[byte] ^= mask
  checksums(address, changed)
  assert not lib.safety_tx_hook(packet((address, bytes(changed), bus)))


@pytest.mark.parametrize('identity', CMA)
def test_active_native_permission_cannot_renew_missing_source(identity):
  lib, incoming = feed(identity)
  assert lib.get_controls_allowed()
  lib.set_timer(110001)
  for address, data, bus in incoming:
    if address in (0x58, 0x57, 0x69, 0x90, 0x67, 0x97, 0x92, 0x16, 0x17):
      assert not lib.safety_tx_hook(packet((address, data, 2 if bus == 0 else 0)))


@pytest.mark.parametrize('identity', CMA)
@pytest.mark.parametrize('active', [False, True])
def test_lca_opaque_bits_survive_real_parser_controller_and_native(identity, active):
  cp, ci = interface(identity)
  lib = libsafety_py.libsafety
  assert lib.set_safety_hooks(cp.safetyConfigs[0].safetyModel.raw, cp.safetyConfigs[0].safetyParam) == 0
  lib.init_tests()
  lib.set_timer(10000)
  incoming = []
  for address, data, bus in sources(identity):
    d = bytearray(data)
    if address == 0x58:
      d[0] |= 0x30
      d[2] |= 0xC0
      d[7] |= 0x0F
    incoming.append((address, bytes(d), bus))
    assert lib.safety_rx_hook(packet(incoming[-1]))
  lib.safety_tick()
  ci.update([(1_000_000_000, incoming)])
  cc = structs.CarControl.new_message()
  cc.latActive = active
  _, emitted = ci.apply(cc.as_reader(), 1_000_000_000)
  output = next(msg for msg in emitted if msg[0] == 0x58)
  assert (output[1][0] & 0x30, output[1][2] & 0xC0, output[1][7] & 0x0F) == (0x30, 0xC0, 0x0F)
  assert lib.safety_tx_hook(packet(output))
