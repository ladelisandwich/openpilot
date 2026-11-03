"""What openpilot's fingerprinting talks to: ISO-TP + UDS on each module, and the FW versions it must match."""
from mazda_sim.car.ecus import CX9_FW
from mazda_sim.car.isotp import DiagEndpoint, UdsServer
from mazda_sim.common.wire import NET_CAR, Frame

VIN = "JM3TCBDY4P0600001"


def request(ep: DiagEndpoint, payload: bytes, addr: int | None = None) -> bytes | None:
  """A tester's single-frame request; follows a multi-frame answer with flow control."""
  ep.on_frame(Frame(NET_CAR, addr or ep.rx_addr, bytes([len(payload)]) + payload + bytes(7 - len(payload))))
  out = ep.drain()
  if not out:
    return None
  first = out[0].dat
  assert out[0].addr == ep.rx_addr + 8
  if first[0] >> 4 == 0:
    return bytes(first[1:1 + (first[0] & 0xF)])
  assert first[0] >> 4 == 1
  n = ((first[0] & 0xF) << 8) | first[1]
  data = bytearray(first[2:8])
  ep.on_frame(Frame(NET_CAR, ep.rx_addr, bytes([0x30, 0, 0, 0, 0, 0, 0, 0])))
  for i, f in enumerate(ep.drain(), start=1):
    assert f.dat[0] == 0x20 | (i & 0xF)
    data += f.dat[1:]
  return bytes(data[:n])


def test_fw_version_read_multi_frame():
  addr, fw = CX9_FW["eps"]
  ep = DiagEndpoint(NET_CAR, addr, UdsServer(fw).handle)
  assert request(ep, bytes([0x22, 0xF1, 0x88])) == bytes([0x62, 0xF1, 0x88]) + fw


def test_vin_over_obd_functional_address():
  addr, fw = CX9_FW["engine"]
  ep = DiagEndpoint(NET_CAR, addr, UdsServer(fw, vin=VIN).handle, obd=True)
  assert request(ep, bytes([0x09, 0x02]), addr=0x7DF) == bytes([0x49, 0x02, 0x01]) + VIN.encode()
  assert request(ep, bytes([0x22, 0xF1, 0x90])) == bytes([0x62, 0xF1, 0x90]) + VIN.encode()


def test_session_refusal_and_unknown_did():
  ep = DiagEndpoint(NET_CAR, 0x764, UdsServer(b"x", on_session=lambda s: s != 0x02).handle)
  assert request(ep, bytes([0x10, 0x03]))[:2] == bytes([0x50, 0x03])
  assert request(ep, bytes([0x10, 0x02])) == bytes([0x7F, 0x10, 0x22])
  assert request(ep, bytes([0x22, 0x12, 0x34])) == bytes([0x7F, 0x22, 0x31])


def test_fw_versions_match_the_builds_fingerprints():
  from opendbc.car.mazda.fingerprints import FW_VERSIONS
  from opendbc.car.mazda.values import CAR
  table = {addr: versions for (_ecu, addr, _sub), versions in FW_VERSIONS[CAR.MAZDA_CX9_2021].items()}
  assert set(table) == {addr for addr, _fw in CX9_FW.values()}
  for name, (addr, fw) in CX9_FW.items():
    assert fw in table[addr], f"{name} at {addr:#x}: {fw!r} is not a CX-9 2021-23 version"
