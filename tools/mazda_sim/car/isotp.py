"""ISO-TP transport and the UDS / OBD services the car's modules answer.

Enough for openpilot's VIN read, ECU presence probe and FW version query to fingerprint the car exactly as a
real CX-9 does, and for the radar's diagnostic sessions that radar emulation and hybrid long depend on.
"""
from __future__ import annotations

from collections.abc import Callable

from ..common.wire import Frame

OBD_FUNCTIONAL_ADDR = 0x7DF

SID_SESSION = 0x10
SID_COMM_CONTROL = 0x28
SID_TESTER_PRESENT = 0x3E
SID_READ_DID = 0x22
OBD_MODE_VEHICLE_INFO = 0x09

NRC_SERVICE_NOT_SUPPORTED = 0x11
NRC_SUBFUNCTION_NOT_SUPPORTED = 0x12
NRC_INCORRECT_LENGTH = 0x13
NRC_OUT_OF_RANGE = 0x31

DID_VIN = 0xF190
DID_ECU_SOFTWARE_NUMBER = 0xF188   # what Mazda's FW query reads
DID_UDS_VERSION = 0xF180


class DiagEndpoint:
  """One module's diagnostic address: reassembles requests, segments responses.

  handler(request: bytes, functional: bool) -> bytes | None is the UDS layer; None means no response.
  """

  def __init__(self, net: int, rx_addr: int, handler: Callable[[bytes, bool], bytes | None], obd: bool = False,
               pad: int = 0x00):
    self.net = net
    self.rx_addr = rx_addr
    self.tx_addr = rx_addr + 8
    self.handler = handler
    self.obd = obd  # also answers functional OBD requests on 0x7DF
    self.pad = pad
    self.outbox: list[Frame] = []
    self._rx_buf = bytearray()
    self._rx_len = 0
    self._rx_seq = 0
    self._tx_pending = b""
    self._tx_seq = 0

  def _frame(self, payload: bytes) -> Frame:
    return Frame(self.net, self.tx_addr, bytes(payload) + bytes([self.pad]) * (8 - len(payload)))

  def on_frame(self, f: Frame) -> bool:
    """Returns True when the frame was addressed to this endpoint."""
    functional = f.addr == OBD_FUNCTIONAL_ADDR and self.obd
    if f.addr != self.rx_addr and not functional:
      return False
    if not f.dat:
      return True
    pci = f.dat[0] >> 4
    if pci == 0:  # single frame
      ln = f.dat[0] & 0x0F
      self._respond(bytes(f.dat[1:1 + ln]), functional)
    elif pci == 1 and not functional:  # first frame of a long request
      self._rx_len = ((f.dat[0] & 0x0F) << 8) | f.dat[1]
      self._rx_buf = bytearray(f.dat[2:8])
      self._rx_seq = 1
      self.outbox.append(self._frame(bytes([0x30, 0x00, 0x00])))
    elif pci == 2 and self._rx_len:
      if (f.dat[0] & 0x0F) != self._rx_seq:
        self._rx_len = 0
        return True
      self._rx_seq = (self._rx_seq + 1) & 0x0F
      self._rx_buf += f.dat[1:8]
      if len(self._rx_buf) >= self._rx_len:
        req = bytes(self._rx_buf[:self._rx_len])
        self._rx_len = 0
        self._respond(req, False)
    elif pci == 3 and self._tx_pending:  # flow control for our multi-frame response; STmin 0 assumed
      while self._tx_pending:
        chunk, self._tx_pending = self._tx_pending[:7], self._tx_pending[7:]
        self.outbox.append(self._frame(bytes([0x20 | self._tx_seq]) + chunk))
        self._tx_seq = (self._tx_seq + 1) & 0x0F
    return True

  def _respond(self, req: bytes, functional: bool) -> None:
    resp = self.handler(req, functional)
    if resp is None:
      return
    if len(resp) <= 7:
      self.outbox.append(self._frame(bytes([len(resp)]) + resp))
    else:
      self.outbox.append(self._frame(bytes([0x10 | (len(resp) >> 8), len(resp) & 0xFF]) + resp[:6]))
      self._tx_pending = resp[6:]
      self._tx_seq = 1

  def drain(self) -> list[Frame]:
    out, self.outbox = self.outbox, []
    return out


def negative(sid: int, nrc: int) -> bytes:
  return bytes([0x7F, sid, nrc])


class UdsServer:
  """The services every module on this car answers, with per-module identity and session hooks."""

  def __init__(self, fw_version: bytes, vin: str | None = None, on_session: Callable[[int], bool] | None = None,
               on_comm_control: Callable[[int, int], None] | None = None):
    self.fw_version = fw_version
    self.vin = vin
    self.session = 0x01
    self.on_session = on_session        # returns False to refuse a session change
    self.on_comm_control = on_comm_control
    self.last_request_age = 0.0         # seconds since the last request; drives S3 session timeout

  def handle(self, req: bytes, functional: bool) -> bytes | None:
    if not req:
      return None
    sid = req[0]
    self.last_request_age = 0.0

    if sid == OBD_MODE_VEHICLE_INFO:
      if len(req) >= 2 and req[1] == 0x02 and self.vin:
        return bytes([0x49, 0x02, 0x01]) + self.vin.encode()
      return None if functional else negative(sid, NRC_OUT_OF_RANGE)

    if sid == SID_TESTER_PRESENT:
      sub = req[1] if len(req) > 1 else 0
      return None if sub & 0x80 else bytes([0x7E, sub & 0x7F])

    if sid == SID_SESSION:
      if len(req) < 2:
        return negative(sid, NRC_INCORRECT_LENGTH)
      sub = req[1] & 0x7F
      if sub not in (0x01, 0x02, 0x03):
        return negative(sid, NRC_SUBFUNCTION_NOT_SUPPORTED)
      if self.on_session is not None and not self.on_session(sub):
        return negative(sid, 0x22)  # conditionsNotCorrect
      self.session = sub
      if req[1] & 0x80:
        return None
      return bytes([0x50, sub, 0x00, 0x32, 0x01, 0xF4])

    if sid == SID_COMM_CONTROL:
      if len(req) < 3:
        return negative(sid, NRC_INCORRECT_LENGTH)
      if self.on_comm_control is not None:
        self.on_comm_control(req[1] & 0x7F, req[2])
      return None if req[1] & 0x80 else bytes([0x68, req[1] & 0x7F])

    if sid == SID_READ_DID:
      if len(req) < 3:
        return negative(sid, NRC_INCORRECT_LENGTH)
      did = (req[1] << 8) | req[2]
      if did == DID_ECU_SOFTWARE_NUMBER:
        return bytes([0x62, req[1], req[2]]) + self.fw_version
      if did == DID_VIN and self.vin:
        return bytes([0x62, req[1], req[2]]) + self.vin.encode()
      return None if functional else negative(sid, NRC_OUT_OF_RANGE)

    return None if functional else negative(sid, NRC_SERVICE_NOT_SUPPORTED)
