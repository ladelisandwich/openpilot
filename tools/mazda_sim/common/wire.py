"""The wire between the Mazda container and the comma container.

Stdlib only: the comma side imports this from inside whatever build is under test.

Two TCP links, mirroring what physically connects a comma 3X to the car:
  harness link (HARNESS_PORT): CAN frames, ignition/power, IMU/GPS, ground truth, state both ways
  optics link  (OPTICS_PORT):  road and wide camera frames, car -> comma only

Every message is: u32 length (of everything after it) | u8 type | payload.

CAN frames travel per *network*, not per panda bus: the car only knows which wires an ECU is on,
and the comma side (the virtual panda) decides what each of its transceivers sees, from the
harness relay and the bus-1 multiplexer it controls. See device/virtual_panda.py.
"""
from __future__ import annotations

import json
import socket
import struct
import threading
from dataclasses import dataclass

PROTO_VERSION = 1
HARNESS_PORT = 7000
OPTICS_PORT = 7001
CONTROL_PORT = 8770

# Networks the car side exposes. The numbers match the panda bus each one reaches when the relay
# is intercepting (and, for AUX, when bus 1 is switched to the harness OBD lines).
NET_CAR = 0  # powertrain/chassis CAN: engine, ABS, EPS, radar, cluster -> panda bus 0
NET_AUX = 1  # harness OBD-C lines: the Torque Interceptor lives here -> panda bus 1 (OBD mode only)
NET_CAM = 2  # Forward Sensing Camera side of the harness -> panda bus 2
NETS = (NET_CAR, NET_AUX, NET_CAM)

MSG_HELLO = 0x01      # json
MSG_CAN = 0x02        # packed frames
MSG_CAR_STATE = 0x03  # json, car -> comma: ignition, battery voltage, sim clock
MSG_PANDA = 0x04      # json, comma -> car: relay, bus-1 mux, safety mode, controls allowed
MSG_SENSORS = 0x05    # packed IMU/GPS, car -> comma
MSG_TRUTH = 0x06      # json, car -> comma: road ahead and lead, for the ground-truth model
MSG_OPENPILOT = 0x07  # json, comma -> car: what openpilot is doing, for the dashboard
MSG_FRAME = 0x08      # optics link: one camera frame
MSG_COMMAND = 0x09    # json, either way: reset, scenario changes

_HDR = struct.Struct(">IB")
_CAN_HDR = struct.Struct(">QH")
_FRAME_HDR = struct.Struct(">BIQHHB")
_SENSORS = struct.Struct(">Q3d3d7d")

CAM_ROAD = 0
CAM_WIDE = 1
CODEC_NV12 = 0
CODEC_JPEG = 1


@dataclass(frozen=True, slots=True)
class Frame:
  net: int
  addr: int
  dat: bytes


def pack_can(t_us: int, frames: list[Frame]) -> bytes:
  parts = [_CAN_HDR.pack(t_us, len(frames))]
  for f in frames:
    parts.append(struct.pack(">BIB", f.net, f.addr, len(f.dat)))
    parts.append(f.dat)
  return b"".join(parts)


def unpack_can(payload: bytes) -> tuple[int, list[Frame]]:
  t_us, n = _CAN_HDR.unpack_from(payload, 0)
  off = _CAN_HDR.size
  frames = []
  for _ in range(n):
    net, addr, ln = struct.unpack_from(">BIB", payload, off)
    off += 6
    frames.append(Frame(net, addr, bytes(payload[off:off + ln])))
    off += ln
  return t_us, frames


@dataclass(slots=True)
class Sensors:
  t_us: int
  accel: tuple[float, float, float]   # m/s^2, device frame (x forward, y right, z down), gravity included
  gyro: tuple[float, float, float]    # rad/s, device frame
  # lat, lon, alt, speed, bearing_deg, v_north, v_east
  gps: tuple[float, float, float, float, float, float, float]


def pack_sensors(s: Sensors) -> bytes:
  return _SENSORS.pack(s.t_us, *s.accel, *s.gyro, *s.gps)


def unpack_sensors(payload: bytes) -> Sensors:
  v = _SENSORS.unpack(payload)
  return Sensors(v[0], tuple(v[1:4]), tuple(v[4:7]), tuple(v[7:14]))


@dataclass(slots=True)
class CameraFrame:
  cam: int
  frame_id: int
  t_us: int
  width: int
  height: int
  codec: int
  data: bytes


def pack_frame(f: CameraFrame) -> list[bytes]:
  # returned as parts so a 3.5 MB NV12 frame is never copied into a new buffer
  return [_FRAME_HDR.pack(f.cam, f.frame_id, f.t_us, f.width, f.height, f.codec), f.data]


def unpack_frame(payload: memoryview | bytes) -> CameraFrame:
  cam, frame_id, t_us, w, h, codec = _FRAME_HDR.unpack_from(payload, 0)
  return CameraFrame(cam, frame_id, t_us, w, h, codec, bytes(payload[_FRAME_HDR.size:]))


def pack_json(obj) -> bytes:
  return json.dumps(obj, separators=(",", ":")).encode()


def unpack_json(payload: bytes):
  return json.loads(payload)


class ConnectionClosed(Exception):
  pass


class Link:
  """One framed TCP connection. send() is thread-safe; recv() must be called from one thread."""

  def __init__(self, sock: socket.socket):
    self.sock = sock
    self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    self._send_lock = threading.Lock()
    self.closed = False

  def send(self, msg_type: int, *parts: bytes) -> None:
    length = 1 + sum(len(p) for p in parts)
    with self._send_lock:
      try:
        self.sock.sendall(_HDR.pack(length, msg_type))
        for p in parts:
          self.sock.sendall(p)
      except OSError as e:
        self.closed = True
        raise ConnectionClosed(str(e)) from e

  def _recv_exact(self, n: int) -> bytearray:
    buf = bytearray(n)
    view = memoryview(buf)
    got = 0
    while got < n:
      try:
        r = self.sock.recv_into(view[got:], n - got)
      except OSError as e:
        self.closed = True
        raise ConnectionClosed(str(e)) from e
      if r == 0:
        self.closed = True
        raise ConnectionClosed("peer closed")
      got += r
    return buf

  def recv(self) -> tuple[int, bytearray]:
    length, msg_type = _HDR.unpack(self._recv_exact(_HDR.size))
    return msg_type, self._recv_exact(length - 1)

  def close(self) -> None:
    self.closed = True
    try:
      self.sock.shutdown(socket.SHUT_RDWR)
    except OSError:
      pass
    self.sock.close()


def connect(host: str, port: int, timeout: float = 2.0) -> Link:
  sock = socket.create_connection((host, port), timeout=timeout)
  sock.settimeout(None)
  return Link(sock)


def listen(port: int, host: str = "0.0.0.0") -> socket.socket:
  srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  srv.bind((host, port))
  srv.listen(4)
  return srv
