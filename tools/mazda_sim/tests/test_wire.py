import socket

import pytest

from mazda_sim.common import wire


def test_can_roundtrip():
  frames = [wire.Frame(wire.NET_CAR, 0x202, bytes(range(8))), wire.Frame(wire.NET_AUX, 0x249, b"\x01\x02"),
            wire.Frame(wire.NET_CAM, 0x243, b"")]
  t, out = wire.unpack_can(wire.pack_can(123456789, frames))
  assert t == 123456789
  assert [(f.net, f.addr, bytes(f.dat)) for f in out] == [(f.net, f.addr, f.dat) for f in frames]


def test_sensors_and_frame_roundtrip():
  s = wire.Sensors(42, (0.5, -1.0, -9.81), (0.0, 0.01, -0.2), (37.0, -122.0, 5.0, 20.0, 90.0, 1.0, 19.0))
  r = wire.unpack_sensors(wire.pack_sensors(s))
  assert r.t_us == 42 and r.accel == pytest.approx(s.accel) and r.gyro == pytest.approx(s.gyro)
  assert r.gps == pytest.approx(s.gps)
  f = wire.CameraFrame(wire.CAM_WIDE, 7, 99, 4, 2, wire.CODEC_NV12, bytes(12))
  g = wire.unpack_frame(b"".join(wire.pack_frame(f)))
  assert (g.cam, g.frame_id, g.t_us, g.width, g.height, g.codec, g.data) == (f.cam, 7, 99, 4, 2, wire.CODEC_NV12, bytes(12))


def test_link_over_socket():
  a, b = socket.socketpair()
  la, lb = wire.Link(a), wire.Link(b)
  la.send(wire.MSG_HELLO, wire.pack_json({"role": "test"}))
  la.send(wire.MSG_CAN, wire.pack_can(1, [wire.Frame(0, 1, b"x")]))
  t, p = lb.recv()
  assert t == wire.MSG_HELLO and wire.unpack_json(p) == {"role": "test"}
  t, p = lb.recv()
  assert t == wire.MSG_CAN and wire.unpack_can(p)[1][0].dat == b"x"
  la.close()
  with pytest.raises(wire.ConnectionClosed):
    lb.recv()
