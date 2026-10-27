#!/usr/bin/env python3
"""The comma's cameras: frames from the simulated world into VisionIPC, in place of camerad.

Buffers use the same NV12 layout camerad gives modeld on the device (VENUS: 128-byte stride, padded planes), so
the driving model's warp reads them exactly as on the car. With no rendered world (lite world + ground-truth
model) a neutral road-coloured frame is served at 20 Hz so modeld and the UI run as usual.
"""
from __future__ import annotations

import argparse
import os
import threading
import time

import numpy as np

import cereal.messaging as messaging
from msgq.visionipc import VisionIpcServer, VisionStreamType

from ..common import wire

W, H = 1928, 1208
FPS = 20


def align(v: int, a: int) -> int:
  return ((v + a - 1) // a) * a


def nv12_layout(w: int, h: int) -> tuple[int, int, int, int]:
  """(stride, y_height, uv_height, size), as system/camerad/cameras/nv12_info.py."""
  stride = align(w, 128)
  y_height = align(h, 32)
  uv_height = align(h // 2, 16)
  size = stride * y_height + stride * uv_height + 4096 + max(16 * 1024, 8 * stride)
  size = align(size, 4096)
  size += align(w, 512) * 512
  return stride, y_height, uv_height, align(size, 4096)


class Camera:
  def __init__(self, server: VisionIpcServer, stream: int, state_name: str):
    self.stream = stream
    self.state_name = state_name
    self.stride, self.y_h, self.uv_h, self.size = nv12_layout(W, H)
    server.create_buffers_with_sizes(stream, 5, W, H, self.size, self.stride, self.stride * self.y_h)
    self.buf = np.zeros(self.size, dtype=np.uint8)
    self.y = self.buf[:self.stride * self.y_h].reshape(self.y_h, self.stride)
    uv_off = self.stride * self.y_h
    self.uv = self.buf[uv_off:uv_off + self.stride * self.uv_h].reshape(self.uv_h, self.stride)
    self.frame_id = 0
    self.fill_placeholder()

  def fill_placeholder(self) -> None:
    # grey sky over darker road, so the onroad view is not a black hole
    self.y[:H // 2, :W] = 150
    self.y[H // 2:H, :W] = 70
    self.uv[:H // 2, :W] = 128

  def load_packed_nv12(self, data: bytes) -> None:
    packed = np.frombuffer(data, dtype=np.uint8)
    if packed.size != W * H * 3 // 2:
      return
    self.y[:H, :W] = packed[:W * H].reshape(H, W)
    self.uv[:H // 2, :W] = packed[W * H:].reshape(H // 2, W)


class Optics:
  def __init__(self, car_host: str, dual: bool):
    self.car_host = car_host
    self.server = VisionIpcServer("camerad")
    self.road = Camera(self.server, VisionStreamType.VISION_STREAM_ROAD, "roadCameraState")
    self.wide = Camera(self.server, VisionStreamType.VISION_STREAM_WIDE_ROAD, "wideRoadCameraState") if dual else None
    self.server.start_listener()
    self.pm = messaging.PubMaster(['roadCameraState', 'wideRoadCameraState'])
    self.lock = threading.Lock()
    self.last_frame_t = 0.0
    self.pending: dict[int, wire.CameraFrame] = {}

  def receiver(self) -> None:
    while True:
      try:
        link = wire.connect(self.car_host, wire.OPTICS_PORT)
        link.send(wire.MSG_HELLO, wire.pack_json({"role": "comma-optics", "proto": wire.PROTO_VERSION}))
        print(f"opticsd: receiving frames from {self.car_host}:{wire.OPTICS_PORT}")
        while True:
          msg_type, payload = link.recv()
          if msg_type != wire.MSG_FRAME:
            continue
          f = wire.unpack_frame(payload)
          if f.codec == wire.CODEC_JPEG:
            import cv2
            bgr = cv2.imdecode(np.frombuffer(f.data, np.uint8), cv2.IMREAD_COLOR)
            f.data = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420).tobytes()
            f.data = i420_to_nv12(f.data, W, H)
          with self.lock:
            self.pending[f.cam] = f
            self.last_frame_t = time.monotonic()
      except (OSError, wire.ConnectionClosed):
        time.sleep(1.0)

  def send(self, cam: Camera, frame_id: int, t_ns: int) -> None:
    self.server.send(cam.stream, cam.buf.data, frame_id, t_ns, t_ns)
    dat = messaging.new_message(cam.state_name, valid=True)
    st = getattr(dat, cam.state_name)
    st.frameId = frame_id
    st.timestampSof = t_ns
    st.timestampEof = t_ns
    st.transform = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    self.pm.send(cam.state_name, dat)

  def run(self) -> None:
    threading.Thread(target=self.receiver, daemon=True).start()
    frame_id = 0
    next_t = time.monotonic()
    while True:
      next_t += 1.0 / FPS
      with self.lock:
        frames, self.pending = self.pending, {}
        live = time.monotonic() - self.last_frame_t < 0.5
      if wire.CAM_ROAD in frames:
        self.road.load_packed_nv12(frames[wire.CAM_ROAD].data)
      if self.wide is not None and wire.CAM_WIDE in frames:
        self.wide.load_packed_nv12(frames[wire.CAM_WIDE].data)
      if not live and frame_id % 40 == 0:
        self.road.fill_placeholder()
        if self.wide is not None:
          self.wide.fill_placeholder()
      t_ns = time.monotonic_ns()
      self.send(self.road, frame_id, t_ns)
      if self.wide is not None:
        self.send(self.wide, frame_id, t_ns)
      frame_id += 1
      time.sleep(max(0.0, next_t - time.monotonic()))


def i420_to_nv12(i420: bytes, w: int, h: int) -> bytes:
  a = np.frombuffer(i420, np.uint8)
  y = a[:w * h]
  u = a[w * h:w * h + w * h // 4].reshape(h // 2, w // 2)
  v = a[w * h + w * h // 4:].reshape(h // 2, w // 2)
  uv = np.empty((h // 2, w), np.uint8)
  uv[:, 0::2] = u
  uv[:, 1::2] = v
  return y.tobytes() + uv.tobytes()


def main() -> None:
  from ..common.config import SimConfig
  ap = argparse.ArgumentParser(description="camera frames from the Mazda simulator into VisionIPC")
  ap.add_argument("--car", default=os.environ.get("MAZDA_SIM_CAR_HOST", "mazda"))
  args, _ = ap.parse_known_args()
  Optics(args.car, SimConfig.load().world.dual_camera).run()


if __name__ == "__main__":
  main()
