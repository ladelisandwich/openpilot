"""MetaDrive as the world's renderer: what the comma's cameras see, and a chase view for the dashboard.

MetaDrive only draws. The car's motion comes from car/vehicle.py; each frame the renderer process places the car
(and the lead) at the simulated pose and renders. The MetaDrive map is built from the same blocks as car/road.py,
so the road in the pictures is the road the ground truth and the physics use.

The renderer runs in its own process, which also serves the optics link (frames go straight to the comma without
crossing process boundaries). It renders through GLX on $DISPLAY: a desktop with a GPU when run natively, Xvfb
with Mesa's llvmpipe in the container, which manages a few frames a second (MetaDrive's terrain shader is the
cost; its PBR pipeline does not start on panda3d's EGL display).
"""
from __future__ import annotations

import math
import multiprocessing as mp
import queue
import threading
import time

import cv2
import numpy as np

from ..common import wire
from ..common.config import SimConfig
from .road import Road, Straight

W, H = 1928, 1208
CHASE_W, CHASE_H = 960, 540
FPS = 20
DEVICE_AHEAD = 0.8
DEVICE_HEIGHT = 1.22


def metadrive_blocks(road: Road) -> list:
  """PG_MAP_FILE block configs for MetaDrive. Two differences from our blocks: a MetaDrive curve block is a bend
  followed by a straight ("length" is that straight), and its radius is the rightmost lane's center line, where ours
  is the road's left edge."""
  right_center = (road.lanes - 0.5) * road.lane_width
  out: list = [None]   # MetaDrive's own entry block
  blocks = list(road.blocks)
  i = 0
  while i < len(blocks):
    b = blocks[i]
    i += 1
    if isinstance(b, Straight):
      out.append({"id": "S", "pre_block_socket_index": 0, "length": b.length})
      continue
    follow = 0.01
    if i < len(blocks) and isinstance(blocks[i], Straight):
      follow = blocks[i].length
      i += 1
    radius = b.radius + right_center if b.left else b.radius - right_center
    out.append({"id": "C", "pre_block_socket_index": 0, "length": follow, "radius": radius, "angle": b.angle_deg,
                "dir": 0 if b.left else 1})
  return out


def bgr_to_nv12(bgr: np.ndarray) -> bytes:
  h, w = bgr.shape[:2]
  i420 = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420).reshape(-1)
  y = i420[:w * h]
  u = i420[w * h:w * h + w * h // 4].reshape(h // 2, w // 2)
  v = i420[w * h + w * h // 4:].reshape(h // 2, w // 2)
  uv = np.empty((h // 2, w), np.uint8)
  uv[:, 0::2] = u
  uv[:, 1::2] = v
  return y.tobytes() + uv.tobytes()


def _box(loader, render, size: tuple[float, float, float], color: tuple[float, float, float, float]):
  """A car-sized box (x width, y length, z height) on the ground, parented to render. No asset files needed."""
  from panda3d.core import ColorAttrib, Geom, GeomNode, GeomTriangles, GeomVertexData, GeomVertexFormat, GeomVertexWriter
  sx, sy, sz = size[0] / 2, size[1] / 2, size[2]
  fmt = GeomVertexFormat.getV3n3c4()
  vdata = GeomVertexData("box", fmt, Geom.UHStatic)
  vw, nw, cw = GeomVertexWriter(vdata, "vertex"), GeomVertexWriter(vdata, "normal"), GeomVertexWriter(vdata, "color")
  tris = GeomTriangles(Geom.UHStatic)
  faces = [((0, 0, 1), [(-sx, -sy, sz), (sx, -sy, sz), (sx, sy, sz), (-sx, sy, sz)]),
           ((0, -1, 0), [(-sx, -sy, 0), (sx, -sy, 0), (sx, -sy, sz), (-sx, -sy, sz)]),
           ((0, 1, 0), [(sx, sy, 0), (-sx, sy, 0), (-sx, sy, sz), (sx, sy, sz)]),
           ((-1, 0, 0), [(-sx, sy, 0), (-sx, -sy, 0), (-sx, -sy, sz), (-sx, sy, sz)]),
           ((1, 0, 0), [(sx, -sy, 0), (sx, sy, 0), (sx, sy, sz), (sx, -sy, sz)])]
  i = 0
  for n, quad in faces:
    shade = 1.0 if n[2] else 0.75
    for p in quad:
      vw.addData3(*p)
      nw.addData3(*n)
      cw.addData4(color[0] * shade, color[1] * shade, color[2] * shade, color[3])
    tris.addVertices(i, i + 1, i + 2)
    tris.addVertices(i, i + 2, i + 3)
    i += 4
  geom = Geom(vdata)
  geom.addPrimitive(tris)
  node = GeomNode("box")
  node.addGeom(geom)
  box = render.attachNewNode(node)
  # shading is baked into the vertex colours; MetaDrive's lights and shaders would otherwise draw it black
  box.setShaderOff(1)
  box.setLightOff(1)
  box.setAttrib(ColorAttrib.makeVertex(), 1)
  # one-sided: from the comma's seat inside the box every face is a back face, so the cameras see out through it
  return box


def _renderer(cfg_dict: dict, poses: mp.Queue, chase: mp.Queue, status: mp.Queue, port: int) -> None:
  from metadrive.component.map.pg_map import MapGenerateMethod
  from metadrive.component.sensors.rgb_camera import RGBCamera
  from metadrive.envs.metadrive_env import MetaDriveEnv
  from panda3d.core import Vec3

  cfg = SimConfig.from_dict(cfg_dict)
  road = Road.preset(cfg.world.track, cfg.world.lanes, cfg.world.lane_width)

  class RoadCam(RGBCamera):
    def __init__(self, *a, **k):
      super().__init__(*a, **k)
      self.get_lens().setFov(40)
      self.get_lens().setNear(0.1)

  class WideCam(RGBCamera):
    def __init__(self, *a, **k):
      super().__init__(*a, **k)
      self.get_lens().setFov(120)
      self.get_lens().setNear(0.1)

  class ChaseCam(RGBCamera):
    def __init__(self, *a, **k):
      super().__init__(*a, **k)
      self.get_lens().setFov(70)
      self.get_lens().setNear(0.3)

  scale = min(1.0, max(0.25, cfg.world.render_scale))
  rw, rh = int(W * scale) // 2 * 2, int(H * scale) // 2 * 2
  sensors = {"road": (RoadCam, rw, rh), "chase": (ChaseCam, CHASE_W, CHASE_H)}
  if cfg.world.dual_camera:
    sensors["wide"] = (WideCam, rw, rh)
  env = MetaDriveEnv(dict(
    use_render=False, image_observation=True, sensors=sensors, window_size=(160, 100),
    vehicle_config=dict(image_source="road", render_vehicle=False, show_navi_mark=False, show_dest_mark=False,
                        show_line_to_dest=False, show_line_to_navi_mark=False, show_lidar=False, show_side_detector=False,
                        show_lane_line_detector=False),
    interface_panel=[], show_logo=False, show_fps=False, traffic_density=0.0, preload_models=False,
    out_of_route_done=False, on_continuous_line_done=False, crash_vehicle_done=False, crash_object_done=False,
    horizon=None, anisotropic_filtering=False,
    map_config=dict(type=MapGenerateMethod.PG_MAP_FILE, lane_num=road.lanes, lane_width=road.lane_width,
                    config=metadrive_blocks(road)),
  ))
  env.reset()
  engine = env.engine
  agent = env.agent

  # our road frame -> MetaDrive: our first block starts after MetaDrive's entry block, at the left edge of our lanes
  first = env.current_map.blocks[1]
  lanes = [ln for d in first.block_network.graph.values() for ls in d.values() for ln in ls
           if abs(math.cos(ln.heading_theta_at(0))) > 0.99 and math.cos(ln.heading_theta_at(0)) > 0]
  starts = [ln.position(0, 0) for ln in lanes]
  x0 = min(p[0] for p in starts)
  y_right = min(p[1] for p in starts)
  dx, dy = x0, y_right + (road.lanes - 0.5) * road.lane_width

  # check: our lane centers against MetaDrive's along the whole track
  all_lanes = [ln for d in env.current_map.road_network.graph.values() for ls in d.values() for ln in ls]
  err = 0.0
  for s in np.linspace(0.0, road.length * 0.98, 80):
    for lane_idx in range(road.lanes):
      px, py, _, _ = road.pose_at(float(s), road.lane_offset(lane_idx))
      best = math.inf
      for ln in all_lanes:
        lon, lat = ln.local_coordinates((px + dx, py + dy))
        if -0.5 <= lon <= ln.length + 0.5:
          best = min(best, abs(lat))
      err = max(err, best)
  status.put({"kind": "metadrive", "alignErr": round(err, 3)})

  ego_box = _box(engine.loader, engine.render, (1.97, 5.1, 1.75), (0.55, 0.05, 0.07, 1.0))   # Soul Red Crystal-ish
  lead_box = _box(engine.loader, engine.render, (1.9, 4.8, 1.5), (0.2, 0.25, 0.3, 1.0))
  lead_box.hide()

  jpeg = cfg.world.frame_codec == "jpeg"
  codec = wire.CODEC_JPEG if jpeg else wire.CODEC_NV12
  srv = wire.listen(port)
  srv.settimeout(0.0)
  link: wire.Link | None = None
  frame_id = 0
  last = time.monotonic()
  next_t = last
  fps = 0.0
  pose = None
  parent = mp.parent_process()
  while True:
    if parent is not None and not parent.is_alive():
      return
    next_t += 1.0 / FPS
    time.sleep(max(0.0, next_t - time.monotonic()))
    if next_t < time.monotonic() - 0.5:
      next_t = time.monotonic()   # fell behind (slow GPU / software GL): don't try to catch up
    try:
      while True:
        pose = poses.get_nowait()
    except queue.Empty:
      pass
    if pose is None:
      continue
    try:
      sock, _ = srv.accept()
      sock.setblocking(True)
      if link is not None:
        link.close()
      link = wire.Link(sock)
    except (BlockingIOError, OSError):
      pass

    x, y, yaw = pose["x"] + dx, pose["y"] + dy, pose["yaw"]
    agent.set_position([x, y])
    agent.set_heading_theta(yaw)
    ego_box.setPos(x, y, 0.0)
    ego_box.setH(math.degrees(yaw) - 90.0)
    lead = pose.get("lead")
    if lead is not None:
      lead_box.show()
      lead_box.setPos(lead[0] + dx, lead[1] + dy, 0.0)
      lead_box.setH(math.degrees(lead[2]) - 90.0)
    else:
      lead_box.hide()

    # pose every camera in the world frame (the agent's origin floats at its body center), render once, read buffers
    c, sn = math.cos(yaw), math.sin(yaw)
    h = math.degrees(yaw) - 90.0
    mounts = {"road": ((x + DEVICE_AHEAD * c, y + DEVICE_AHEAD * sn, DEVICE_HEIGHT), (h, 0, 0)),
              "wide": ((x + DEVICE_AHEAD * c, y + DEVICE_AHEAD * sn, DEVICE_HEIGHT), (h, 0, 0)),
              "chase": ((x - 9.0 * c, y - 9.0 * sn, 3.4), (h, -12, 0))}
    for name, (pos, hpr) in mounts.items():
      if name in engine.sensors:
        cam_np = engine.sensors[name].get_cam()
        cam_np.reparentTo(engine.render)
        cam_np.setPos(Vec3(*pos))
        cam_np.setHpr(Vec3(*hpr))
    engine.taskMgr.step()
    images = {}
    for name in mounts:
      if name in engine.sensors and (name != "chase" or (frame_id % 2 == 0 and chase.empty())):
        img = engine.sensors[name].perceive(to_float=False)
        images[name] = np.ascontiguousarray(img if isinstance(img, np.ndarray) else img.get())
    if "chase" in images:
      ok, jpg = cv2.imencode(".jpg", images["chase"], [cv2.IMWRITE_JPEG_QUALITY, 75])
      if ok:
        chase.put(jpg.tobytes())

    if link is not None and not link.closed:
      t_us = int(time.monotonic() * 1e6)
      try:
        for name, cam_id in (("road", wire.CAM_ROAD), ("wide", wire.CAM_WIDE)):
          if name not in images:
            continue
          img = images[name]
          if img.shape[1] != W:
            img = cv2.resize(img, (W, H), interpolation=cv2.INTER_LINEAR)
          if jpeg:
            data = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes()
          else:
            data = bgr_to_nv12(img)
          link.send(wire.MSG_FRAME, *wire.pack_frame(wire.CameraFrame(cam_id, frame_id, t_us, W, H, codec, data)))
      except wire.ConnectionClosed:
        link = None
    frame_id += 1
    now = time.monotonic()
    fps = 0.9 * fps + 0.1 / max(now - last, 1e-3)
    last = now
    if frame_id % 20 == 0:
      status.put({"kind": "metadrive", "fps": round(fps, 1), "alignErr": round(err, 3), "commaWatching": link is not None})


class MetaDriveWorld:
  def __init__(self, cfg: SimConfig, port: int = wire.OPTICS_PORT):
    self.cfg = cfg
    ctx = mp.get_context("spawn")
    self.poses: mp.Queue = ctx.Queue(maxsize=2)
    self.chase_q: mp.Queue = ctx.Queue(maxsize=1)
    self.status_q: mp.Queue = ctx.Queue()
    self._status = {"kind": "metadrive", "state": "starting"}
    self._jpeg: bytes | None = None
    self.proc = ctx.Process(target=_renderer, args=(cfg.to_dict(), self.poses, self.chase_q, self.status_q, port), daemon=True)
    self.proc.start()
    threading.Thread(target=self._collect, daemon=True).start()

  def _collect(self) -> None:
    while True:
      try:
        self._jpeg = self.chase_q.get(timeout=0.2)
      except queue.Empty:
        pass
      try:
        while True:
          self._status.update(self.status_q.get_nowait())
          self._status["state"] = "running"
      except queue.Empty:
        pass
      if not self.proc.is_alive():
        self._status["state"] = f"renderer exited ({self.proc.exitcode})"

  def update(self, car) -> None:
    v = car.veh
    pose = {"x": v.x, "y": v.y, "yaw": v.yaw}
    rel = car.lead_rel()
    if rel is not None:
      lx, ly, lh, _ = car.road.pose_at(car.lead.s - 2.4, car.road.lane_offset(car.lane))
      pose["lead"] = (lx, ly, lh)
    try:
      self.poses.put_nowait(pose)
    except queue.Full:
      pass

  def chase_jpeg(self) -> bytes | None:
    return self._jpeg

  def status(self) -> dict:
    return dict(self._status)

