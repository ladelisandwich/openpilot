#!/usr/bin/env python3
"""The Mazda container: the car, its road, and the servers the comma and the dashboard connect to.

  harness  :7000  the comma's harness connector (CAN, ignition, sensors, ground truth)
  optics   :7001  what the comma's cameras see (MetaDrive world only)
  control  :8770  dashboard websocket: your driver inputs and switches in, live telemetry out
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import queue
import signal
import sys
import threading
import time

from ..common import wire
from ..common.config import SimConfig
from .mazda import MazdaCar

DT = 0.01


class HarnessPort:
  """One comma at a time; a new connection replaces the old one, like re-plugging the harness."""

  def __init__(self, port: int):
    self.srv = wire.listen(port)
    self.link: wire.Link | None = None
    self.inbound: queue.Queue = queue.Queue()
    self.connected_at = 0.0
    threading.Thread(target=self._accept, daemon=True).start()

  def _accept(self) -> None:
    while True:
      sock, addr = self.srv.accept()
      link = wire.Link(sock)
      if self.link is not None:
        self.link.close()
      self.link = link
      self.connected_at = time.monotonic()
      print(f"car: comma plugged in from {addr[0]}")
      threading.Thread(target=self._read, args=(link,), daemon=True).start()

  def _read(self, link: wire.Link) -> None:
    while True:
      try:
        self.inbound.put(link.recv())
      except wire.ConnectionClosed:
        if self.link is link:
          self.link = None
          self.inbound.put((None, b""))
          print("car: comma unplugged")
        return

  def send(self, msg_type: int, *parts: bytes) -> None:
    link = self.link
    if link is None:
      return
    try:
      link.send(msg_type, *parts)
    except wire.ConnectionClosed:
      pass

  @property
  def connected(self) -> bool:
    return self.link is not None


class Sim:
  def __init__(self, cfg: SimConfig, world=None):
    self.cfg = cfg
    self.car = MazdaCar(cfg)
    self.harness = HarnessPort(wire.HARNESS_PORT)
    self.world = world
    self.lock = threading.Lock()
    self.panda: dict = {}
    self.driver_distracted = False
    self.relay_stuck = False
    self.running = True
    self.frame = 0
    self.loop_ms = 0.0
    self.paused = False

  # ---- comma messages ----
  def _drain_comma(self) -> list[wire.Frame]:
    frames: list[wire.Frame] = []
    while True:
      try:
        msg_type, payload = self.harness.inbound.get_nowait()
      except queue.Empty:
        return frames
      if msg_type is None:
        self.car.op = {}
        self.panda = {}
        self.car.net.relay_intercept = False
      elif msg_type == wire.MSG_CAN:
        frames.extend(wire.unpack_can(payload)[1])
      elif msg_type == wire.MSG_PANDA:
        self.panda = wire.unpack_json(payload)
        self.car.net.relay_intercept = bool(self.panda.get("relayIntercept"))
      elif msg_type == wire.MSG_OPENPILOT:
        self.car.op = wire.unpack_json(payload)

  def step(self) -> None:
    t0 = time.perf_counter()
    with self.lock:
      inbound = self._drain_comma()
      if self.paused:
        return
      out = self.car.tick(DT, inbound)
      t_us = int(time.monotonic() * 1e6)
      if self.harness.connected:
        if out:
          self.harness.send(wire.MSG_CAN, wire.pack_can(t_us, out))
        self.harness.send(wire.MSG_SENSORS, wire.pack_sensors(self.car.sensors(t_us)))
        if self.frame % 5 == 0:
          self.harness.send(wire.MSG_TRUTH, wire.pack_json(self.car.truth()))
        if self.frame % 10 == 0:
          self.harness.send(wire.MSG_CAR_STATE, wire.pack_json({
            "ignition": self.car.body.ignition, "voltageMv": 14200 if self.car.body.ignition else 12300,
            "driverDistracted": self.driver_distracted, "relayStuck": self.relay_stuck}))
      if self.world is not None and self.frame % 5 == 0:
        self.world.update(self.car)
    self.frame += 1
    self.loop_ms = 0.98 * self.loop_ms + 0.02 * (time.perf_counter() - t0) * 1000

  def run(self) -> None:
    next_t = time.monotonic()
    while self.running:
      self.step()
      next_t += DT
      delay = next_t - time.monotonic()
      if delay > 0:
        time.sleep(delay)
      elif delay < -0.25:
        next_t = time.monotonic()  # fell behind (debugger, host hiccup): do not try to catch up

  # ---- dashboard ----
  def command(self, cmd: dict) -> None:
    car, body, drv = self.car, self.car.body, self.car.driver
    t = cmd.get("type")
    with self.lock:
      if t == "axes":
        drv.torque_input = max(-1.0, min(1.0, float(cmd.get("steer", 0.0))))
        drv.gas_input = max(0.0, min(1.0, float(cmd.get("gas", 0.0))))
        drv.brake_input = max(0.0, min(1.0, float(cmd.get("brake", 0.0))))
      elif t == "button":
        name = str(cmd.get("name"))
        if "hold" in cmd:
          body.buttons.hold(name, bool(cmd["hold"]))
        else:
          body.buttons.press(name, float(cmd.get("duration", 0.25)))
      elif t == "set":
        key, value = cmd.get("key"), cmd.get("value")
        if key == "ignition":
          car.set_ignition(bool(value))
        elif key in ("gear", "blinker"):
          setattr(body, key, str(value))
        elif key in ("seatbelt", "door_open", "high_beams", "blindspot_left", "blindspot_right"):
          setattr(body, key, bool(value))
        elif key == "driver_mode":
          drv.mode = str(value)
        elif key == "auto_speed_kph":
          drv.auto_speed_kph = float(value)
        elif key == "lane":
          car.lane = max(0, min(car.road.lanes - 1, int(value)))
        elif key == "driver_distracted":
          self.driver_distracted = bool(value)
        elif key == "paused":
          self.paused = bool(value)
      elif t == "fault":
        self._fault(str(cmd.get("name")), cmd.get("value"))
      elif t == "reset":
        car.reset()
      elif t == "lead":
        car.lead.mode = str(cmd.get("mode", car.lead.mode))
        car.lead.cruise = float(cmd.get("speed_kph", car.lead.cruise * 3.6)) / 3.6
        car.lead.gap0 = float(cmd.get("gap_m", car.lead.gap0))
        car.lead.reset(car.road_pos[0], car.veh.v)
      elif t == "plant":
        key = str(cmd.get("key"))
        if hasattr(self.cfg.plant, key):
          setattr(self.cfg.plant, key, cmd.get("value"))

  def _fault(self, name: str, value) -> None:
    car = self.car
    v = bool(value)
    if name == "ti_unplugged":
      car.ti.fault_unplugged = v
    elif name == "ti_sensor_error":
      car.ti.fault_error = 0x01 if v else 0
    elif name == "ti_ignore_commands":
      car.ti.fault_ignore_commands = v
    elif name == "radar_refuse_programming":
      car.radar.refuse_programming = v
    elif name == "radar_restart_in_standby":
      car.radar.restart_in_standby = v
    elif name == "radar_restart":
      car.radar._restart()
    elif name == "relay_stuck":
      self.relay_stuck = v
    elif name == "eps_lockout":
      car.eps.locked_out = v
      car.eps.hands_off_t = 1e9 if v else 0.0

  def telemetry(self) -> dict:
    with self.lock:
      tel = self.car.telemetry()
      tel["op"] = self.car.op
      tel["panda"] = self.panda
      tel["comma"] = {"connected": self.harness.connected, "loopMs": round(self.loop_ms, 2)}
      tel["world"] = self.world.status() if self.world is not None else {"kind": "lite"}
      tel["driverDistracted"] = self.driver_distracted
      tel["relayStuck"] = self.relay_stuck
      tel["paused"] = self.paused
    return tel

  def road_shape(self) -> dict:
    r = self.car.road
    step = max(1, r.n // 3000)
    return {"x": [round(float(v), 1) for v in r.x[::step]], "y": [round(float(v), 1) for v in r.y[::step]],
            "lanes": r.lanes, "laneWidth": r.lane_width, "closed": r.closed}


async def serve_control(sim: Sim, port: int) -> None:
  from aiohttp import WSMsgType, web

  async def ws_handler(request):
    ws = web.WebSocketResponse(heartbeat=10)
    await ws.prepare(request)
    await ws.send_str(json.dumps({"type": "road", "road": sim.road_shape(), "config": sim.cfg.to_dict()}))

    async def pump():
      while not ws.closed:
        await ws.send_str(json.dumps({"type": "tel", **sim.telemetry()}))
        await asyncio.sleep(0.05)

    task = asyncio.create_task(pump())
    try:
      async for msg in ws:
        if msg.type == WSMsgType.TEXT:
          try:
            sim.command(json.loads(msg.data))
          except Exception as e:  # a bad command must never stop the car
            await ws.send_str(json.dumps({"type": "error", "error": str(e)}))
    finally:
      task.cancel()
    return ws

  async def status(_request):
    return web.json_response(sim.telemetry())

  async def chase(_request):
    if sim.world is None:
      raise web.HTTPNotFound(text="lite world: the dashboard draws the scene itself")
    resp = web.StreamResponse(headers={"Content-Type": "multipart/x-mixed-replace; boundary=frame",
                                       "Cache-Control": "no-cache", "Access-Control-Allow-Origin": "*"})
    await resp.prepare(_request)
    last = None
    while True:
      jpg = sim.world.chase_jpeg()
      if jpg is not None and jpg is not last:
        last = jpg
        await resp.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
      await asyncio.sleep(0.05)

  @web.middleware
  async def cors(request, handler):
    resp = await handler(request)
    if not isinstance(resp, web.StreamResponse) or not resp.prepared:
      resp.headers["Access-Control-Allow-Origin"] = "*"
    return resp

  app = web.Application(middlewares=[cors])
  app.router.add_get("/ws", ws_handler)
  app.router.add_get("/status", status)
  app.router.add_get("/chase.mjpg", chase)
  runner = web.AppRunner(app)
  await runner.setup()
  await web.TCPSite(runner, "0.0.0.0", port).start()
  print(f"car: dashboard control on :{port}")
  while True:
    await asyncio.sleep(3600)


def main() -> None:
  ap = argparse.ArgumentParser(description="Mazda CX-9 2023 + TI1 simulator (car side)")
  ap.add_argument("--config", default=os.environ.get("MAZDA_SIM_CONFIG", ""))
  ap.add_argument("--control-port", type=int, default=wire.CONTROL_PORT)
  args = ap.parse_args()
  cfg = SimConfig.load(args.config)
  signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))   # docker stop: exit cleanly so the renderer goes too
  world = None
  if cfg.world.world == "metadrive":
    from .world_metadrive import MetaDriveWorld
    world = MetaDriveWorld(cfg)
  sim = Sim(cfg, world)
  ti = f"TI{cfg.car.ti_version}" if cfg.car.torque_interceptor else "no TI"
  print(f"car: {cfg.car.fingerprint} on '{cfg.world.track}' ({sim.car.road.length:.0f} m), {ti}, " +
        f"radar emulation {cfg.car.radar_emulation}, hybrid long {cfg.car.hybrid_long}, model {cfg.world.model}")
  threading.Thread(target=sim.run, daemon=True).start()
  asyncio.run(serve_control(sim, args.control_port))


if __name__ == "__main__":
  main()
