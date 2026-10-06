"""Stock MRCC radar (fwdRadar, 0x764) on the car's main CAN.

It is the ACC brain: it follows the PCM's engagement and set speed, measures the lead and commands acceleration in
CRZ_INFO (0x21B), reports cruise status in CRZ_CTRL (0x21C), and broadcasts its track heartbeats (0x361-0x366, 0x499).

Diagnostic sessions, as openpilot's radar emulation and hybrid long rely on:
  programming session (0x10 0x02)  -> positive response, then silent; tester present keeps it there
  S3 timeout (5 s without requests) or default session (0x10 0x01) -> restarts, ~12 s warm-up before SET/RES work
  restart while the PCM still has cruise engaged -> comes back in standby (ACCEL_CMD 4094, no command) until RES
"""
from __future__ import annotations

from collections.abc import Callable

from ..common.wire import Frame, NET_CAR
from .canbus import Ecu, Periodic, addr_of, decode, encode
from .ecus import CX9_FW, KPH, Body, accel_to_accel_cmd, crz_info_checksum
from .isotp import DiagEndpoint, UdsServer
from .vehicle import Vehicle

CRZ_INFO_STANDBY = bytes.fromhex("01ffe3ffc0000000")
CRZ_INFO_ACTIVE = bytes.fromhex("01ffe20006800000")
CRZ_CTRL_STANDBY = bytes.fromhex("0201010000000000")
CRZ_CTRL_CRUISE = bytes.fromhex("0a018b2000001000")
CRZ_CTRL_FOLLOW = bytes.fromhex("0a018b4000001000")
# heartbeats from a 2023 CX-9 capture (opendbc/car/mazda/longitudinal.py)
HEARTBEATS = {
  0x499: bytes.fromhex("0008c00000000000"),
  0x361: bytes.fromhex("fff7fefe1fc00080"),
  0x362: bytes.fromhex("fff7fefe1fc97080"),
  0x363: bytes.fromhex("fff7fefe1fc00000"),
  0x364: bytes.fromhex("fff7fefe1fc00000"),
  0x365: bytes.fromhex("04808e08a0461d40"),
  0x366: bytes.fromhex("1911607ffbff03c0"),
}
GAP_S = {1: 2.2, 2: 1.8, 3: 1.4, 4: 1.0}   # DISTANCE_SETTING code -> time gap (code 1 = 4 bars)

PEDALS_ADDR = addr_of("PEDALS")
CRZ_EVENTS_ADDR = addr_of("CRZ_EVENTS")
CRZ_BTNS_ADDR = addr_of("CRZ_BTNS")
ENGINE_DATA_ADDR = addr_of("ENGINE_DATA")


class Radar(Ecu):
  name = "radar"
  net = NET_CAR
  BOOT_T = 4.0
  RESTART_T = 12.0
  S3_T = 5.0

  def __init__(self, veh: Vehicle, body: Body, lead: Callable[[], tuple[float, float] | None]):
    super().__init__()
    self.veh = veh
    self.body = body
    self.lead = lead
    self.uds = UdsServer(CX9_FW["fwdRadar"][1], on_session=self._on_session, on_comm_control=self._on_comm_control)
    self.diag = DiagEndpoint(NET_CAR, CX9_FW["fwdRadar"][0], self.uds.handle)
    self.warmup = self.BOOT_T
    self.programming = False
    self.enter_programming_in: float | None = None
    self.comm_disabled = False
    self.pcm_engaged = False
    self.pcm_main = True
    self.set_speed_kph = 0.0
    self.res_edge = False
    self.driver_gas = False
    self.btn_prev = {}
    self.distance_code = 2
    self.engaged_prev = False
    self.stuck_standby = False
    self.stopped_t = 0.0
    self.need_resume = False
    self.accel = 0.0
    self.ctr = 0
    self.hb_ctr = 0
    self.restarts = 0
    # fault injection
    self.refuse_programming = False
    self.restart_in_standby = True
    self.periodics = [
      Periodic("CRZ_INFO", 50.0, self._crz_info, 0.15),
      Periodic("CRZ_CTRL", 50.0, self._crz_ctrl, 0.65),
      Periodic(0x499, 10.0, lambda: self._heartbeat(0x499), 0.05),
      *[Periodic(a, 10.0, (lambda a=a: self._heartbeat(a)), 0.1 + 0.05 * i) for i, a in enumerate(range(0x361, 0x367))],
    ]

  # ---- diagnostics ----
  def _on_session(self, sub: int) -> bool:
    if sub == 0x02:
      if self.refuse_programming:
        return False
      self.enter_programming_in = 0.02
    elif sub == 0x01 and (self.programming or self.comm_disabled):
      self._restart()
    return True

  def _on_comm_control(self, ctrl: int, _comm_type: int) -> None:
    self.comm_disabled = ctrl in (0x01, 0x02, 0x03)

  def _restart(self) -> None:
    self.programming = False
    self.comm_disabled = False
    self.silent = False
    self.uds.session = 0x01
    self.warmup = self.RESTART_T
    self.restarts += 1
    self.stuck_standby = self.restart_in_standby and self.pcm_engaged
    self.accel = 0.0

  @property
  def alive(self) -> bool:
    return self.powered and not self.silent

  @property
  def state(self) -> str:
    if not self.powered:
      return "off"
    if self.programming:
      return "programming (silent)"
    if self.comm_disabled:
      return "comm disabled"
    if self.warmup > 0:
      return f"warming up {self.warmup:.0f}s"
    if self.stuck_standby:
      return "standby (needs RES)"
    return "active" if self.pcm_engaged else "ready"

  # ---- bus inputs ----
  def on_frame(self, f: Frame) -> None:
    if self.diag.on_frame(f):
      return
    if f.addr == PEDALS_ADDR and len(f.dat) == 8:
      s = decode("PEDALS", f.dat)
      self.pcm_engaged = bool(s["ACC_ACTIVE"])
      self.pcm_main = bool(s["CRZ_AVAILABLE"])
    elif f.addr == ENGINE_DATA_ADDR and len(f.dat) == 8:
      self.driver_gas = decode("ENGINE_DATA", f.dat)["PEDAL_GAS"] > 0
    elif f.addr == CRZ_EVENTS_ADDR and len(f.dat) == 8:
      self.set_speed_kph = decode("CRZ_EVENTS", f.dat)["CRZ_SPEED"]
    elif f.addr == CRZ_BTNS_ADDR and len(f.dat) == 8:
      s = decode("CRZ_BTNS", f.dat)
      for k in ("RES", "DISTANCE_LESS"):
        v = int(s[k])
        if v and not self.btn_prev.get(k, 0):
          if k == "RES":
            self.res_edge = True
          elif not self.silent:
            self.distance_code = 1 + (self.distance_code % 4)
        self.btn_prev[k] = v

  # ---- ACC ----
  def step(self, dt: float) -> None:
    self.uds.last_request_age += dt
    if self.enter_programming_in is not None:
      self.enter_programming_in -= dt
      if self.enter_programming_in <= 0:
        self.enter_programming_in = None
        self.programming = True
    if self.programming and self.uds.last_request_age > self.S3_T:
      self._restart()
    self.silent = self.programming or self.comm_disabled
    if self.silent:
      self.accel = 0.0
      self.res_edge = False
      return
    self.warmup = max(0.0, self.warmup - dt)

    if self.stuck_standby and (self.res_edge or not self.pcm_engaged):
      self.stuck_standby = False
    if self.pcm_engaged and not self.engaged_prev:
      self.need_resume = False
      self.stopped_t = 0.0
    self.engaged_prev = self.pcm_engaged

    v = self.veh.speed
    if not self.pcm_engaged or self.stuck_standby or self.warmup > 0:
      self.accel = 0.0
      self.res_edge = False
      return

    v_set = self.set_speed_kph / KPH
    a_target = max(-1.5, min(1.2, 0.35 * (v_set - v)))
    lead = self.lead()
    if lead is not None:
      dist, v_lead = lead
      gap = 4.0 + v * GAP_S.get(self.distance_code, 1.8)
      a_follow = 0.22 * (dist - gap) + 0.9 * (v_lead - v)
      if dist < 8.0 and v_lead < 0.5:
        a_follow = min(a_follow, -1.0 if v > 0.1 else -0.5)
      a_target = min(a_target, max(-3.5, min(1.5, a_follow)))

    # stop and go: after more than 3 s stopped, wait for RES (or the gas) before driving off
    if self.veh.standstill:
      self.stopped_t += dt
      if self.stopped_t > 3.0:
        self.need_resume = True
      if self.need_resume and not self.res_edge:
        a_target = min(a_target, -0.5)
    else:
      self.stopped_t = 0.0
    if self.res_edge or self.driver_gas:
      self.need_resume = False
    self.res_edge = False

    jerk = 2.5 * dt
    self.accel += max(-jerk, min(jerk, a_target - self.accel))

  # ---- frames ----
  @property
  def set_allowed(self) -> bool:
    return self.warmup <= 0 and self.pcm_main and self.body.gear == "D" and not self.stuck_standby

  def _crz_info(self) -> bytes:
    self.ctr = (self.ctr + 1) % 16
    v = self.veh.speed
    if self.stuck_standby or (not self.pcm_engaged and not self.set_allowed):
      raw = encode("CRZ_INFO", {"CTR1": self.ctr}, CRZ_INFO_STANDBY)
    else:
      active = self.pcm_engaged and self.warmup <= 0
      stopping = active and self.veh.standstill and self.accel <= 0.0
      raw = encode("CRZ_INFO", {
        "ACCEL_CMD": accel_to_accel_cmd(self.accel, v) if active else 0,
        "ACC_ACTIVE": int(active), "ACC_SET_ALLOWED": int(self.set_allowed or active), "CRZ_ENDED": 0,
        "STOPPING_MAYBE": int(stopping), "STOPPING_MAYBE2": int(stopping), "CTR1": self.ctr,
      }, CRZ_INFO_ACTIVE)
    d = bytearray(raw)
    d[7] = crz_info_checksum(d)
    return bytes(d)

  def _crz_ctrl(self) -> bytes:
    lead = self.lead()
    active = self.pcm_engaged and not self.stuck_standby and self.warmup <= 0
    if not (active or self.set_allowed):
      return CRZ_CTRL_STANDBY
    tmpl = CRZ_CTRL_FOLLOW if lead is not None else CRZ_CTRL_CRUISE
    rel = 0
    if lead is not None:
      rel = 1 if lead[0] > 80 else 2 if lead[0] > 50 else 3 if lead[0] > 30 else 4 if lead[0] > 15 else 5
    return encode("CRZ_CTRL", {"CRZ_ACTIVE": int(active), "ACC_ACTIVE_2": int(active), "CRZ_AVAILABLE": int(self.pcm_main),
                               "DISTANCE_SETTING": self.distance_code, "RADAR_HAS_LEAD": int(lead is not None),
                               "RADAR_LEAD_RELATIVE_DISTANCE": rel, "ACC_GAS_MAYBE2": int(active and self.accel > 0)}, tmpl)

  def _heartbeat(self, addr: int) -> bytes:
    if addr == 0x499:
      self.hb_ctr = (self.hb_ctr + 1) % 16
    d = bytearray(HEARTBEATS[addr])
    if addr != 0x499:
      d[7] = (d[7] & 0xF0) | self.hb_ctr
    return bytes(d)

  def emit(self, dt: float) -> list[Frame]:
    return super().emit(dt) + self.diag.drain()

  def telemetry(self) -> dict:
    lead = self.lead()
    return {"state": self.state, "session": {1: "default", 2: "programming", 3: "extended"}.get(self.uds.session, "?"),
            "accel": round(self.accel, 2), "distanceBars": 5 - self.distance_code, "restarts": self.restarts,
            "lead": None if lead is None else {"dist": round(lead[0], 1), "v": round(lead[1], 2)},
            "refuseProgramming": self.refuse_programming, "restartInStandby": self.restart_in_standby}
