"""Chassis and powertrain modules of a GEN1 CX-9 on the car's main CAN (panda bus 0 side)."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from ..common.config import CarOptions, PlantOptions
from ..common.wire import Frame, NET_CAR
from .canbus import Ecu, Periodic, addr_of, decode, encode, with_checksum
from .isotp import DiagEndpoint, UdsServer
from .vehicle import Vehicle

KPH = 3.6
MPH = 2.23694

CAM_LKAS_ADDR = addr_of("CAM_LKAS")
CRZ_BTNS_ADDR = addr_of("CRZ_BTNS")
CRZ_INFO_ADDR = addr_of("CRZ_INFO")
CRZ_CTRL_ADDR = addr_of("CRZ_CTRL")
PEDALS_ADDR = addr_of("PEDALS")
CRZ_EVENTS_ADDR = addr_of("CRZ_EVENTS")
LANEINFO_ADDR = addr_of("CAM_LANEINFO")

# FW versions served per module for a 2021-23 CX-9 (one of each from the fingerprints)
CX9_FW = {
  "eps": (0x730, b'TC3M-3210X-A-00\x00\x00\x00\x00\x00\x00\x00\x00\x00'),
  "engine": (0x7e0, b'PXM7-188K2-E\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'),
  "fwdRadar": (0x764, b'K131-67XK2-F\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'),
  "abs": (0x760, b'TA0B-437K2-C\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'),
  "fwdCamera": (0x706, b'GSH7-67XK2-U\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'),
  "transmission": (0x7e1, b'PXM7-21PS1-C\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'),
}


def cam_lkas_checksum(req: int, ctr: int, er1: int, lnv: int, ldw: int, er2: int, b1: int, angle: int, b2: int) -> int:
  """The FSC's CAM_LKAS checksum, as openpilot computes it (mazdacan.create_steering_control)."""
  tmp = req + 2048
  lo, hi = tmp & 0xFF, tmp >> 8
  tmp = angle + 2048
  ahi = tmp >> 10
  amd = (tmp & 0x3FF) >> 2
  amd = (amd >> 4) | ((amd & 0xF) << 4)
  alo = (tmp & 0x3) << 2
  csum = 249 - ctr - hi - lo - (lnv << 3) - er1 - (ldw << 7) - (er2 << 4) - (b1 << 5)
  csum = csum - ahi - amd - alo - b2
  if ahi == 1:
    csum += 15
  if csum < 0:
    csum += 512 if csum < -256 else 256
  return csum % 256


def make_cam_lkas(req: int, ctr: int, er1: int = 0, er2: int = 0, b1: int = 1, lnv: int = 0, ldw: int = 0) -> bytes:
  csum = cam_lkas_checksum(req, ctr, er1, lnv, ldw, er2, b1, 0, 0)
  return encode("CAM_LKAS", {"LKAS_REQUEST": req, "CTR": ctr, "ERR_BIT_1": er1, "LINE_NOT_VISIBLE": lnv, "LDW": ldw,
                             "BIT_1": b1, "ERR_BIT_2": er2, "STEERING_ANGLE": 0, "ANGLE_ENABLED": 0, "CHKSUM": csum})


@dataclass
class Buttons:
  """Steering-wheel switches currently held, with how long each stays held (s)."""
  held: dict[str, float] = field(default_factory=dict)

  def press(self, name: str, duration: float = 0.25) -> None:
    self.held[name] = max(self.held.get(name, 0.0), duration)

  def hold(self, name: str, on: bool) -> None:
    if on:
      self.held[name] = 1e9
    else:
      self.held.pop(name, None)

  def step(self, dt: float) -> None:
    for k in list(self.held):
      self.held[k] -= dt
      if self.held[k] <= 0:
        del self.held[k]

  def __contains__(self, name: str) -> bool:
    return name in self.held


@dataclass
class Body:
  gear: str = "D"
  ignition: bool = True
  seatbelt: bool = True
  door_open: bool = False
  blinker: str = ""            # "", "left", "right", "hazard"
  high_beams: bool = False
  blindspot_left: bool = False
  blindspot_right: bool = False
  buttons: Buttons = field(default_factory=Buttons)


class Eps(Ecu):
  """Electric power steering: assist, the stock LKAS channel, and its hands-off lockout."""
  name = "eps"
  net = NET_CAR
  LKAS_TIMEOUT_T = 0.05
  HANDS_ON_NM = 0.35
  MOTOR_SLEW_NM_S = 25.0

  def __init__(self, veh: Vehicle, plant: PlantOptions):
    super().__init__()
    self.veh = veh
    self.p = plant
    self.lkas_req = 0
    self.lkas_ctr = None
    self.since_lkas = 1e9
    self.lkas_bad_frames = 0
    self.speed_ok = False
    self.hands_off_t = 0.0
    self.locked_out = False
    self.fault_t = 0.0
    self.lkas_effective = 0
    self.diag = DiagEndpoint(NET_CAR, CX9_FW["eps"][0], UdsServer(CX9_FW["eps"][1]).handle)
    self.periodics = [
      Periodic("STEER", 100.0, self._steer),
      Periodic("STEER_RATE", 83.3, self._steer_rate, 0.5),
      Periodic("STEER_TORQUE", 83.3, self._steer_torque, 0.2),
    ]
    self.ctr = 0

  @property
  def lkas_active(self) -> bool:
    return self.speed_ok and not self.locked_out and self.fault_t <= 0 and self.since_lkas < self.LKAS_TIMEOUT_T

  def on_frame(self, f: Frame) -> None:
    if self.diag.on_frame(f):
      return
    if f.addr != CAM_LKAS_ADDR or len(f.dat) != 8:
      return
    s = decode("CAM_LKAS", f.dat)
    req, ctr = int(s["LKAS_REQUEST"]), int(s["CTR"])
    expect = cam_lkas_checksum(req, ctr, int(s["ERR_BIT_1"]), int(s["LINE_NOT_VISIBLE"]), int(s["LDW"]), int(s["ERR_BIT_2"]),
                               int(s["BIT_1"]), int(s["STEERING_ANGLE"]), int(s["ANGLE_ENABLED"]))
    if expect != int(s["CHKSUM"]):
      self.lkas_bad_frames += 1
      return
    if self.lkas_ctr is not None and ctr not in ((self.lkas_ctr + 1) % 16, self.lkas_ctr):
      self.lkas_bad_frames += 1
    if self.since_lkas < 0.2 and abs(req - self.lkas_req) > 150:
      self.fault_t = 1.0   # a torque step this big makes the real EPS drop LKAS for a moment
    self.lkas_ctr = ctr
    self.lkas_req = req
    self.since_lkas = 0.0

  def step(self, dt: float) -> None:
    self.since_lkas += dt
    self.fault_t = max(0.0, self.fault_t - dt)
    kph = self.veh.speed * KPH
    if kph > self.p.lkas_enable_kph:
      self.speed_ok = True
    elif kph < self.p.lkas_disable_kph:
      self.speed_ok = False

    hands_on = abs(self.veh.steer.sensor_eps) > self.HANDS_ON_NM
    if hands_on or not self.speed_ok:
      self.hands_off_t = 0.0
      self.locked_out = False
    elif self.since_lkas < self.LKAS_TIMEOUT_T and self.lkas_req != 0:
      self.hands_off_t += dt
      if self.hands_off_t > self.p.hands_off_lockout_s:
        self.locked_out = True

    target = self.lkas_req / 800.0 * self.p.lkas_full_nm if self.lkas_active else 0.0
    cur = self.veh.steer.t_lkas
    step = self.MOTOR_SLEW_NM_S * dt
    self.veh.steer.t_lkas = cur + max(-step, min(step, target - cur))
    self.lkas_effective = int(round(self.veh.steer.t_lkas / self.p.lkas_full_nm * 800.0))

  def emit(self, dt: float) -> list[Frame]:
    return super().emit(dt) + self.diag.drain()

  def _steer(self) -> bytes:
    self.ctr = (self.ctr + 1) % 16
    d = encode("STEER", {"STEER_ANGLE": max(-1600.0, min(1600.0, self.veh.steering_wheel_deg)), "CTR": self.ctr})
    return with_checksum(d)

  def _steer_rate(self) -> bytes:
    hands_warn = self.hands_off_t > max(0.0, self.p.hands_off_lockout_s - 5.0)
    d = encode("STEER_RATE", {
      "STEER_ANGLE_RATE": max(-8000.0, min(8000.0, self.veh.steering_rate_deg)),
      "CTR": self.ctr,
      "LKAS_REQUEST": self.lkas_req if self.since_lkas < self.LKAS_TIMEOUT_T else 0,
      "LKAS_EFFECTIVE": self.lkas_effective,
      "HANDS_OFF_5_SECONDS": int(hands_warn),
      "LKAS_BLOCK": int(not self.speed_ok or self.locked_out or self.fault_t > 0),
      "LKAS_TRACK_STATE": int(self.lkas_active),
    })
    return with_checksum(d)

  def _steer_torque(self) -> bytes:
    units = self.veh.steer.sensor_eps / self.p.sensor_nm_per_unit
    return encode("STEER_TORQUE", {
      "STEER_TORQUE_SENSOR": max(-127.0, min(128.0, round(units))),
      "STEER_TORQUE_MOTOR": max(-1600.0, min(1600.0, self.veh.steer.t_motor * 10.0)),
    })

  def telemetry(self) -> dict:
    return {"lkasReq": self.lkas_req if self.since_lkas < 0.2 else None, "lkasActive": self.lkas_active,
            "lkasEffective": self.lkas_effective, "speedOk": self.speed_ok, "lockedOut": self.locked_out,
            "handsOffT": round(self.hands_off_t, 1), "badFrames": self.lkas_bad_frames, "fault": self.fault_t > 0,
            "sensorNm": round(self.veh.steer.sensor_eps, 3), "assistNm": round(self.veh.steer.t_assist, 3),
            "lkasNm": round(self.veh.steer.t_lkas, 3), "motorNm": round(self.veh.steer.t_motor, 3)}


class Pcm(Ecu):
  """Engine/PCM: executes the ACC master's acceleration, owns the cruise state and set speed.

  Engagement belongs to the PCM: SET or RES engages when the ACC master's CRZ_INFO says ACC_SET_ALLOWED.
  Whoever sends CRZ_INFO (the stock radar, or openpilot impersonating it) then commands the acceleration.
  The PCM drops cruise on CANCEL, the brake, leaving D, an open door or belt, a master that reports itself
  inactive while still commanding, or CRZ_INFO going missing. A master that sends standby frames (no command)
  does not drop cruise: the PCM holds the throttle with nothing braking, as the real car does after a radar
  restart.
  """
  name = "pcm"
  net = NET_CAR
  ACTUATOR_TAU = 0.3
  MASTER_GRACE_T = 0.6
  CRZ_INFO_LOST_T = 0.5

  def __init__(self, veh: Vehicle, body: Body, car: CarOptions):
    super().__init__()
    self.veh = veh
    self.body = body
    self.car = car
    self.driver_gas = 0.0
    self.driver_brake = 0.0
    self.main_on = True
    self.engaged = False
    self.set_speed_kph = 0.0
    self.engaged_t = 0.0
    self.inactive_frames = 0
    self.since_crz_info = 1e9
    self.crz_info: dict | None = None
    self.crz_info_raw = b""
    self.crz_info_bad = 0
    self.a_cmd = 0.0          # m/s^2 from the master's last valid command
    self.a_out = 0.0          # after actuator lag
    self.standby = True
    self.hold_t = 0.0
    self.res_seen_t = 1e9
    self.cancel_reason = ""
    self.btn_prev: dict[str, int] = {}
    self.hold_repeat_t = 0.0
    self.ctr = 0
    self.events_ctr = 0
    self.diag = DiagEndpoint(NET_CAR, CX9_FW["engine"][0], UdsServer(CX9_FW["engine"][1], vin=car.vin).handle, obd=True)
    self.periodics = [
      Periodic("ENGINE_DATA", 100.0, self._engine_data),
      Periodic("PEDALS", 50.0, self._pedals, 0.3),
      Periodic("CRZ_EVENTS", 50.0, self._crz_events, 0.6),
    ]

  # ---- inputs from the bus ----
  def on_frame(self, f: Frame) -> None:
    if self.diag.on_frame(f):
      return
    if f.addr == CRZ_INFO_ADDR and len(f.dat) == 8:
      d = f.dat
      chk = (0xFF - ((sum(d[:7]) - (d[5] & 0x04) - (d[6] & 0x40)) & 0xFF)) & 0xFF
      if chk != d[7]:
        self.crz_info_bad += 1   # the real car ignores CRZ_INFO with a bad checksum
        return
      self.crz_info = decode("CRZ_INFO", d)
      self.crz_info_raw = bytes(d)
      self.since_crz_info = 0.0
    elif f.addr == CRZ_BTNS_ADDR and len(f.dat) == 8:
      self._buttons(decode("CRZ_BTNS", f.dat))

  @property
  def unit(self) -> float:
    return 1.609344 if self.car.imperial else 1.0

  @property
  def set_allowed(self) -> bool:
    return bool(self.crz_info and self.since_crz_info < self.CRZ_INFO_LOST_T and self.crz_info["ACC_SET_ALLOWED"])

  @property
  def armed(self) -> bool:
    return self.main_on and self.set_allowed and not self.engaged

  def _round_set(self, kph: float) -> float:
    u = self.unit
    lo = 25 if self.car.imperial else 30
    return max(lo, round(kph / u)) * u

  def _buttons(self, b: dict) -> None:
    edge = {}
    for k in ("CAN_OFF", "SET_P", "SET_M", "RES", "MODE_X"):
      v = int(b[k])
      edge[k] = v and not self.btn_prev.get(k, 0)
      self.btn_prev[k] = v
    if b["RES"]:
      self.res_seen_t = 0.0
    if edge["CAN_OFF"]:
      if self.engaged:
        self._disengage("cancel button")
      else:
        self.main_on = False
    if edge["MODE_X"] and not self.engaged:
      self.main_on = not self.main_on
    if self.engaged:
      if edge["SET_P"]:
        self.set_speed_kph = min(self.set_speed_kph + self.unit, 145.0)
      if edge["SET_M"]:
        self.set_speed_kph = max(self.set_speed_kph - self.unit, self._round_set(0))
    elif self.main_on and self.set_allowed and self._can_engage():
      if edge["SET_M"] or edge["SET_P"]:
        self.set_speed_kph = self._round_set(self.veh.speed * KPH)
        self._engage()
      elif edge["RES"] and self.set_speed_kph > 0:
        self._engage()

  def _can_engage(self) -> bool:
    return (self.body.gear == "D" and self.body.seatbelt and not self.body.door_open and
            self.driver_brake <= 0.0)

  def _engage(self) -> None:
    self.engaged = True
    self.engaged_t = 0.0
    self.inactive_frames = 0
    self.cancel_reason = ""

  def _disengage(self, why: str) -> None:
    if self.engaged:
      self.cancel_reason = why
    self.engaged = False
    self.veh.hold = False
    self.hold_t = 0.0

  def step(self, dt: float) -> None:
    self.since_crz_info += dt
    self.res_seen_t += dt
    self.engaged_t += dt
    if self.engaged:
      if self.driver_brake > 0.0:
        self._disengage("brake")
      elif not self._can_engage():
        self._disengage("gear, belt or door")
      elif self.since_crz_info > self.CRZ_INFO_LOST_T:
        self._disengage("CRZ_INFO lost")
    # hold-to-repeat for SET+/SET- while engaged
    for k, sgn in (("set_plus", 1), ("set_minus", -1)):
      if k in self.body.buttons and self.body.buttons.held[k] > 1e8 and self.engaged:
        self.hold_repeat_t += dt
        if self.hold_repeat_t > 0.6:
          self.hold_repeat_t = 0.0
          self.set_speed_kph = max(self._round_set(0), min(145.0, self.set_speed_kph + sgn * 5 * self.unit))

    info = self.crz_info if self.since_crz_info < self.CRZ_INFO_LOST_T else None
    accel_raw = int(info["ACCEL_CMD"]) if info else 4094
    self.standby = info is None or accel_raw >= 4000
    if self.engaged and info is not None and not self.standby:
      if not info["ACC_ACTIVE"] and self.engaged_t > self.MASTER_GRACE_T:
        self.inactive_frames += 1
        if self.inactive_frames >= 3:
          self._disengage("master inactive")
      else:
        self.inactive_frames = 0
      self.a_cmd = accel_cmd_to_accel(accel_raw, self.veh.speed)
    elif self.engaged and self.standby:
      # radar restarted under an engaged cruise: throttle held, nothing brakes
      self.a_cmd = max(self.a_cmd, 0.0) if self.veh.speed > 1.0 else 0.0

    # HOLD at a stop: Mazda needs RES, the gas, or the master's resume unlatch after more than 3 s stopped
    stopping_bits = bool(info and (info["STOPPING_MAYBE"] or info["STOPPING_MAYBE2"]))
    if self.engaged and self.veh.standstill:
      if not self.veh.hold and (self.a_cmd <= 0.0 or stopping_bits):
        self.veh.hold = True
        self.hold_t = 0.0
      elif self.veh.hold:
        self.hold_t += dt
        unlatch = self.res_seen_t < 0.5 or self.driver_gas > 0.0 or bool(info and info["RESUME_UNLATCHING_MAYBE"])
        if self.a_cmd > 0.05 and not stopping_bits and (self.hold_t < 3.0 or unlatch):
          self.veh.hold = False
    elif not self.engaged or not self.veh.standstill:
      if self.veh.hold and not self.veh.standstill:
        self.veh.hold = False
    if self.driver_gas > 0.0:
      self.veh.hold = False

    self.a_out += (self.a_cmd - self.a_out) * min(1.0, dt / self.ACTUATOR_TAU)
    m = self.veh.m
    driver_force = self.driver_gas * self.veh.max_drive_force()
    if self.engaged:
      # the force that produces a_out once drag, rolling and engine braking are paid for
      resist = 0.6 * self.veh.v ** 2 + 0.012 * m * 9.81 + (150.0 if self.veh.speed > 0.5 else 0.0)
      needed = m * self.a_out + resist
      acc_force = min(max(needed, 0.0), self.veh.max_drive_force())
      self.veh.drive_force = max(driver_force, acc_force)
      acc_brake = max(0.0, -needed / m) if self.driver_gas <= 0.0 else 0.0
    else:
      self.veh.drive_force = driver_force
      acc_brake = 0.0
      self.a_cmd = 0.0
    self.veh.brake_decel = max(self.driver_brake * 9.0, acc_brake)

  def emit(self, dt: float) -> list[Frame]:
    return super().emit(dt) + self.diag.drain()

  def _engine_data(self) -> bytes:
    rpm = 750.0 + self.veh.speed * 45.0 + self.driver_gas * 2500.0
    d = encode("ENGINE_DATA", {"SPEED": self.veh.speed * KPH, "RPM": rpm,
                               "PEDAL_GAS": round(self.driver_gas * 255.0)})
    return with_checksum(d)

  def _pedals(self) -> bytes:
    brake = self.driver_brake > 0.0
    acc_off = self.armed
    gear = {"P": 13, "R": 26, "N": 13, "D": 24}.get(self.body.gear, 13)
    d = encode("PEDALS", {"CRZ_AVAILABLE": int(self.main_on), "ACC_ACTIVE": int(self.engaged), "ACC_OFF": int(acc_off),
                          "BRAKE_ON": int(brake), "NO_BRAKE": int(not brake), "BRAKE_ON_2": int(brake),
                          "NO_BRAKE_2": int(not brake), "STANDSTILL": int(self.veh.standstill), "GEAR": gear})
    return with_checksum(d)

  def _crz_events(self) -> bytes:
    self.events_ctr = (self.events_ctr + 1) % 16
    d = encode("CRZ_EVENTS", {"CRZ_SPEED": self.set_speed_kph if self.set_speed_kph > 0 else 0.0,
                              "CRUISE_ACTIVE_CAR_MOVING": int(self.engaged and not self.veh.standstill),
                              "CRZ_STARTED": int(self.engaged), "CTR": self.events_ctr})
    return with_checksum(d)

  def telemetry(self) -> dict:
    return {"mainOn": self.main_on, "engaged": self.engaged, "armed": self.armed, "setSpeedKph": round(self.set_speed_kph, 1),
            "aCmd": round(self.a_cmd, 3), "standby": self.standby, "hold": self.veh.hold, "crzInfoAge": round(min(self.since_crz_info, 99), 2),
            "crzInfoBadChecksum": self.crz_info_bad, "lastCancel": self.cancel_reason}


class Transmission(Ecu):
  name = "tcm"
  net = NET_CAR

  def __init__(self, veh: Vehicle, body: Body):
    super().__init__()
    self.veh = veh
    self.body = body
    self.diag = DiagEndpoint(NET_CAR, CX9_FW["transmission"][0], UdsServer(CX9_FW["transmission"][1]).handle, obd=True)
    self.periodics = [Periodic("GEAR", 40.0, self._gear)]

  def on_frame(self, f: Frame) -> None:
    self.diag.on_frame(f)

  def emit(self, dt: float) -> list[Frame]:
    return super().emit(dt) + self.diag.drain()

  def _gear(self) -> bytes:
    g = {"P": 1, "R": 2, "N": 3, "D": 4}.get(self.body.gear, 1)
    box = {"P": 0, "R": 14, "N": 0}.get(self.body.gear, min(6, 1 + int(self.veh.speed / 9)))
    return encode("GEAR", {"GEAR": g, "GEAR_BOX": box})


class Abs(Ecu):
  name = "abs"
  net = NET_CAR

  def __init__(self, veh: Vehicle):
    super().__init__()
    self.veh = veh
    self.ctr = 0
    self.diag = DiagEndpoint(NET_CAR, CX9_FW["abs"][0], UdsServer(CX9_FW["abs"][1]).handle)
    self.periodics = [Periodic("WHEEL_SPEEDS", 100.0, self._wheels, 0.1), Periodic("BRAKE", 100.0, self._brake, 0.4)]

  def on_frame(self, f: Frame) -> None:
    self.diag.on_frame(f)

  def emit(self, dt: float) -> list[Frame]:
    return super().emit(dt) + self.diag.drain()

  def _wheels(self) -> bytes:
    kph = self.veh.speed * KPH
    # inner wheels slower in a turn: half-track 0.82 m
    dv = self.veh.r * 0.82 * KPH
    return encode("WHEEL_SPEEDS", {"FL": max(0.0, kph - dv), "FR": max(0.0, kph + dv),
                                   "RL": max(0.0, kph - dv), "RR": max(0.0, kph + dv)})

  def _brake(self) -> bytes:
    self.ctr = (self.ctr + 1) % 256
    return encode("BRAKE", {"BRAKE_PRESSURE": min(255.0, self.veh.brake_decel * 25.0), "CTR": self.ctr,
                            "VEHICLE_ACC_X": max(-39.0, min(39.0, self.veh.ax)),
                            "VEHICLE_ACC_Y": max(-4.0, min(4.0, -self.veh.ay))})


class BodyModule(Ecu):
  """BCM and friends: lamps, belts, doors, blind spot, and the steering-wheel cruise switches."""
  name = "body"
  net = NET_CAR

  def __init__(self, veh: Vehicle, body: Body):
    super().__init__()
    self.veh = veh
    self.body = body
    self.t = 0.0
    self.btn_ctr = 0
    self.periodics = [
      Periodic("BLINK_INFO", 50.0, self._blink, 0.7),
      Periodic("SEATBELT", 10.0, self._seatbelt),
      Periodic("DOORS", 10.0, self._doors, 0.5),
      Periodic("BSM", 10.0, self._bsm, 0.2),
      Periodic("CRZ_BTNS", 10.0, self._crz_btns, 0.9),
    ]

  def step(self, dt: float) -> None:
    self.t += dt
    self.body.buttons.step(dt)

  def _blink(self) -> bytes:
    lit = (self.t % 0.66) < 0.33
    left = self.body.blinker in ("left", "hazard") and lit
    right = self.body.blinker in ("right", "hazard") and lit
    return encode("BLINK_INFO", {"LEFT_BLINK": int(left), "RIGHT_BLINK": int(right),
                                 "HIGH_BEAMS": 2 if self.body.high_beams else 0, "LOW_BEAMS": 1})

  def _seatbelt(self) -> bytes:
    return encode("SEATBELT", {"DRIVER_SEATBELT": int(self.body.seatbelt), "PASSENGER_SEATBELT": 0})

  def _doors(self) -> bytes:
    return encode("DOORS", {"FL": int(self.body.door_open), "FR": 0, "BL": 0, "BR": 0})

  def _bsm(self) -> bytes:
    def status(on: bool, side: str) -> int:
      return (2 if self.body.blinker == side else 1) if on else 0
    return encode("BSM", {"LEFT_BS_STATUS": status(self.body.blindspot_left, "left"),
                          "RIGHT_BS_STATUS": status(self.body.blindspot_right, "right"),
                          "STANDSTILL": int(self.veh.standstill), "IS_MOVING": int(not self.veh.standstill)})

  def _crz_btns(self) -> bytes:
    b = self.body.buttons
    vals = {"CAN_OFF": "cancel" in b, "SET_P": "set_plus" in b, "SET_M": "set_minus" in b, "RES": "resume" in b,
            "DISTANCE_LESS": "distance" in b, "DISTANCE_MORE": False, "MODE_X": "mode" in b, "MODE_Y": False}
    sig = {}
    for k, v in vals.items():
      sig[k] = int(v)
      sig[k + "_INV"] = int(not v)
    self.btn_ctr = (self.btn_ctr + 1) % 16
    sig.update({"BIT1": 1, "BIT2": 1, "BIT3": 1, "CTR": self.btn_ctr})
    return encode("CRZ_BTNS", sig)


class Cluster(Ecu):
  """The instrument cluster: only listens, to show what a driver would see on the dash."""
  name = "cluster"
  net = NET_CAR

  def __init__(self):
    super().__init__()
    self.hands_on_warning = False
    self.ldw = False
    self.since_laneinfo = 1e9

  def on_frame(self, f: Frame) -> None:
    if f.addr == LANEINFO_ADDR and len(f.dat) == 8:
      s = decode("CAM_LANEINFO", f.dat)
      self.hands_on_warning = bool(s["HANDS_ON_STEER_WARN"])
      self.ldw = bool(s["LDW_WARN_LL"] or s["LDW_WARN_RL"])
      self.since_laneinfo = 0.0

  def step(self, dt: float) -> None:
    self.since_laneinfo += dt


# ---- the acceleration map: the same raw <-> m/s^2 relation openpilot's Mazda port assumes ----
_UP_BP, _UP_V = (0.0, 4.2, 11.1, 22.2), (1000.0, 1000.0, 950.0, 800.0)
_DN_BP, _DN_V = (0.0, 1.4, 5.6, 22.2), (1200.0, 1000.0, 925.0, 950.0)


def _scale(v: float, bp, vals) -> float:
  if v <= bp[0]:
    return vals[0]
  for i in range(1, len(bp)):
    if v <= bp[i]:
      return vals[i - 1] + (vals[i] - vals[i - 1]) * (v - bp[i - 1]) / (bp[i] - bp[i - 1])
  return vals[-1]


def accel_to_accel_cmd(accel: float, v: float) -> int:
  s = _scale(v, _UP_BP, _UP_V) if accel >= 0 else _scale(v, _DN_BP, _DN_V)
  return int(round(max(-2000.0, min(2000.0, accel * s))))


def accel_cmd_to_accel(cmd: float, v: float) -> float:
  cmd = max(-2000.0, min(2000.0, cmd))
  s = _scale(v, _UP_BP, _UP_V) if cmd >= 0 else _scale(v, _DN_BP, _DN_V)
  return cmd / s


def crz_info_checksum(d: bytes | bytearray) -> int:
  return (0xFF - ((sum(d[:7]) - (d[5] & 0x04) - (d[6] & 0x40)) & 0xFF)) & 0xFF


def yaw_unwrap(a: float) -> float:
  return (a + math.pi) % (2 * math.pi) - math.pi
