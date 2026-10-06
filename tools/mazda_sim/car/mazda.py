"""The whole simulated car: physics, modules, driver, lead traffic, ground truth and sensors."""
from __future__ import annotations

import math

import numpy as np

from ..common.config import SimConfig
from ..common.wire import Frame, Sensors
from .canbus import Networks
from .ecus import Abs, Body, BodyModule, Cluster, Eps, Pcm, Transmission
from .fsc import ForwardCamera
from .radar import Radar
from .road import Road
from .ti import TorqueInterceptor
from .vehicle import STEER_RATIO, WHEELBASE, Vehicle

T_IDXS = [10.0 * (i / 32) ** 2 for i in range(33)]
X_IDXS = [192.0 * (i / 32) ** 2 for i in range(33)]
LEAD_T_IDXS = [0.0, 2.0, 4.0, 6.0, 8.0, 10.0]
DEVICE_AHEAD_OF_CG = 0.8     # m, comma on the windshield
FRONT_BUMPER_AHEAD_OF_CG = 2.5
CAR_LENGTH = 5.1
BASE_LAT, BASE_LON = 32.7531, -117.2095  # where the track is laid out


class Lead:
  """A scripted car ahead in the ego lane."""

  def __init__(self, mode: str, speed_kph: float, gap: float):
    self.mode = mode
    self.cruise = speed_kph / 3.6
    self.gap0 = gap
    self.s = 0.0
    self.v = 0.0
    self.a = 0.0
    self.t = 0.0
    self.active = mode != "none"

  def reset(self, s_ego: float, v_ego: float) -> None:
    self.s = s_ego + self.gap0 + CAR_LENGTH
    self.v = self.cruise if self.mode != "stopgo" else max(v_ego, 0.0)
    self.t = 0.0
    self.active = self.mode != "none"

  def step(self, dt: float) -> None:
    if not self.active:
      return
    self.t += dt
    target_a = 0.0
    if self.mode == "cruise":
      target_a = 0.5 * (self.cruise - self.v)
    elif self.mode == "stopgo":
      phase = self.t % 50.0          # cruise 20 s, stop, wait 10 s, go
      if phase < 20.0:
        target_a = 0.5 * (self.cruise - self.v)
      elif self.v > 0.0 and phase < 40.0:
        target_a = -2.0
      elif phase < 40.0:
        target_a = 0.0
      else:
        target_a = 1.5
    elif self.mode == "brake":
      phase = self.t % 40.0          # cruise 25 s, emergency brake to a stop, wait, go
      if phase < 25.0:
        target_a = 0.5 * (self.cruise - self.v)
      elif phase < 33.0:
        target_a = -5.0 if self.v > 0.0 else 0.0
      else:
        target_a = 2.0
    self.a = max(-6.0, min(2.5, target_a))
    self.v = max(0.0, self.v + self.a * dt)
    if self.v == 0.0 and self.a < 0.0:
      self.a = 0.0
    self.s += self.v * dt


class Driver:
  """The person in the driver's seat.

  manual: applies exactly the torque, gas and brake you give it (hands off otherwise)
  hold:   hands resting on the wheel: resists quick movements a little, plus whatever you add
  auto:   a virtual driver that keeps the lane and a speed whenever openpilot is not steering or not in control
          of speed, so you can test engagement on the move without driving by hand. Your inputs still add on top.
  """

  MAX_NM = 6.0

  def __init__(self):
    self.mode = "auto"
    self.torque_input = 0.0      # -1..1, positive left
    self.gas_input = 0.0
    self.brake_input = 0.0
    self.auto_speed_kph = 0.0    # 0: the road's speed limit
    self.angle_ref = 0.0
    self.e_int = 0.0
    self.torque_nm = 0.0
    self.gas = 0.0
    self.brake = 0.0

  def update(self, dt: float, car: MazdaCar, op_steering: bool, op_long: bool) -> None:
    veh = car.veh
    user_nm = self.torque_input * self.MAX_NM
    hands = 0.0
    if self.mode == "hold":
      self.angle_ref += (veh.steer.angle - self.angle_ref) * min(1.0, dt / 1.5)
      hands = -0.6 * (veh.steer.angle - self.angle_ref) - 0.05 * veh.steer.rate
    elif self.mode == "auto" and not op_steering:
      hands = self._lane_keep_torque(car)
    target = user_nm + hands
    self.torque_nm += (target - self.torque_nm) * min(1.0, dt / 0.05)
    self.torque_nm = max(-self.MAX_NM, min(self.MAX_NM, self.torque_nm))

    gas, brake = self.gas_input, self.brake_input
    if self.mode == "auto" and not op_long and gas == 0.0 and brake == 0.0 and veh.gear == "D" and not car.pcm.engaged:
      gas, brake = self._speed_keep(car)
    self.gas, self.brake = gas, brake

  def _lane_keep_torque(self, car: MazdaCar) -> float:
    """A practiced driver: feed-forward the torque the curve needs, correct offset and heading on top."""
    veh = car.veh
    road = car.road
    v = veh.speed
    s, lat, _ = car.road_pos
    lane_off = road.lane_offset(car.lane)
    _, _, h, _ = road.pose_at(s, lane_off)
    k = float(np.mean(road.poses_at(s + v * np.array([0.3, 0.5, 0.7]), lane_off)[3]))
    k_lane = k / (1.0 - lane_off * k) if abs(1.0 - lane_off * k) > 1e-3 else k
    e = lat - lane_off
    psi = (veh.yaw - h + math.pi) % (2 * math.pi) - math.pi
    if v > 4.0:
      self.e_int = max(-25.0, min(25.0, self.e_int + e * 0.01))
      a_des = v * v * k_lane - 0.5 * e - 1.3 * v * psi - 0.08 * self.e_int
      a_des = max(-6.0, min(6.0, a_des))
      gain = 1.0 + veh.steer.assist_gain(v)
      ff = car.cfg.plant.align_nm_per_mps2 * a_des / gain
      sw_target = math.atan(WHEELBASE * a_des / (v * v)) * STEER_RATIO * (1.0 + 0.0015 * v * v)
      return max(-self.MAX_NM, min(self.MAX_NM, ff + 0.6 * (sw_target - veh.steer.angle) - 0.08 * veh.steer.rate))
    # walking pace: pure pursuit on the lane center, holding the wheel at the angle it needs
    look = max(4.0, 1.2 * v)
    x, y, _, _ = road.pose_at(s + look, lane_off)
    dx, dy = x - veh.x, y - veh.y
    c, sn = math.cos(veh.yaw), math.sin(veh.yaw)
    xl, yl = dx * c + dy * sn, -dx * sn + dy * c
    curv = 2.0 * yl / max(xl * xl + yl * yl, 1.0)
    sw_target = math.atan(curv * WHEELBASE) * STEER_RATIO
    return max(-self.MAX_NM, min(self.MAX_NM, 1.5 * (sw_target - veh.steer.angle) - 0.15 * veh.steer.rate))

  def _speed_keep(self, car: MazdaCar) -> tuple[float, float]:
    veh = car.veh
    target = (self.auto_speed_kph or car.road.speed_limit_mph * 1.609) / 3.6
    # slow for the curves ahead: 2.2 m/s^2 lateral, braking at 2 m/s^2 to reach them
    road = car.road
    ahead = np.arange(0.0, max(40.0, veh.speed ** 2 / 4.0 + 20.0), 2.0)
    k = np.abs(road.poses_at(car.road_pos[0] + ahead, road.lane_offset(car.lane))[3])
    v_curve = np.sqrt(2.2 / np.maximum(k, 1e-4))
    target = min(target, float(np.min(np.sqrt(v_curve ** 2 + 2.0 * 2.0 * ahead))))
    lead = car.lead_rel()
    a = 0.8 * (target - veh.speed)
    if lead is not None:
      d, vl = lead
      a = min(a, 0.25 * (d - (5.0 + 1.6 * veh.speed)) + 0.8 * (vl - veh.speed))
    if a > 0:
      return min(0.6, a / 3.0), 0.0
    if a < -0.25:
      return 0.0, min(0.8, -a / 7.0)
    return 0.0, 0.0


class MazdaCar:
  def __init__(self, cfg: SimConfig):
    self.cfg = cfg
    self.veh = Vehicle(cfg.plant)
    self.body = Body()
    self.road = Road.preset(cfg.world.track, cfg.world.lanes, cfg.world.lane_width)
    self.lane = cfg.world.lanes - 1
    self.lead = Lead(cfg.world.lead, cfg.world.lead_speed_kph, cfg.world.lead_gap_m)
    self.driver = Driver()
    self.t = 0.0
    self.road_hint: int | None = None
    self.road_pos = (0.0, 0.0, 0)
    self.op = {}                  # latest openpilot state from the comma, for the driver model and the dashboard

    self.net = Networks()
    car = cfg.car
    self.pcm = self.net.add(Pcm(self.veh, self.body, car))
    self.tcm = self.net.add(Transmission(self.veh, self.body))
    self.abs = self.net.add(Abs(self.veh))
    self.eps = self.net.add(Eps(self.veh, cfg.plant))
    self.bcm = self.net.add(BodyModule(self.veh, self.body))
    self.cluster = self.net.add(Cluster())
    self.radar = self.net.add(Radar(self.veh, self.body, self.lead_rel))
    self.fsc = self.net.add(ForwardCamera(lambda: self.road.speed_limit_mph, lambda: 2))
    self.ti = self.net.add(TorqueInterceptor(cfg.plant, car.ti_version))
    self.radar.powered = not car.no_mrcc
    self.fsc.powered = not car.no_fsc
    self.ti.powered = car.torque_interceptor
    self.reset()

  # ---- lifecycle ----
  def reset(self) -> None:
    s0 = 5.0
    x, y, h, _ = self.road.pose_at(s0, self.road.lane_offset(self.lane))
    self.veh.reset(x, y, h, self.cfg.world.start_speed_kph / 3.6)
    self.road_hint = None
    self._update_road_pos()
    self.lead.reset(self.road_pos[0], self.veh.v)

  def set_ignition(self, on: bool) -> None:
    self.body.ignition = on
    for ecu in self.net.ecus:
      if ecu is self.ti:
        self.ti.power(on and self.cfg.car.torque_interceptor)
      elif ecu is self.fsc:
        self.fsc.power(on and not self.cfg.car.no_fsc)
      elif ecu is self.radar:
        if on and not self.radar.powered and not self.cfg.car.no_mrcc:
          self.radar.warmup = self.radar.BOOT_T
        self.radar.powered = on and not self.cfg.car.no_mrcc
      else:
        ecu.powered = on

  def _update_road_pos(self) -> None:
    s, lat, i = self.road.project(self.veh.x, self.veh.y, self.road_hint)
    self.road_hint = i
    self.road_pos = (s, lat, i)

  def lead_rel(self) -> tuple[float, float] | None:
    """Bumper-to-bumper distance and speed of the lead car, if one is within radar range."""
    if not self.lead.active:
      return None
    s_ego = self.road_pos[0]
    d = self.lead.s - s_ego
    if self.road.closed:
      d %= self.road.length
    gap = d - CAR_LENGTH
    if 0.0 < gap < 180.0:
      return gap, self.lead.v
    return None

  # ---- one 10 ms step ----
  def tick(self, dt: float, inbound: list[Frame]) -> list[Frame]:
    self.t += dt
    self.net.deliver(inbound)
    op_steering = bool(self.op.get("latActive")) and bool(self.op.get("connected"))
    op_long = bool(self.op.get("longActive")) and bool(self.op.get("connected"))
    self.driver.update(dt, self, op_steering, op_long)
    self.veh.steer.t_driver = self.driver.torque_nm
    self.ti.set_driver_torque(self.driver.torque_nm)
    self.pcm.driver_gas = self.driver.gas
    self.pcm.driver_brake = self.driver.brake
    self.veh.gear = self.body.gear

    out = self.net.tick(dt) if self.body.ignition else []
    self.veh.steer.t_ti = self.ti.injected_nm
    self.veh.step(dt)
    self.lead.step(dt)
    self._update_road_pos()
    return out

  # ---- outputs for the comma ----
  def sensors(self, t_us: int) -> Sensors:
    v = self.veh
    north, east = v.y, v.x
    lat = BASE_LAT + north / 111_111.0
    lon = BASE_LON + east / (111_111.0 * math.cos(math.radians(BASE_LAT)))
    bearing = (90.0 - math.degrees(v.yaw)) % 360.0
    c, s = math.cos(v.yaw), math.sin(v.yaw)
    v_east = v.v * c - v.vy * s
    v_north = v.v * s + v.vy * c
    return Sensors(t_us, (v.ax, -v.ay, -9.81), (0.0, 0.0, -v.r),
                   (lat, lon, 5.0, v.speed, bearing, v_north, v_east))

  def truth(self) -> dict:
    """The road ahead, the plan a good driver would follow, and the lead, all in the comma's frame
    (x forward, y right, z down, origin at the device) on the model's sampling grids."""
    v = self.veh
    road = self.road
    s0, lat, _ = self.road_pos
    lane_off = road.lane_offset(self.lane)
    yaw0 = v.yaw
    c0, s0n = math.cos(yaw0), math.sin(yaw0)
    dev_x = v.x + DEVICE_AHEAD_OF_CG * c0
    dev_y = v.y + DEVICE_AHEAD_OF_CG * s0n

    def to_dev(px: np.ndarray, py: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
      dx, dy = px - dev_x, py - dev_y
      return dx * c0 + dy * s0n, -(-dx * s0n + dy * c0)

    # longitudinal plan: speed limit, curve speed (2 m/s^2 lateral) and the lead
    v_cap = max(road.speed_limit_mph * 0.44704, 3.0)
    lead = self.lead_rel()
    dt = 0.05
    n = int(10.0 / dt) + 1
    ts = np.arange(n) * dt
    ss, vs, as_ = np.zeros(n), np.zeros(n), np.zeros(n)
    sp, vp = 0.0, v.speed
    lead_d = lead[0] if lead else None
    lead_v = lead[1] if lead else 0.0
    # curve speed limit along the road ahead, looked up rather than recomputed per step
    probe_s = np.arange(0.0, 400.0, 2.0)
    probe_k = np.abs(road.poses_at(s0 + probe_s)[3])
    for k in range(n):
      ss[k], vs[k] = sp, vp
      look = sp + max(5.0, vp * 1.5)
      kmax = max(float(np.interp(look, probe_s, probe_k)), 1e-4)
      v_tgt = min(v_cap, math.sqrt(2.0 / kmax))
      a = max(-2.5, min(1.5, 0.6 * (v_tgt - vp)))
      if lead_d is not None:
        d = lead_d + lead_v * ts[k] - sp
        a = min(a, 0.3 * (d - (4.0 + 1.6 * vp)) + 0.9 * (lead_v - vp))
        a = max(-4.0, a)
      as_[k] = a
      vp = max(0.0, vp + a * dt)
      sp += vp * dt

    # lateral plan: what a smooth driver does from where the car is now, simulated in road coordinates.
    # Starts at the car's own position and heading, returns to the lane center (about 3 s), follows the road.
    k_ref = road.poses_at(s0 + probe_s)[3]
    k_lane = k_ref / np.maximum(1.0 - lane_off * k_ref, 0.2)
    e = lat - lane_off                              # m, left of the lane center
    psi = (yaw0 - road.pose_at(s0, lane_off)[2] + math.pi) % (2 * math.pi) - math.pi
    wn, zeta = 1.2, 0.9
    s_rel = 0.0
    yaw_w = yaw0
    # the plan starts from how the car is turning now and eases into the turn it wants (as a net's plan does;
    # openpilot's get_curvature_from_plan relies on plan[0]'s yaw rate being the car's own)
    kappa = max(-0.2, min(0.2, v.r / max(v.speed, 1.0)))
    s_arr, e_arr, yaws_w, yawrates_w = np.zeros(n), np.zeros(n), np.zeros(n), np.zeros(n)
    for k in range(n):
      s_arr[k], e_arr[k], yaws_w[k] = s_rel, e, yaw_w
      vk = max(vs[k], 0.0)
      k_road = float(np.interp(s_rel, probe_s, k_lane))
      v_eff = max(vk, 3.0)
      k_corr = -(wn * wn * e + 2.0 * zeta * wn * v_eff * math.sin(psi)) / (v_eff * v_eff)
      kappa_des = k_road + max(-0.2, min(0.2, k_corr))
      yawrates_w[k] = vk * kappa
      kappa += (kappa_des - kappa) * min(1.0, dt / 0.3)
      ds = vk * math.cos(psi) / max(1.0 - e * k_road, 0.2) * dt
      e += vk * math.sin(psi) * dt
      psi += vk * kappa * dt - k_road * ds
      yaw_w += vk * kappa * dt
      s_rel += ds
    cx, cy, _, _ = road.poses_at(s0 + s_arr, lane_off + e_arr)
    xs_w = cx + DEVICE_AHEAD_OF_CG * np.cos(yaws_w)
    ys_w = cy + DEVICE_AHEAD_OF_CG * np.sin(yaws_w)
    x_d, y_d = to_dev(np.interp(T_IDXS, ts, xs_w), np.interp(T_IDXS, ts, ys_w))
    x_d[0], y_d[0] = 0.0, 0.0
    yaw_d = -(np.interp(T_IDXS, ts, yaws_w) - yaw0)             # device frame: positive right
    yaw_rate_d = -np.interp(T_IDXS, ts, yawrates_w)
    e0 = lat - lane_off
    ph = np.array([road.pose_at(s0, lane_off)[2]])

    # lane lines and road edges on the X grid
    def line(offset: float) -> list[float]:
      d = np.linspace(-2.0, 220.0, 240)
      qx, qy, _, _ = road.poses_at(s0 + d, offset)
      lx, ly = to_dev(qx, qy)
      keep = np.concatenate([[True], np.diff(lx) > 0])
      keep &= np.maximum.accumulate(lx) == lx
      lx, ly = lx[keep], ly[keep]
      return np.interp(X_IDXS, lx, ly).round(3).tolist()

    w = road.lane_width
    near_left, near_right = lane_off + w / 2, lane_off - w / 2
    left_edge, right_edge = road.edge_offsets()
    lanes = [line(near_left + w), line(near_left), line(near_right), line(near_right - w)]
    probs = [0.6 if near_left + w <= left_edge + 0.1 else 0.05, 0.98, 0.98,
             0.6 if near_right - w >= right_edge - 0.1 else 0.05]
    edges = [line(left_edge), line(right_edge)]

    lead_out = None
    if lead is not None:
      d, vl = lead
      dist = d + FRONT_BUMPER_AHEAD_OF_CG - DEVICE_AHEAD_OF_CG
      lead_out = {"x": [max(0.0, dist + (vl - v.speed) * t) for t in LEAD_T_IDXS], "y": [0.0] * 6,
                  "v": [vl] * 6, "a": [self.lead.a] * 6, "prob": 0.99}

    return {
      "t": round(self.t, 3), "v": v.speed, "a": v.ax, "yawRate": -v.r,
      "plan": {"x": x_d.round(3).tolist(), "y": y_d.round(3).tolist(),
               "v": np.interp(T_IDXS, ts, vs).round(3).tolist(), "a": np.interp(T_IDXS, ts, as_).round(3).tolist(),
               "yaw": yaw_d.round(4).tolist(), "yawRate": yaw_rate_d.round(4).tolist()},
      "lanes": lanes, "laneProbs": probs, "edges": edges, "lead": lead_out,
      "laneOffset": round(-e0, 3), "headingErr": round(float(-(ph[0] - yaw0)), 4),
      "speedLimit": road.speed_limit_mph,
    }

  # ---- dashboard ----
  def telemetry(self) -> dict:
    v = self.veh
    s, lat, _ = self.road_pos
    lane_off = self.road.lane_offset(self.lane)
    return {
      "t": round(self.t, 2),
      "pose": {"x": round(v.x, 2), "y": round(v.y, 2), "yaw": round(v.yaw, 4), "s": round(s, 1),
               "laneOffset": round(lat - lane_off, 3)},
      "speedKph": round(v.speed * 3.6, 2), "ax": round(v.ax, 2), "ay": round(v.ay, 2), "yawRate": round(v.r, 4),
      "steer": {"angleDeg": round(v.steering_wheel_deg, 2), "rateDeg": round(v.steering_rate_deg, 1),
                "driverNm": round(v.steer.t_driver, 3), "tiNm": round(v.steer.t_ti, 3), "alignNm": round(v.steer.t_align, 3)},
      "pedals": {"gas": round(self.driver.gas, 2), "brake": round(self.driver.brake, 2)},
      "body": {"gear": self.body.gear, "ignition": self.body.ignition, "seatbelt": self.body.seatbelt,
               "doorOpen": self.body.door_open, "blinker": self.body.blinker, "highBeams": self.body.high_beams,
               "buttons": sorted(self.body.buttons.held)},
      "driver": {"mode": self.driver.mode, "autoSpeedKph": self.driver.auto_speed_kph},
      "ti": self.ti.telemetry(), "eps": self.eps.telemetry(), "pcm": self.pcm.telemetry(),
      "radar": self.radar.telemetry(), "fsc": self.fsc.telemetry(),
      "cluster": {"handsOnWarning": self.cluster.hands_on_warning, "ldw": self.cluster.ldw},
      "lead": None if self.lead_rel() is None else {"gap": round(self.lead_rel()[0], 1), "v": round(self.lead.v * 3.6, 1)},
      "relayIntercept": self.net.relay_intercept,
      "track": {"name": self.cfg.world.track, "length": round(self.road.length), "lanes": self.road.lanes,
                "laneWidth": self.road.lane_width, "lane": self.lane, "speedLimitMph": self.road.speed_limit_mph},
    }
