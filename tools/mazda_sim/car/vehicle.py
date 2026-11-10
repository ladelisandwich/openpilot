"""The physical CX-9: chassis dynamics, steering column and EPS, powertrain and brakes.

Everything is in SI units at the steering column (Nm, rad of steering-wheel angle) unless a name says otherwise.

Lateral: a dynamic bicycle model (linear tires with a soft saturation) that blends into the kinematic model below
3 m/s. Steering: the steering wheel is not commanded by angle, it moves because of torques, as in the real car:

  J*th'' = T_driver + T_motor - T_align - T_center - B(v)*th' - friction

  T_motor  = G(v) * (T_driver + T_ti) + T_lkas      EPS assist on what the torque sensor reports, plus LKAS
  T_ti     = what the Torque Interceptor fakes on the sensor line (0 unless it is running)
  T_align  = tire aligning torque, proportional to the front axle's lateral force
  T_center = low-speed self-centering (caster and kingpin), fading out by walking pace

So openpilot's torque reaches the road the same two ways it does on the car, through the TI's fake sensor torque
amplified by the speed-dependent assist curve, and through the stock LKAS channel above 52 km/h.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from ..common.config import PlantOptions

G = 9.81
STEER_RATIO = 17.6
WHEELBASE = 3.1
CENTER_TO_FRONT = WHEELBASE * 0.41
CENTER_TO_REAR = WHEELBASE - CENTER_TO_FRONT
TIRE_STIFFNESS_FACTOR = 0.7
MAX_ROAD_WHEEL_ANGLE = math.radians(36.0)  # ~ 1.75 turns lock to lock with a 17.6 ratio
SUBSTEPS = 10                               # 1 kHz inner integration for the column


def interp(x: float, xp: list[float], fp: list[float]) -> float:
  if x <= xp[0]:
    return fp[0]
  for i in range(1, len(xp)):
    if x <= xp[i]:
      return fp[i - 1] + (fp[i] - fp[i - 1]) * (x - xp[i - 1]) / (xp[i] - xp[i - 1])
  return fp[-1]


def clip(x: float, lo: float, hi: float) -> float:
  return lo if x < lo else hi if x > hi else x


@dataclass
class Pedals:
  gas: float = 0.0     # 0..1 driver accelerator
  brake: float = 0.0   # 0..1 driver brake


class Steering:
  """Steering wheel, column, torque sensor and EPS motor."""
  J = 0.045          # kg m^2: wheel, column and the EPS motor reflected through its gear
  K_CENTER = 12.0    # Nm per rad of road-wheel angle at standstill
  SCRUB_NM = 1.2     # extra static friction when parked

  def __init__(self, plant: PlantOptions, mass: float):
    self.p = plant
    self.mass = mass
    self.angle = 0.0         # rad, steering wheel
    self.rate = 0.0          # rad/s
    self.t_driver = 0.0      # Nm, physical driver torque on the wheel
    self.t_ti = 0.0          # Nm, fake torque the TI adds to the sensor signal
    self.t_lkas = 0.0        # Nm, EPS motor torque from the stock LKAS channel
    self.t_assist = 0.0      # Nm, EPS assist (filtered)
    self.t_align = 0.0

  def assist_gain(self, v: float) -> float:
    return interp(v * 3.6, self.p.assist_bp_kph, self.p.assist_gain)

  @property
  def sensor_eps(self) -> float:
    """What the EPS's torque sensor reports: the driver plus whatever the TI injects."""
    return self.t_driver + self.t_ti

  @property
  def t_motor(self) -> float:
    return self.t_assist + self.t_lkas

  @property
  def road_wheel_angle(self) -> float:
    return self.angle / STEER_RATIO

  def substep(self, dt: float, v: float, front_lat_accel: float) -> None:
    target_assist = self.assist_gain(v) * self.sensor_eps
    self.t_assist += (target_assist - self.t_assist) * min(1.0, dt / 0.02)

    self.t_align = self.p.align_nm_per_mps2 * front_lat_accel
    low = max(0.0, 1.0 - abs(v) / 5.0)
    t_center = self.K_CENTER * self.road_wheel_angle * low
    damping = 0.15 + 0.9 * min(abs(v) / 20.0, 1.0)
    friction = self.p.column_friction_nm + self.SCRUB_NM * max(0.0, 1.0 - abs(v) / 2.0)

    t_net = self.t_driver + self.t_motor - self.t_align - t_center - damping * self.rate
    # Coulomb friction: holds the wheel still until the net torque beats it
    if abs(self.rate) < 1e-3 and abs(t_net) <= friction:
      self.rate = 0.0
      return
    t_net -= friction * math.copysign(1.0, self.rate if abs(self.rate) >= 1e-3 else t_net)
    self.rate += t_net / self.J * dt
    self.angle += self.rate * dt
    lim = MAX_ROAD_WHEEL_ANGLE * STEER_RATIO
    if abs(self.angle) > lim:
      self.angle = math.copysign(lim, self.angle)
      self.rate = 0.0


class Vehicle:
  """Planar chassis + powertrain. Frame: x east, y north, yaw counter-clockwise from x; vy and r positive left."""

  def __init__(self, plant: PlantOptions):
    self.p = plant
    m = plant.mass_kg
    self.m = m
    ref_mass, ref_wb, ref_cf = 1326.0 + 136.0, 2.70, 0.4
    self.Iz = 2500.0 * m * WHEELBASE ** 2 / (ref_mass * ref_wb ** 2)
    self.Cf = 192150 * TIRE_STIFFNESS_FACTOR * m / ref_mass * (CENTER_TO_REAR / WHEELBASE) / (1 - ref_cf)
    self.Cr = 202500 * TIRE_STIFFNESS_FACTOR * m / ref_mass * (CENTER_TO_FRONT / WHEELBASE) / ref_cf
    self.mu = 0.95

    self.steer = Steering(plant, m)
    self.x = 0.0
    self.y = 0.0
    self.yaw = 0.0
    self.v = 0.0          # m/s, longitudinal (negative in reverse)
    self.vy = 0.0         # m/s, lateral at the CG
    self.r = 0.0          # rad/s yaw rate
    self.ax = 0.0         # m/s^2 longitudinal accel (last step)
    self.ay = 0.0         # m/s^2 lateral accel (last step), positive left
    self.front_force = 0.0
    self.odometer = 0.0

    # powertrain / brakes, set by the PCM and ABS models each tick
    self.drive_force = 0.0     # N, from engine (driver gas or ACC)
    self.brake_decel = 0.0     # m/s^2 requested by brakes (driver or ACC)
    self.hold = False          # electric hold at standstill (ACC hold / auto hold)
    self.gear = "D"

  def reset(self, x: float, y: float, yaw: float, v: float) -> None:
    self.x, self.y, self.yaw, self.v = x, y, yaw, v
    self.vy = self.r = self.ax = self.ay = 0.0
    self.steer.angle = self.steer.rate = 0.0

  @property
  def speed(self) -> float:
    return abs(self.v)

  @property
  def standstill(self) -> bool:
    return abs(self.v) < 0.01

  def _lateral(self, dt: float) -> None:
    v = self.v
    delta = self.steer.road_wheel_angle
    a, b, L, m = CENTER_TO_FRONT, CENTER_TO_REAR, WHEELBASE, self.m
    fmax_f = self.mu * m * G * b / L
    fmax_r = self.mu * m * G * a / L

    # kinematic below ~1 m/s, dynamic above 3 m/s, blended in between
    w_dyn = clip((abs(v) - 1.0) / 2.0, 0.0, 1.0)
    r_kin = v * math.tan(delta) / L
    vy_kin = r_kin * b

    if w_dyn > 0.0:
      vv = max(abs(v), 1.0) * (1 if v >= 0 else -1)
      alpha_f = (self.vy + a * self.r) / vv - delta
      alpha_r = (self.vy - b * self.r) / vv
      fyf = -fmax_f * math.tanh(self.Cf * alpha_f / fmax_f)
      fyr = -fmax_r * math.tanh(self.Cr * alpha_r / fmax_r)
      dvy = (fyf + fyr) / m - v * self.r
      dr = (a * fyf - b * fyr) / self.Iz
      vy_dyn = self.vy + dvy * dt
      r_dyn = self.r + dr * dt
    else:
      fyf = m * (b / L) * v * r_kin
      vy_dyn, r_dyn = vy_kin, r_kin

    self.vy = w_dyn * vy_dyn + (1 - w_dyn) * vy_kin
    self.r = w_dyn * r_dyn + (1 - w_dyn) * r_kin
    fyf_kin = m * (b / L) * v * r_kin
    self.front_force = w_dyn * fyf + (1 - w_dyn) * fyf_kin
    self.ay = v * self.r

  def _longitudinal(self, dt: float) -> None:
    m = self.m
    sign = 1.0 if self.v >= 0 else -1.0
    drag = 0.6 * self.v * self.v + 0.012 * m * G + (150.0 if self.gear == "D" and abs(self.v) > 0.5 else 0.0)
    creep = 0.0
    if self.gear in ("D", "R") and self.brake_decel <= 0.0 and not self.hold:
      creep = 650.0 * max(0.0, 1.0 - abs(self.v) / 2.2) * (1.0 if self.gear == "D" else -1.0)
    drive = self.drive_force * (-1.0 if self.gear == "R" else 1.0) if self.gear in ("D", "R") else 0.0
    f = drive + creep
    a_free = f / m
    # resistances and brakes always oppose motion; at a stop they only hold
    resist = (drag / m) + self.brake_decel
    if self.standstill:
      if self.hold or abs(a_free) <= resist:
        self.v = 0.0
        self.ax = 0.0
        return
      a = a_free - math.copysign(resist, a_free)
    else:
      a = a_free - sign * resist
    v_new = self.v + a * dt
    if self.v != 0.0 and (v_new * self.v) < 0.0:   # brakes stop the car, never reverse it
      v_new = 0.0
    if self.hold and abs(v_new) < 0.05:
      v_new = 0.0
    self.ax = (v_new - self.v) / dt
    self.v = v_new

  def step(self, dt: float) -> None:
    h = dt / SUBSTEPS
    for _ in range(SUBSTEPS):
      front_lat_accel = self.front_force / (self.m * CENTER_TO_REAR / WHEELBASE)
      self.steer.substep(h, abs(self.v), front_lat_accel)
      self._lateral(h)
    self._longitudinal(dt)

    c, s = math.cos(self.yaw), math.sin(self.yaw)
    self.x += (self.v * c - self.vy * s) * dt
    self.y += (self.v * s + self.vy * c) * dt
    self.yaw = (self.yaw + self.r * dt + math.pi) % (2 * math.pi) - math.pi
    self.odometer += abs(self.v) * dt

  # --- convenience for ECUs ---
  @property
  def steering_wheel_deg(self) -> float:
    return math.degrees(self.steer.angle)

  @property
  def steering_rate_deg(self) -> float:
    return math.degrees(self.steer.rate)

  def max_drive_force(self) -> float:
    return min(7200.0, 150_000.0 / max(abs(self.v), 1.0))
