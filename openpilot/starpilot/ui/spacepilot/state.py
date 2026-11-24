"""Bounded game simulation and input state; no vehicle writes or UI dependencies."""

from dataclasses import dataclass
from enum import StrEnum
import math
import random


class FiveTaps:
  """Five separate releases at a natural pace; use original touch timestamps."""
  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self.count = 0
    self.first = self.last = None

  def tap(self, now: float) -> bool:
    if not math.isfinite(now):
      self.reset()
      return False
    if self.last is not None and 0 <= now - self.last < .04:
      return False
    if self.last is None or not 0 <= now - self.last <= 1.5 or self.first is None or now - self.first > 6.:
      self.count, self.first = 0, now
    self.count += 1
    self.last = now
    if self.count == 5:
      self.reset()
      return True
    return False


class WheelInput:
  """Observe original events once. A release arms each key; held/repeated keys do nothing.

  Unknown pre-admission button state cannot be inferred from an empty event list.
  The first wheel press/release primes a key; its next press can launch the game.
  """
  def __init__(self, stamp: int = 0):
    self.stamp = stamp
    self.armed: set[str] = set()
    self.last_action = -math.inf

  def update(self, stamp: int, events: tuple[tuple[str, bool], ...], now: float) -> bool:
    if stamp <= self.stamp:
      return False
    self.stamp = stamp
    fire = False
    for kind, pressed in events:
      if kind not in ('accelCruise', 'decelCruise'):
        continue
      if not pressed:
        self.armed.add(kind)
      else:
        if kind in self.armed and now - self.last_action >= .12:
          self.last_action, fire = now, True
        self.armed.discard(kind)
    return fire


class Phase(StrEnum):
  READY = 'ready'
  FLIGHT = 'flight'
  OVER = 'over'


@dataclass
class Gate:
  x: float
  center: float
  scored: bool = False


class Flight:
  """1072 × 480 C4 canvas, fixed 120 Hz physics, capped catch-up and storage."""
  WIDTH, HEIGHT = 1072., 480.
  SHIP_X, RADIUS = 270., 12.
  TOP, BOTTOM = 64., 438.
  GATE_WIDTH, GAP = 72., 174.
  STEP = 1 / 120

  def __init__(self, seed: int = 0):
    self.rng = random.Random(seed)
    self.best = 0
    self.phase = Phase.READY
    self.y, self.vy = 250., 0.
    self.score = 0
    self.age = self.over_age = self.accumulator = 0.
    self.distance = 0.
    self.gates: list[Gate] = []
    self.last_input = -math.inf

  def act(self, now: float) -> bool:
    if not math.isfinite(now) or now - self.last_input < .12 or (self.phase == Phase.OVER and self.over_age < .65):
      return False
    self.last_input = now
    if self.phase != Phase.FLIGHT:
      self.phase = Phase.FLIGHT
      self.y, self.vy, self.score, self.distance, self.accumulator = 250., 0., 0, 0., 0.
      self.gates = [Gate(1100., 250.)]
    self.vy = -268.
    return True

  def _crash(self) -> None:
    self.phase, self.over_age = Phase.OVER, 0.
    self.best = max(self.best, self.score)

  def update(self, dt: float) -> None:
    if not math.isfinite(dt) or dt <= 0:
      return
    dt = min(dt, .05)
    self.age += dt
    if self.phase == Phase.OVER:
      self.over_age += dt
    if self.phase != Phase.FLIGHT:
      return
    self.accumulator += dt
    while self.accumulator >= self.STEP and self.phase == Phase.FLIGHT:
      self.accumulator -= self.STEP
      speed = min(238., 168. + self.score * 3.)
      self.vy = min(420., self.vy + 760. * self.STEP)
      self.y += self.vy * self.STEP
      self.distance += speed * self.STEP
      if self.y - self.RADIUS <= self.TOP or self.y + self.RADIUS >= self.BOTTOM:
        self._crash()
        break
      for gate in self.gates:
        gate.x -= speed * self.STEP
        if gate.x - self.RADIUS < self.SHIP_X < gate.x + self.GATE_WIDTH + self.RADIUS:
          if abs(self.y - gate.center) + self.RADIUS >= self.GAP / 2:
            self._crash()
            break
        if not gate.scored and gate.x + self.GATE_WIDTH + self.RADIUS < self.SHIP_X:
          gate.scored = True
          self.score += 1
      if self.phase != Phase.FLIGHT:
        break
      self.gates = [gate for gate in self.gates if gate.x > -self.GATE_WIDTH]
      if self.gates[-1].x < 835.:
        center = max(170., min(330., self.gates[-1].center + self.rng.uniform(-80., 80.)))
        self.gates.append(Gate(self.gates[-1].x + 325., center))

  def close(self) -> None:
    self.gates.clear()
    self.phase = Phase.READY
