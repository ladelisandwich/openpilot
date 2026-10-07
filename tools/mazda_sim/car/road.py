"""Road geometry: closed tracks built from straight and arc blocks.

The same block list drives the lite world and the MetaDrive map, so the ground truth handed to openpilot is the
road that is rendered. Frame: x east, y north, heading counter-clockwise from x. Lanes are counted from the left
edge of the road; the car drives in the rightmost lane unless told otherwise.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

STEP = 0.5  # m between samples


@dataclass(frozen=True)
class Straight:
  length: float


@dataclass(frozen=True)
class Arc:
  radius: float
  angle_deg: float
  left: bool


def _rounded_rectangle(a: float, b: float, r: float, s_bend: tuple[float, float] | None = None) -> list:
  """Closed loop of four left corners. An S-bend (radius, angle) on both long sides keeps it closed."""
  def side(length: float) -> list:
    if s_bend is None:
      return [Straight(length)]
    rs, ang = s_bend
    chord = 2 * rs * math.sin(math.radians(ang))
    rest = max(10.0, (length - 2 * chord) / 2)
    return [Straight(rest), Arc(rs, ang, False), Arc(rs, ang, True), Arc(rs, ang, True), Arc(rs, ang, False), Straight(rest)]
  corner = Arc(r, 90.0, True)
  return [*side(a), corner, Straight(b), corner, *side(a), corner, Straight(b), corner]


TRACKS = {
  # the layout tools/sim uses: 60 m straights, 120 m radius corners
  "loop": dict(blocks=_rounded_rectangle(60.0, 60.0, 120.0), speed_limit_mph=45),
  # interstate-like: long straights, gentle sweepers and an S-bend on each long side
  "highway": dict(blocks=_rounded_rectangle(1400.0, 600.0, 650.0, s_bend=(900.0, 8.0)), speed_limit_mph=70),
  # back road: tight corners and S-bends, where low-speed TI torque and DRIVER_OVER show up
  "twisty": dict(blocks=_rounded_rectangle(260.0, 120.0, 55.0, s_bend=(45.0, 35.0)), speed_limit_mph=35),
  # city blocks: 90 degree turns at 12 m radius, the turn-apex case
  "city": dict(blocks=_rounded_rectangle(140.0, 90.0, 12.0), speed_limit_mph=25),
  "straight": dict(blocks=[Straight(20000.0)], speed_limit_mph=65),
}


class Road:
  def __init__(self, blocks: list, lanes: int = 2, lane_width: float = 3.6, speed_limit_mph: int = 65):
    self.blocks = blocks
    self.lanes = lanes
    self.lane_width = lane_width
    self.speed_limit_mph = speed_limit_mph
    xs, ys, hs, ks = [0.0], [0.0], [0.0], [0.0]
    x = y = h = 0.0
    for blk in blocks:
      if isinstance(blk, Straight):
        n = max(1, int(round(blk.length / STEP)))
        ds = blk.length / n
        for _ in range(n):
          x += ds * math.cos(h)
          y += ds * math.sin(h)
          xs.append(x)
          ys.append(y)
          hs.append(h)
          ks.append(0.0)
      else:
        k = (1.0 if blk.left else -1.0) / blk.radius
        length = blk.radius * math.radians(blk.angle_deg)
        n = max(1, int(round(length / STEP)))
        ds = length / n
        for _ in range(n):
          # exact arc step
          h2 = h + k * ds
          x += (math.sin(h2) - math.sin(h)) / k
          y += (-math.cos(h2) + math.cos(h)) / k
          h = h2
          xs.append(x)
          ys.append(y)
          hs.append(h)
          ks.append(k)
    self.x = np.array(xs)
    self.y = np.array(ys)
    self.h = np.unwrap(np.array(hs))
    self.k = np.array(ks)
    seg = np.hypot(np.diff(self.x), np.diff(self.y))
    self.s = np.concatenate([[0.0], np.cumsum(seg)])
    self.length = float(self.s[-1])
    self.closed = math.hypot(self.x[-1] - self.x[0], self.y[-1] - self.y[0]) < 1.0
    if self.closed:  # drop the duplicate end point
      self.x, self.y, self.h, self.k, self.s = self.x[:-1], self.y[:-1], self.h[:-1], self.k[:-1], self.s[:-1]
    self.n = len(self.x)

  @classmethod
  def preset(cls, name: str, lanes: int = 2, lane_width: float = 3.6) -> Road:
    t = TRACKS.get(name, TRACKS["highway"])
    return cls(t["blocks"], lanes, lane_width, t["speed_limit_mph"])

  # --- lanes: lateral offset of a lane center from the road's left edge reference line (positive left) ---
  def lane_offset(self, lane: int) -> float:
    return -(lane + 0.5) * self.lane_width

  def edge_offsets(self) -> tuple[float, float]:
    return 0.0, -self.lanes * self.lane_width

  def wrap(self, s: float) -> float:
    if self.closed:
      return s % self.length
    return min(max(s, 0.0), self.length)

  def idx_at(self, s: float) -> int:
    s = self.wrap(s)
    return int(min(self.n - 1, max(0, round(s / (self.length / max(1, self.n - (0 if self.closed else 1)))))))

  def pose_at(self, s: float, offset: float = 0.0) -> tuple[float, float, float, float]:
    """(x, y, heading, curvature of the reference line) at arc length s, shifted left by offset."""
    s = self.wrap(s)
    i = int(np.searchsorted(self.s, s, side="right")) - 1
    i = max(0, min(i, self.n - 1))
    j = (i + 1) % self.n if self.closed else min(i + 1, self.n - 1)
    s0 = self.s[i]
    s1 = self.s[j] if j > i else (self.length if self.closed else self.s[i] + 1e-6)
    t = 0.0 if s1 <= s0 else (s - s0) / (s1 - s0)
    x = self.x[i] + (self.x[j] - self.x[i]) * t
    y = self.y[i] + (self.y[j] - self.y[i]) * t
    hj = self.h[j]
    hi = self.h[i]
    if j < i:  # wrapped: unwrap the end heading
      hj = hi + ((hj - hi + math.pi) % (2 * math.pi) - math.pi)
    h = hi + (hj - hi) * t
    k = float(self.k[j])
    return x - offset * math.sin(h), y + offset * math.cos(h), h, k

  def poses_at(self, s: np.ndarray, offset: float = 0.0) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Vectorized pose_at: (x, y, heading, curvature) arrays."""
    s = np.asarray(s, dtype=float)
    if self.closed:
      ss = np.concatenate([self.s, [self.length]])
      xs = np.concatenate([self.x, [self.x[0]]])
      ys = np.concatenate([self.y, [self.y[0]]])
      hs = np.concatenate([self.h, [self.h[-1] + ((self.h[0] - self.h[-1] + math.pi) % (2 * math.pi) - math.pi)]])
      ks = np.concatenate([self.k, [self.k[0]]])
      s = np.mod(s, self.length)
    else:
      ss, xs, ys, hs, ks = self.s, self.x, self.y, self.h, self.k
      s = np.clip(s, 0.0, self.length)
    x = np.interp(s, ss, xs)
    y = np.interp(s, ss, ys)
    h = np.interp(s, ss, hs)
    i = np.clip(np.searchsorted(ss, s, side="right"), 0, len(ks) - 1)
    k = ks[i]
    return x - offset * np.sin(h), y + offset * np.cos(h), h, k

  def project(self, x: float, y: float, hint: int | None = None) -> tuple[float, float, int]:
    """(s, lateral offset from the reference line, positive left, sample index) of the closest point."""
    if hint is None:
      d2 = (self.x - x) ** 2 + (self.y - y) ** 2
      i = int(np.argmin(d2))
    else:
      w = 120
      idx = (np.arange(hint - w, hint + w) % self.n) if self.closed else np.clip(np.arange(hint - w, hint + w), 0, self.n - 1)
      d2 = (self.x[idx] - x) ** 2 + (self.y[idx] - y) ** 2
      i = int(idx[int(np.argmin(d2))])
    h = self.h[i]
    dx, dy = x - self.x[i], y - self.y[i]
    along = dx * math.cos(h) + dy * math.sin(h)
    lat = -dx * math.sin(h) + dy * math.cos(h)
    return self.wrap(self.s[i] + along), lat, i

  def heading_at(self, s: float) -> float:
    return self.pose_at(s)[2]
