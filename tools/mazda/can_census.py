#!/usr/bin/env python3
"""What is on the CAN buses, and what the unnamed messages might be (Mazda). Read-only: it only reads rlogs.

  python3 tools/mazda/can_census.py                    every route from the last 7 days
  python3 tools/mazda/can_census.py --days 1           the last day
  python3 tools/mazda/can_census.py <route> ...        just these (a prefix is enough)
  python3 tools/mazda/can_census.py --watch 0x9e,0x45b <route>
                                                       a change-by-change timeline of those addresses with the
                                                       driving context at each change (for action recordings)

Writes --out (default /data/can_census.txt). For every (bus, address) seen it reports the rate and, byte by byte,
what the data looks like: static, a counter, a checksum, a multiplex page index (the other bytes are then
described page by page), a noisy sensor reading, an analog value or flags. Every analog byte, byte pair and
flipping bit is correlated against the signals the DBC already decodes (speed, RPM, gas, steering angle, rate and
torque, brake pressure, yaw and lateral accel derived from them, the blinkers, brake, cruise, gear, lead) and a
match is printed as a hint. Addresses the DBC names normally are listed by shape only; the unnamed ones
(NEW_MSG_*, MSG_*, 2017_*, MOB*) and the ones the DBC lacks get the full treatment.

What it cannot do: name a body signal that never changed in the recording. Locks, wipers, hazards, the stalks,
the wheel buttons, windows and the like need a recording in which they are used at known times; --watch on the
candidate addresses then shows which bytes moved at those times.
"""
import argparse
import os
import re
import sys
import time

import numpy as np

from openpilot.tools.lib.logreader import _LogFileReader
from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value

try:
  from openpilot.system.hardware.hw import Paths
  DEFAULT_ROOT = Paths.log_root()
except Exception:
  DEFAULT_ROOT = "/data/media/0/realdata"

DBC_2017 = DBC("mazda_2017")
UNNAMED = re.compile(r"^(NEW_MSG|MSG_|2017_|MOB|last_byte|CAM_IDK|CAM_EMPTY|CAM_Empty|SPEED_TBD|TWO_STATES|STEER_RELATED|" +
                     r"RPM_RELATED|GEAR_RELATED|CAM_LANEMAYBE|CHECK_AND_TEMP)", re.I)
BUSES = (0, 1, 2)                   # the car, the TI, the camera side; our own echoes (128+) and rejects (192+) are skipped
CAP_FRAMES = 150_000                # per address: beyond this the stored frames are thinned uniformly (memory)
MAX_GRID_POINTS = 600_000           # the common timeline the correlations run on
MIN_CORR_ANALOG = 0.75
MIN_CORR_BIT = 0.85
# known signals (address, DBC signal, continuous or discrete)
KNOWN = [(0x202, "SPEED", "c"), (0x202, "RPM", "c"), (0x202, "PEDAL_GAS", "c"), (0x82, "STEER_ANGLE", "c"),
         (0x241, "STEER_ANGLE_RATE", "c"), (0x78, "BRAKE_PRESSURE", "c"), (0x240, "STEER_TORQUE_SENSOR", "c"),
         (0x240, "STEER_TORQUE_MOTOR", "c"), (0x215, "FL", "c"), (0x215, "FR", "c"), (0x165, "BRAKE_ON", "d"),
         (0x165, "ACC_ACTIVE", "d"), (0x9a, "LEFT_BLINK", "d"), (0x9a, "RIGHT_BLINK", "d"), (0x9a, "HIGH_BEAMS", "d"),
         (0x9a, "WIPER_LO", "d"), (0x9a, "WIPER_HI", "d"), (0x91, "TURN_LEFT_SWITCH", "d"), (0x91, "TURN_RIGHT_SWITCH", "d"),
         (0x91, "HAZARD", "d"), (0x228, "GEAR", "d"), (0x43e, "FL", "d"), (0x43e, "FR", "d"), (0x43e, "DOORS_UNLOCKED", "d"),
         (0x21c, "CRZ_ACTIVE", "d"), (0x21c, "RADAR_HAS_LEAD", "d"), (0x241, "LKAS_BLOCK", "d"), (0x340, "DRIVER_SEATBELT", "d")]


def sig(addr: int, dat: bytes, name: str) -> float:
  s = DBC_2017.addr_to_msg[addr].sigs[name]
  raw = get_raw_value(dat, s)
  if s.is_signed and raw & (1 << (s.size - 1)):
    raw -= 1 << s.size
  return raw * s.factor + s.offset


class Capped:
  """Frames of one address: all of them up to CAP_FRAMES, then thinned uniformly so a week fits in memory."""
  def __init__(self):
    self.t, self.d, self.stride, self.n = [], [], 1, 0

  def add(self, t: float, d) -> None:
    self.n += 1
    if self.n % self.stride:
      return
    self.t.append(t)
    self.d.append(d)
    if len(self.t) >= CAP_FRAMES:
      self.t, self.d, self.stride = self.t[::2], self.d[::2], self.stride * 2


class Census:
  def __init__(self):
    self.frames: dict[tuple[int, int], Capped] = {}
    self.known: dict[tuple[int, str], Capped] = {}
    self.known_ok = [k for k in KNOWN if k[0] in DBC_2017.addr_to_msg and k[1] in DBC_2017.addr_to_msg[k[0]].sigs]
    self.by_addr: dict[int, list] = {}
    for addr, name, kind in self.known_ok:
      self.by_addr.setdefault(addr, []).append((name, kind))
    self.t = 0.0
    self.offset = 0.0

  def feed_route(self, segs: list) -> None:
    for _seg, path in segs:
      fn = log_file(path)
      if fn is None:
        continue
      t0, last = None, 0.0
      try:
        for evt in _LogFileReader(fn):
          if evt.which() != "can":
            continue
          t_abs = evt.logMonoTime * 1e-9
          if t0 is None:
            t0 = t_abs
          t = self.offset + (t_abs - t0)
          last = t
          for c in evt.can:
            if c.src not in BUSES:
              continue
            key = (c.src, c.address)
            store = self.frames.get(key)
            if store is None:
              store = self.frames[key] = Capped()
            d = bytes(c.dat)
            store.add(t, d)
            if c.src == 0 and c.address in self.by_addr:
              for name, _kind in self.by_addr[c.address]:
                try:
                  v = sig(c.address, d, name)
                except Exception:
                  continue
                ks = self.known.get((c.address, name))
                if ks is None:
                  ks = self.known[(c.address, name)] = Capped()
                ks.add(t, v)
      except Exception as e:   # a truncated last segment is normal after a power cut
        print(f"  {os.path.basename(path)}: {e}", file=sys.stderr)
      self.offset = last + 1.0
    self.t = self.offset


# ---------------------------------------------------------------------------------------------------------------------
def log_file(seg_path: str):
  for fn in ("rlog.zst", "rlog.bz2", "rlog", "qlog.zst", "qlog.bz2", "qlog"):
    if os.path.isfile(os.path.join(seg_path, fn)):
      return os.path.join(seg_path, fn)
  return None


def routes_under(root: str) -> dict:
  routes = {}
  for name in os.listdir(root):
    path = os.path.join(root, name)
    route, _, seg = name.rpartition("--")
    if os.path.isdir(path) and route and seg.isdigit():
      routes.setdefault(route, []).append((int(seg), path))
  return {r: sorted(s) for r, s in routes.items()}


def hold(grid, ts, vs):
  """The last value at or before each grid time (NaN before the first)."""
  idx = np.searchsorted(ts, grid, side="right") - 1
  ok = idx >= 0
  g = np.full(len(grid), np.nan, dtype=np.float32)
  g[ok] = vs[idx[ok]]
  return g


def corr(a, b) -> float:
  m = ~np.isnan(a) & ~np.isnan(b)
  if m.sum() < 50:
    return 0.0
  x, y = a[m].astype(np.float64), b[m].astype(np.float64)
  sx, sy = x.std(), y.std()
  if sx == 0 or sy == 0:
    return 0.0
  return float(((x - x.mean()) * (y - y.mean())).mean() / (sx * sy))


def is_cycle(col) -> tuple[bool, int]:
  """A multiplex page index: k distinct values, 3 <= k <= 32, walked in one fixed order (99% of the steps between
  consecutive different values; a page may be sent more than once in a row). Two values alternating is a toggle."""
  vals = np.unique(col)
  k = len(vals)
  if k < 3 or k > 32 or len(col) < 10 * k:
    return False, 0
  steps = col[np.concatenate(([True], col[1:] != col[:-1]))]     # consecutive repeats collapsed
  if len(steps) < 5 * k:
    return False, 0
  order = {}
  for v in steps:
    if v not in order:
      order[v] = len(order)
  idx = np.array([order[v] for v in steps])
  return bool(np.mean((np.diff(idx) % k) == 1) > 0.99), k


def best_match(g, knowns, kinds=None) -> tuple[float, str, float]:
  best = (0.0, "", 0.0)
  for name, (kg, kind) in knowns.items():
    if kinds is not None and kind not in kinds:
      continue
    c = corr(g, kg)
    if abs(c) > best[0]:
      best = (abs(c), name, c)
  return best


def describe_bytes(ts, data, grid, knowns, page: str = "") -> tuple[list[str], list[str]]:
  """Shape of each byte of a frame set, and the correlation hints found."""
  n, dlc = data.shape
  desc, hints = [], []
  diffs = (data[1:] != data[:-1]) if n > 1 else np.zeros((0, dlc), dtype=bool)
  skip_pairs = set()
  for b in range(dlc):
    col = data[:, b]
    nd = len(np.unique(col))
    if nd == 1:
      desc.append(f"b{b}=0x{col[0]:02x}")
      skip_pairs.add(b)
      continue
    d = np.diff(col.astype(int)) % 256
    lo, hi = col & 0x0F, col >> 4
    if n > 50 and np.mean(d == 1) > 0.95:
      desc.append(f"b{b}=COUNTER")
      skip_pairs.add(b)
      continue
    ctr_lo = n > 50 and np.mean((np.diff(lo.astype(int)) % 16) == 1) > 0.95
    ctr_hi = n > 50 and np.mean((np.diff(hi.astype(int)) % 16) == 1) > 0.95
    if ctr_lo != ctr_hi:
      which, other, ov = ("lo", "hi", hi) if ctr_lo else ("hi", "lo", lo)
      desc.append(f"b{b}=ctr({which} nibble)+{other}:{len(np.unique(ov))}v")
      skip_pairs.add(b)
      continue
    others = np.delete(diffs, b, axis=1).any(axis=1) if dlc > 1 else np.zeros(n - 1, dtype=bool)
    if nd >= 64 and others.sum() > 20 and np.mean(diffs[:, b][others]) > 0.9 and np.mean(diffs[:, b][~others]) < 0.2:
      desc.append(f"b{b}=CHECKSUM?")
      skip_pairs.add(b)
      continue
    cyc, k = is_cycle(col)
    if cyc and not page:
      desc.append(f"b{b}=MUX({k} pages)")
      skip_pairs.add(b)
      continue
    span = int(col.max()) - int(col.min())
    flips = int(np.sum(diffs[:, b])) if n > 1 else 0
    if nd == 2 and flips > 0.4 * n:
      desc.append(f"b{b}=toggle 0x{int(col.min()):02x}/0x{int(col.max()):02x}")   # an alive bit flipping every frame or two
      continue
    if 2 <= span <= 12 and nd >= 3 and flips > 0.2 * n:
      desc.append(f"b{b}=noisy {int(col.min())}-{int(col.max())}")      # jitters within a narrow band: a sensor reading
    elif nd <= 16:
      desc.append(f"b{b}:{nd}v")
    else:
      desc.append(f"b{b}:{nd}v {int(col.min())}-{int(col.max())}")
    g = hold(grid, ts, col.astype(np.float32))
    if nd >= 6:
      best = best_match(g, knowns)
      if best[0] >= MIN_CORR_ANALOG:
        hints.append(f"{page}b{b} ~ {best[1]} (r={best[2]:+.2f})")
    if nd <= 16:
      for bit in range(8):
        bits = (col >> bit) & 1
        if 0 < bits.mean() < 1 and np.sum(np.diff(bits) != 0) >= 2:
          gb = hold(grid, ts, bits.astype(np.float32))
          best = best_match(gb, knowns)
          if best[0] >= MIN_CORR_BIT:
            hints.append(f"{page}b{b}.bit{bit} ~ {best[1]} (r={best[2]:+.2f})")
  # byte pairs as 16-bit values, both byte orders
  for b in range(dlc - 1):
    if b in skip_pairs or (b + 1) in skip_pairs:
      continue
    for order, val in (("BE", data[:, b].astype(np.int32) * 256 + data[:, b + 1]), ("LE", data[:, b + 1].astype(np.int32) * 256 + data[:, b])):
      if len(np.unique(val)) < 32:
        continue
      best = best_match(hold(grid, ts, val.astype(np.float32)), knowns)
      if best[0] >= 0.85:
        hints.append(f"{page}b{b}-{b + 1}({order}) ~ {best[1]} (r={best[2]:+.2f})")
  return desc, hints


def describe(store: Capped, grid, knowns) -> dict:
  ts = np.array(store.t, dtype=np.float64)
  n = len(ts)
  dlc = max(len(d) for d in store.d)
  data = np.zeros((n, dlc), dtype=np.uint8)
  for i, d in enumerate(store.d):
    data[i, :len(d)] = np.frombuffer(d, dtype=np.uint8)
  rate = store.n / max(ts[-1] - ts[0], 1e-3) if n > 1 else 0.0
  out = {"rate": rate, "dlc": dlc, "n": store.n, "pages": []}
  out["bytes"], out["hints"] = describe_bytes(ts, data, grid, knowns)
  # multiplexed: describe the other bytes page by page
  for b, desc in enumerate(out["bytes"]):
    if desc.startswith(f"b{b}=MUX"):
      col = data[:, b]
      for v in np.unique(col):
        m = col == v
        if m.sum() < 10:
          continue
        pd_, ph = describe_bytes(ts[m], data[m], grid, knowns, page=f"page{int(v)}:")
        out["pages"].append((int(v), int(m.sum()), [x for i, x in enumerate(pd_) if i != b]))
        out["hints"].extend(ph)
      break
  text = bytes(data[0]).decode("ascii", errors="replace")
  if n > 0 and all(32 <= x < 127 for x in data[0]) and len(np.unique(data, axis=0)) <= 2:
    out["ascii"] = text
  return out


def family_note(bus: int, addr: int, data0: bytes) -> str:
  if 0x700 <= addr <= 0x7FF:
    return "diagnostic (UDS) traffic"
  if len(data0) == 8 and data0[2:] == b"\xaa\x55\xaa\x55\xaa\x55":
    return "a node-alive beacon (0xaa55 pattern, byte 0 is the node id)"
  return ""


def write_report(census: Census, out, knowns_used: list[str]) -> None:
  T = census.t
  grid_hz = min(10.0, MAX_GRID_POINTS / max(T, 1.0))
  grid = np.arange(0.0, T, 1.0 / grid_hz)
  knowns = {}
  for (addr, name), store in census.known.items():
    ts, vs = np.array(store.t, dtype=np.float64), np.array(store.d, dtype=np.float32)
    if len(ts) < 10 or vs.std() == 0:
      continue
    kind = next(k for a, s, k in census.known_ok if (a, s) == (addr, name))
    knowns[f"{DBC_2017.addr_to_msg[addr].name}.{name}"] = (hold(grid, ts, vs), kind)
  # derived: lateral accel and yaw rate from angle and speed, longitudinal accel from speed, left-right wheel difference
  if "ENGINE_DATA.SPEED" in knowns and "STEER.STEER_ANGLE" in knowns:
    v, a = knowns["ENGINE_DATA.SPEED"][0], knowns["STEER.STEER_ANGLE"][0]
    knowns["derived.YAW_RATE(angle*speed)"] = (a * v, "c")
    knowns["derived.LAT_ACCEL(angle*speed^2)"] = (a * v * v, "c")
  if "ENGINE_DATA.SPEED" in knowns:
    v = knowns["ENGINE_DATA.SPEED"][0].astype(np.float64)
    dv = np.gradient(np.nan_to_num(v, nan=0.0)) * grid_hz
    k = max(1, int(grid_hz))
    dv = np.convolve(dv, np.ones(k) / k, mode="same")
    knowns["derived.LONG_ACCEL(d speed)"] = (dv.astype(np.float32), "c")
  if "WHEEL_SPEEDS.FL" in knowns and "WHEEL_SPEEDS.FR" in knowns:
    knowns["derived.WHEEL_FL-FR"] = (knowns["WHEEL_SPEEDS.FL"][0] - knowns["WHEEL_SPEEDS.FR"][0], "c")
  knowns_used[:] = sorted(knowns)

  named, unnamed, absent = [], [], []
  for (bus, addr), store in sorted(census.frames.items()):
    name = DBC_2017.addr_to_msg[addr].name if addr in DBC_2017.addr_to_msg else None
    (absent if name is None else (unnamed if UNNAMED.match(name) else named)).append((bus, addr, name, store))
  print(f"{T / 60:.1f} min of CAN; {len(named)} named, {len(unnamed)} unnamed and {len(absent)} undeclared addresses", file=out)
  print(f"correlated against: {', '.join(knowns_used)}\n", file=out)
  all_hints = []
  print(f"=== {len(named)} addresses the DBC names (shape only)", file=out)
  for bus, addr, name, store in named:
    r = describe(store, grid, knowns)
    print(f"bus{bus} 0x{addr:03x} {name:20s} {r['rate']:6.1f} Hz  {' '.join(r['bytes'])}", file=out)
  for title, group in ((f"{len(unnamed)} UNNAMED in the DBC", unnamed), (f"{len(absent)} NOT IN THE DBC", absent)):
    print(f"\n=== {title}", file=out)
    for bus, addr, name, store in group:
      r = describe(store, grid, knowns)
      note = family_note(bus, addr, store.d[0])
      print(f"\nbus{bus} 0x{addr:03x} {name or '-':20s} {r['rate']:6.1f} Hz  n={r['n']}" + (f"   [{note}]" if note else ""), file=out)
      print(f"   bytes: {' '.join(r['bytes'])}", file=out)
      if "ascii" in r:
        print(f"   ascii: {r['ascii']!r}", file=out)
      for v, cnt, pd_ in r["pages"]:
        print(f"   page {v:2d} ({cnt} frames): {' '.join(pd_)}", file=out)
      if name:
        print(f"   DBC declares: {', '.join(DBC_2017.addr_to_msg[addr].sigs.keys())}", file=out)
      for h in r["hints"]:
        print(f"   hint: {h}", file=out)
        all_hints.append(f"bus{bus} 0x{addr:03x} {name or '-'}: {h}")
  print("\n=== every hint, in one place", file=out)
  for h in all_hints:
    print("  " + h, file=out)


# ---------------------------------------------------------------------------------------------------------------------
def watch(segs: list, addrs: list[int], out) -> None:
  """Every change of the given addresses, with the driving context: what to read against an action recording."""
  ctx = {"kph": 0, "gas": 0, "brake": 0, "L": 0, "R": 0, "gear": 0, "hb": 0, "acc": 0, "angle": 0}
  last: dict[int, bytes] = {}
  offset = 0.0
  for _seg, path in segs:
    fn = log_file(path)
    if fn is None:
      continue
    t0, t_last = None, 0.0
    for evt in _LogFileReader(fn):
      if evt.which() != "can":
        continue
      t_abs = evt.logMonoTime * 1e-9
      if t0 is None:
        t0 = t_abs
      t = offset + t_abs - t0
      t_last = t
      for c in evt.can:
        if c.src != 0 and c.address not in addrs:
          continue
        d = bytes(c.dat)
        try:
          if c.address == 0x202:
            ctx["kph"], ctx["gas"] = round(sig(0x202, d, "SPEED")), round(sig(0x202, d, "PEDAL_GAS"))
          elif c.address == 0x165:
            ctx["brake"], ctx["acc"] = int(sig(0x165, d, "BRAKE_ON")), int(sig(0x165, d, "ACC_ACTIVE"))
          elif c.address == 0x9a:
            ctx["L"], ctx["R"], ctx["hb"] = int(sig(0x9a, d, "LEFT_BLINK")), int(sig(0x9a, d, "RIGHT_BLINK")), int(sig(0x9a, d, "HIGH_BEAMS"))
          elif c.address == 0x228:
            ctx["gear"] = int(sig(0x228, d, "GEAR"))
          elif c.address == 0x82:
            ctx["angle"] = round(sig(0x82, d, "STEER_ANGLE"))
        except Exception:
          pass
        if c.address in addrs and c.src in BUSES:
          prev = last.get((c.src, c.address))
          if prev is None:
            print(f"{t:8.2f} bus{c.src} 0x{c.address:03x} {d.hex(' ')}  first   {ctx}", file=out)
          elif prev != d:
            marks = "".join("^^ " if a != b else "   " for a, b in zip(prev, d, strict=False))
            print(f"{t:8.2f} bus{c.src} 0x{c.address:03x} {d.hex(' ')}  {ctx}", file=out)
            print(f"{'':21s}{marks}", file=out)
          last[(c.src, c.address)] = d
    offset = t_last + 1.0


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("routes", nargs="*", help="route names or prefixes; none = every route from the last --days")
  ap.add_argument("--days", type=float, default=7.0)
  ap.add_argument("--root", default=DEFAULT_ROOT)
  ap.add_argument("--out", default="/data/can_census.txt")
  ap.add_argument("--watch", default="", help="comma-separated addresses (hex) to print a change timeline for, to stdout")
  args = ap.parse_args()

  routes = routes_under(args.root)
  if args.routes:
    names = [n for n in routes if any(n.startswith(p) for p in args.routes)]
  else:
    cutoff = time.time() - args.days * 86400  # noqa: TID251  wall clock: compared with the segments' file times
    names = [n for n, segs in routes.items() if max(os.path.getmtime(p) for _, p in segs) >= cutoff]
  names.sort(key=lambda n: max(os.path.getmtime(p) for _, p in routes[n]))
  if not names:
    print("no routes", file=sys.stderr)
    return
  if args.watch:
    addrs = [int(a, 16) for a in args.watch.split(",")]
    for n in names:
      print(f"===== {n}")
      watch(routes[n], addrs, sys.stdout)
    return

  census = Census()
  start = time.monotonic()
  for i, n in enumerate(names, 1):
    print(f"\r  {i}/{len(names)} {n}", end="", file=sys.stderr, flush=True)
    census.feed_route(routes[n])
  print(f"\n  read in {time.monotonic() - start:.0f} s, {len(census.frames)} (bus, address) pairs", file=sys.stderr)
  knowns_used: list[str] = []
  with open(args.out, "w") as out:
    print(f"routes: {' '.join(names)}", file=out)
    write_report(census, out, knowns_used)
  print(f"  -> {args.out}", file=sys.stderr)


if __name__ == "__main__":
  main()
