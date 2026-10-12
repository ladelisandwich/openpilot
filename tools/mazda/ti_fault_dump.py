#!/usr/bin/env python3
"""When did the Torque Interceptor stop taking commands, and what was openpilot asking of it just before?
(Mazda + TI.) Read-only: it only reads rlogs.

  python3 tools/mazda/ti_fault_dump.py                  newest route
  python3 tools/mazda/ti_fault_dump.py <route>          e.g. 00000074--a469916bb3 (a prefix is enough)
  python3 tools/mazda/ti_fault_dump.py --last 5         the newest 5 routes

A fault is the TI dropping out of RUN or DRIVER_OVER into OFF or DISCOVER, or raising a VIOL or ERROR
code; everything until it is back in RUN with no codes counts as the same fault. For each one it prints:
  - the codes, the TI firmware version, and how long the TI took to come back to RUN with no codes
    (and through which states)
  - how long the TI command had sat near its ceiling beforehand (the ceiling in force at the time,
    read back from carOutput: 800 counts, or 600 on builds before c4cd66c), and the longest
    gap between TI command frames on the bus (a late or missing frame)
  - the last 1.5 s at 10 Hz, the last 0.1 s frame by frame, and 0.3 s after
Then every stretch where the TI command sat at 90% of its ceiling or more (--near) for 0.3 s or longer,
with the speed and how it ended, so a fault can be told apart from the TI simply being asked for a lot.
"""
import argparse
import os
import sys
from collections import deque

from openpilot.tools.lib.logreader import _LogFileReader
from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value

try:
  from openpilot.system.hardware.hw import Paths
  DEFAULT_ROOT = Paths.log_root()
except Exception:
  DEFAULT_ROOT = "/data/media/0/realdata"

DBC_2017 = DBC("mazda_2017")
TI_FEEDBACK, TI_COMMAND, STOCK_COMMAND, STEER_TORQUE, STEER_RATE = 0x24A, 0x249, 0x243, 0x240, 0x241
TX_ECHO_BUS1 = 129       # the panda echoes what it actually put on bus 1 as src 128 + bus
STATES = {0: "DISC", 1: "OFF", 2: "DRVOV", 3: "RUN"}
HISTORY_S = 5.0          # context kept before each fault
POST_S = 0.3             # frames kept after it
NEAR = 0.9               # "near the ceiling": this fraction of the ceiling in force
NEAR_MIN_S = 0.3
MPH = 2.23694
HEADER = "        dt   mph  TIcmd ceil  stock state vi er rd spr TIdrv EPSsen BLK   eff lat"


def lkas_request(dat: bytes) -> int:
  return (((dat[0] & 0x0F) << 8) | dat[1]) - 2048


def decode(addr: int, dat: bytes) -> dict:
  msg = DBC_2017.addr_to_msg[addr]
  out = {}
  for name, sig in msg.sigs.items():
    raw = get_raw_value(dat, sig)
    if sig.is_signed:
      raw -= ((raw >> (sig.size - 1)) & 1) * (1 << sig.size)
    out[name] = raw * sig.factor + sig.offset
  return out


def routes_under(root: str):
  routes = {}
  for name in os.listdir(root):
    path = os.path.join(root, name)
    route, _, seg = name.rpartition("--")
    if os.path.isdir(path) and route and seg.isdigit():
      routes.setdefault(route, []).append((int(seg), path))
  return routes


def log_file(seg_path: str):
  for fn in ("rlog.zst", "rlog.bz2", "rlog"):
    if os.path.isfile(os.path.join(seg_path, fn)):
      return os.path.join(seg_path, fn)
  return None


class Sample:
  __slots__ = ("t", "v", "ti", "stock", "ceil", "state", "viol", "err", "rd", "drv", "spare", "esen", "blk", "eff", "lat")


def scan(name: str, segs: list, near: float):
  """One pass over a route: faults (with context), TI state transitions and near-ceiling stretches."""
  t0 = None
  t = 0.0
  commit = ""
  v, lat = 0.0, False
  ti = stock = 0
  ceil = 800.0                   # the TI's ceiling in force, from carOutput
  fb = None                      # (state, viol, err, ramp_down, driver, version, spare) from the latest TI_FEEDBACK
  prev_fb = None
  esen, blk, eff = 0.0, 0, 0
  hist = deque()                 # Samples covering HISTORY_S, one per TI command frame
  gaps_echo, gaps_send = deque(), deque()   # (t, gap) between TI command frames, last 2 s
  last_echo = last_send = None
  faults, transitions, stretches = [], [], []
  open_posts = []                # faults still collecting frames after the event
  outage = None                  # the fault in progress: until the TI is back in RUN with no codes
  skipped = 0                    # segments with no rlog
  cur = None                     # open near-ceiling stretch: [start, vmin, vmax, peak, ceiling]

  for i, seg in enumerate(segs, 1):
    print(f"\rreading {name} segment {i}/{len(segs)}", end="", file=sys.stderr, flush=True)
    fn = log_file(seg)
    if fn is None:
      skipped += 1
      continue
    try:
      reader = _LogFileReader(fn)
    except Exception as e:
      print(f"\n  (skipped {os.path.basename(seg)}: {e})", file=sys.stderr)
      continue
    for evt in reader:
      w = evt.which()
      if t0 is None:
        t0 = evt.logMonoTime * 1e-9
      t = evt.logMonoTime * 1e-9 - t0
      if w == "initData":
        commit = evt.initData.gitCommit[:7]
      elif w == "carState":
        v = evt.carState.vEgo
      elif w == "carControl":
        lat = evt.carControl.latActive
      elif w == "carOutput":
        # card publishes the previous frame's output here, so it is the TI's only when it matches the TI
        # command last sent (otherwise it may be the stock channel's, normalised by 800)
        ao = evt.carOutput.actuatorsOutput
        if abs(ao.torque) > 0.05 and ao.torqueOutputCan == ti:
          ceil = abs(ao.torqueOutputCan / ao.torque)
      elif w == "can":
        for c in evt.can:
          if c.src == 1 and c.address == TI_FEEDBACK and len(c.dat) >= 8:
            d = c.dat
            # RAMP_DOWN only counts from TI version 2, as in carstate
            fb = (d[3], d[4], d[5], d[6] if d[2] > 1 else 0, d[0] - 127, d[2], d[7])
            if prev_fb is not None and fb[:3] != prev_fb[:3]:
              transitions.append((t, prev_fb[0], fb[0], fb[1], fb[2]))
              to_off = fb[0] in (0, 1) and prev_fb[0] in (2, 3)   # stopped taking commands (not OFF <-> DISC)
              new_code = bool(fb[1] or fb[2]) and fb[1:3] != prev_fb[1:3]
              if outage is not None and fb[0] == 3 and not fb[1] and not fb[2]:
                outage = None
              elif (to_off or new_code) and outage is not None:
                outage["to"] = fb[0] if to_off else outage["to"]
                outage["viol"], outage["err"] = outage["viol"] | fb[1], outage["err"] | fb[2]
              elif to_off or new_code:
                gaps, last_frame = (gaps_echo, last_echo) if last_echo is not None else (gaps_send, last_send)
                recent = [g for tt, g in gaps if t - tt <= 2.0] + ([t - last_frame] if last_frame is not None else [])
                outage = {"t": t, "v": v, "from": prev_fb[0], "to": fb[0], "viol": fb[1], "err": fb[2], "version": fb[5],
                          "hist": list(hist), "gaps": recent, "gap_src": "on the bus" if last_echo is not None else "as sent",
                          "post": []}
                faults.append(outage)
                open_posts.append(outage)
                if cur is not None:
                  stretches.append((cur[0], t, cur[1], cur[2], cur[3], cur[4], f"TI fault #{len(faults)}"))
                  cur = None
            prev_fb = fb
          elif c.src == TX_ECHO_BUS1 and c.address == TI_COMMAND:
            if last_echo is not None:
              gaps_echo.append((t, t - last_echo))
              while gaps_echo and t - gaps_echo[0][0] > 2.0:
                gaps_echo.popleft()
            last_echo = t
          elif c.src == 0 and c.address == STEER_TORQUE:
            esen = decode(STEER_TORQUE, c.dat)["STEER_TORQUE_SENSOR"]
          elif c.src == 0 and c.address == STEER_RATE:
            r = decode(STEER_RATE, c.dat)
            blk, eff = int(r["LKAS_BLOCK"]), int(r["LKAS_EFFECTIVE"])
      elif w == "sendcan":
        for c in evt.sendcan:
          if c.src == 0 and c.address == STOCK_COMMAND:
            stock = lkas_request(c.dat)
          elif c.src == 1 and c.address == TI_COMMAND:
            ti = lkas_request(c.dat)
            if last_send is not None:
              gaps_send.append((t, t - last_send))
              while gaps_send and t - gaps_send[0][0] > 2.0:
                gaps_send.popleft()
            last_send = t
            s = Sample()
            s.t, s.v, s.ti, s.stock, s.lat, s.ceil = t, v, ti, stock, lat, ceil
            s.state, s.viol, s.err, s.rd, s.drv, _, s.spare = fb if fb else (-1, 0, 0, 0, 0, 0, 0)
            s.esen, s.blk, s.eff = esen, blk, eff
            hist.append(s)
            while hist and t - hist[0].t > HISTORY_S:
              hist.popleft()
            for f in list(open_posts):
              if t - f["t"] <= POST_S:
                f["post"].append(s)
              else:
                open_posts.remove(f)
            if abs(ti) >= near * ceil:
              if cur is None:
                cur = [t, v, v, abs(ti), ceil]
              else:
                cur[1], cur[2], cur[3], cur[4] = min(cur[1], v), max(cur[2], v), max(cur[3], abs(ti)), min(cur[4], ceil)
            elif cur is not None:
              if s.rd:
                how = "TI ramp-down"
              elif s.state == 2 or abs(s.drv) > 15:
                how = "driver's hands"
              elif s.state != 3:
                how = f"TI {STATES.get(s.state, s.state)}"
              elif not lat:
                how = "lateral off"
              else:
                how = "command eased"
              stretches.append((cur[0], t, cur[1], cur[2], cur[3], cur[4], how))
              cur = None
  print(file=sys.stderr)
  if cur is not None:
    stretches.append((cur[0], t, cur[1], cur[2], cur[3], cur[4], "route end"))
  version = prev_fb[5] if prev_fb else None
  return commit, faults, stretches, transitions, version, t, skipped


def recovery(fault, transitions):
  """Seconds until the TI is back in RUN with no codes, and the states it went through."""
  path, back = [], None
  for (tt, _, b, viol, err) in transitions:
    if tt <= fault["t"]:
      continue
    path.append(f"{STATES.get(b, b)}@{tt - fault['t']:.1f}s" + (f"(VIOL {viol:02x} ERROR {err:02x})" if viol or err else ""))
    if b == 3 and not viol and not err:
      back = tt - fault["t"]
      break
  if len(path) > 6:
    path = path[:5] + [f"... {len(path) - 6} more", path[-1]]
  return back, path


def held(h, frac):
  """Seconds the TI command was at `frac` of its ceiling or more, in total and in the longest run."""
  total = longest = run = 0.0
  for a, b in zip(h, h[1:], strict=False):
    dt = b.t - a.t
    if abs(a.ti) >= frac * a.ceil:
      total += dt
      run += dt
      longest = max(longest, run)
    else:
      run = 0.0
  return total, longest


def row(s: Sample, t_ref: float) -> str:
  cols = [f"{s.t - t_ref:+6.2f}", f"{s.v * MPH:5.1f}", f"{s.ti:6d}", f"{s.ceil:4.0f}", f"{s.stock:6d}", f"{STATES.get(s.state, '?'):>5}",
          f"{s.viol:02x}", f"{s.err:02x}", f"{s.rd:2d}", f"{s.spare:3d}", f"{s.drv:5d}", f"{s.esen:6.0f}", f"{s.blk:3d}",
          f"{s.eff:5d}", f"{'Y' if s.lat else '-':>3}"]
  return "    " + " ".join(cols)


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("route", nargs="?")
  ap.add_argument("--root", default=DEFAULT_ROOT)
  ap.add_argument("--last", type=int, default=1, help="newest N routes (ignored when a route is given)")
  ap.add_argument("--near", type=float, default=NEAR, help=f"fraction of the TI's ceiling that counts as near it (default {NEAR})")
  args = ap.parse_args()
  if not os.path.isdir(args.root):
    sys.exit(f"no log folder at {args.root} (pass --root)")
  routes = routes_under(args.root)
  if not routes:
    sys.exit(f"no routes under {args.root}")
  if args.route:
    names = [n for n in routes if n == args.route or n.startswith(args.route) or n.endswith(args.route)]
    if not names:
      sys.exit(f"route {args.route} not found under {args.root}")
  else:
    names = sorted(routes, key=lambda n: os.path.getmtime(max(routes[n])[1]))[-args.last:]

  for name in names:
    segs = [p for _, p in sorted(routes[name])]
    commit, faults, stretches, transitions, version, dur, skipped = scan(name, segs, args.near)
    takeovers = sum(1 for tr in transitions if tr[2] == 2 and tr[1] != 2)
    print(f"\n=== route {name}  build {commit}  {dur / 60:.1f} min  TI firmware version {version}  " +
          f"TI faults {len(faults)}  driver takeovers (DRIVER_OVER) {takeovers}")
    if skipped:
      print(f"    ({skipped} of {len(segs)} segments have no rlog on the device and were skipped)")
    for n, f in enumerate(faults, 1):
      h = f["hist"]
      back, path = recovery(f, transitions)
      head = f"\n#{n}  t={f['t']:.2f}s  {f['v'] * MPH:.0f} mph  TI {STATES.get(f['from'], f['from'])} -> {STATES.get(f['to'], f['to'])}"
      codes = f"  VIOL 0x{f['viol']:02x}  ERROR 0x{f['err']:02x}"
      back_txt = f"{back:.1f} s" if back is not None else "(not before the route ended)"
      print(head + codes + "  back to RUN with no codes after " + back_txt + ("  via " + " > ".join(path) if path else ""))
      if not h:
        print("    (no TI command frames before it)\n" + HEADER + "\n    -- after --")
        for s in f["post"][::10]:
          print(row(s, f["t"]))
        continue
      parts = []
      for frac in (0.75, 0.9, 0.97):
        tot, lng = held(h, frac)
        parts.append(f">={frac:.0%}: {tot:.1f} s (longest {lng:.1f})")
      print(f"    in the {HISTORY_S:.0f} s before, with the ceiling at {h[-1].ceil:.0f}, TI command " + ", ".join(parts))
      gap_ms = max(f['gaps'], default=0.0) * 1000
      print(f"    peak stock command {max(abs(s.stock) for s in h)}; longest gap between TI command frames in the 2 s before: " +
            f"{gap_ms:.0f} ms ({f['gap_src']})")
      print(HEADER)
      rows, next_t = [], h[-1].t - 1.5
      tail = [s for s in h if s.t > h[-1].t - 0.1]
      for s in h[:len(h) - len(tail)]:
        if s.t >= next_t:
          rows.append(s)
          next_t = s.t + 0.1
      for s in rows + tail:
        print(row(s, f["t"]))
      print("    -- after --")
      for s in f["post"][::10]:
        print(row(s, f["t"]))
    long_ones = [s for s in stretches if s[1] - s[0] >= NEAR_MIN_S]
    print(f"\n  TI command at {args.near:.0%}+ of its ceiling for {NEAR_MIN_S} s or longer: {len(long_ones)} times")
    if long_ones:
      print("      start    dur     mph   peak ceil  ended by")
      for (a, b, vmin, vmax, peak, c, how) in long_ones:
        print(f"    {a:7.1f} {b - a:5.1f}s  {vmin * MPH:3.0f}-{vmax * MPH:<3.0f}  {peak:5d} {c:4.0f}  {how}")


if __name__ == "__main__":
  main()
