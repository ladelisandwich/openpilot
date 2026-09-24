#!/usr/bin/env python3
"""Why did openpilot say "Take Control / Turn Exceeds Steering Limit"? (Mazda + Torque Interceptor)

Reads a route's rlogs on the comma and prints, for every steerSaturated alert, what openpilot asked for,
what the car did and what the Torque Interceptor was doing, then totals the causes.

  python3 tools/mazda/ti_steer_report.py                 newest route
  python3 tools/mazda/ti_steer_report.py <route>         e.g. 0000012a--4b3c2d1e0f
  python3 tools/mazda/ti_steer_report.py --last 3        newest 3 routes
  python3 tools/mazda/ti_steer_report.py --segments 10   only the last 10 minutes of the newest route
  python3 tools/mazda/ti_steer_report.py --root <dir>    logs somewhere else

Only reads logs. It changes nothing on the car or the device.
"""
import argparse
import os
import sys
from collections import Counter, deque

from openpilot.tools.lib.logreader import _LogFileReader

try:
  from openpilot.system.hardware.hw import Paths
  DEFAULT_ROOT = Paths.log_root()
except Exception:
  DEFAULT_ROOT = "/data/media/0/realdata"

TI_FEEDBACK = 0x24A    # bus 1, from the TI
TI_COMMAND = 0x249     # bus 1, CAM_LKAS2 from openpilot
STOCK_COMMAND = 0x243  # bus 0, CAM_LKAS from openpilot
TI_STATES = {0: "DISCOVER", 1: "OFF", 2: "DRIVER_OVER", 3: "RUN"}
LIMITED_LAT_ACCEL = 0.2  # m/s^2: controls clipped the model's curvature by more than this (clip_curvature:
                         # 3 m/s^2 +- road roll, 0.2 1/m, lateral jerk)
WINDOW_S = 1.0           # look-back at each alert
SAME_ALERT_S = 2.0       # an alert that drops out and returns within this is the same alert


def lkas_request(dat: bytes) -> int:
  return (((dat[0] & 0x0F) << 8) | dat[1]) - 2048


def find_segments(root: str):
  routes = {}
  for name in os.listdir(root):
    path = os.path.join(root, name)
    if not os.path.isdir(path) or "--" not in name:
      continue
    route, _, seg = name.rpartition("--")
    if not seg.isdigit():
      continue
    routes.setdefault(route, []).append((int(seg), path))
  for segs in routes.values():
    segs.sort()
  return routes


def log_file(seg_path: str):
  for fn in ("rlog.zst", "rlog.bz2", "rlog", "qlog.zst", "qlog.bz2", "qlog"):
    p = os.path.join(seg_path, fn)
    if os.path.isfile(p):
      return p
  return None


class Route:
  def __init__(self, name):
    self.name = name
    self.t0 = None
    self.commit = self.branch = None
    self.v = 0.0
    self.pressed = False
    self.driver_torque = 0
    self.lat_active = False
    self.lac_output = 0.0
    self.lac_saturated = False
    self.actual_la = 0.0
    self.desired_la = 0.0
    self.ti_state = None
    self.ti_ramp = 0
    self.ti_viol = self.ti_error = 0
    self.ti_cmd = 0
    self.stock_cmd = 0
    self.alert_prev = False
    self.alert_prev_goat = False
    self.alert_last_t = -1e9
    self.model_dc = 0.0
    self.ctl_dc = 0.0
    self.ceilings = Counter()  # TI_STEER_MAX of the build, read back from carOutput
    self.hist = deque()
    self.alerts = []
    self.causes = Counter()
    self.engaged_s = 0.0
    self.ceiling_s = 0.0
    self.peak_ti = 0
    self.dropouts = Counter()
    self.faults = Counter()   # (VIOL, ERROR) codes the TI reported while engaged
    self.last_ctl_t = None
    self.used_qlog = False

  def feed(self, evt):
    w = evt.which()
    t = evt.logMonoTime * 1e-9
    if self.t0 is None:
      self.t0 = t

    if w == "initData":
      self.commit = evt.initData.gitCommit[:7]
      self.branch = evt.initData.gitBranch
    elif w == "carState":
      cs = evt.carState
      self.v, self.pressed, self.driver_torque = cs.vEgo, cs.steeringPressed, cs.steeringTorque
    elif w == "carControl":
      self.lat_active = evt.carControl.latActive
    elif w == "can":
      for c in evt.can:
        if c.address == TI_FEEDBACK and c.src == 1 and len(c.dat) >= 7:
          d = c.dat
          state, ramp = d[3], (d[6] if d[2] > 1 else 0)  # carstate honours RAMP_DOWN only for version > 1
          if self.lat_active and self.ti_state == 3 and state != 3:
            self.dropouts[TI_STATES.get(state, str(state))] += 1
          if self.lat_active and ramp and not self.ti_ramp:
            self.dropouts["RAMP_DOWN"] += 1
          self.ti_state, self.ti_ramp, self.ti_viol, self.ti_error = state, ramp, d[4], d[5]
          if self.lat_active and (d[4] or d[5]):
            self.faults[(d[4], d[5])] += 1
    elif w == "sendcan":
      for c in evt.sendcan:
        if c.address == TI_COMMAND and c.src == 1:
          self.ti_cmd = lkas_request(c.dat)
          self.peak_ti = max(self.peak_ti, abs(self.ti_cmd))
        elif c.address == STOCK_COMMAND and c.src == 0:
          self.stock_cmd = lkas_request(c.dat)
    elif w == "modelV2":
      self.model_dc = evt.modelV2.action.desiredCurvature
      self.desired_la = self.model_dc * max(self.v, 0.3) ** 2
    elif w == "carOutput":
      ao = evt.carOutput.actuatorsOutput
      if abs(ao.torque) > 0.05 and self.ti_state == 3 and not self.ti_ramp:
        self.ceilings[int(round(abs(ao.torqueOutputCan / ao.torque) / 50.0)) * 50] += 1
    elif w == "controlsState":
      ctl = evt.controlsState
      lcs = ctl.lateralControlState
      lac = getattr(lcs, lcs.which())
      self.lac_output = float(getattr(lac, "output", 0.0))
      self.lac_saturated = bool(getattr(lac, "saturated", False))
      self.actual_la = ctl.curvature * max(self.v, 0.3) ** 2
      self.ctl_dc = ctl.desiredCurvature
      dt = 0.0 if self.last_ctl_t is None else min(t - self.last_ctl_t, 0.1)
      self.last_ctl_t = t
      if self.lat_active:
        self.engaged_s += dt
        if abs(self.ti_cmd) >= 0.97 * self.ceiling():
          self.ceiling_s += dt
      self.hist.append((t, self.ti_state, self.ti_ramp, self.ti_cmd, self.lac_output, self.v))
      while self.hist and t - self.hist[0][0] > WINDOW_S:
        self.hist.popleft()
    elif w == "onroadEvents":
      self.alert_update(t, any(e.name == "steerSaturated" for e in evt.onroadEvents), goat=False)
    elif w == "starpilotOnroadEvents":  # the same alert with StarPilot's goat sound
      self.alert_update(t, any(e.name == "goatSteerSaturated" for e in evt.starpilotOnroadEvents.events), goat=True)

  def alert_update(self, t, active: bool, goat: bool):
    was_active = self.alert_prev or self.alert_prev_goat
    if goat:
      self.alert_prev_goat = active
    else:
      self.alert_prev = active
    if active and not was_active and t - self.alert_last_t > SAME_ALERT_S:
      self.on_alert(t)
    if active or was_active:
      self.alert_last_t = t

  def ceiling(self) -> int:
    # carOutput reports the TI command both raw and normalised by TI_STEER_MAX, so the ratio is
    # the build's own ceiling (600 before the parity change, 800 after). 800 until it is seen.
    return self.ceilings.most_common(1)[0][0] if self.ceilings else 800

  def on_alert(self, t):
    window = list(self.hist)
    states = {s for _, s, _, _, _, _ in window if s is not None}
    ramped = any(r for _, _, r, _, _, _ in window)
    ti_peak = max((abs(c) for _, _, _, c, _, _ in window), default=0)
    limited_la = (abs(self.model_dc) - abs(self.ctl_dc)) * max(self.v, 0.3) ** 2
    if not states:
      cause = "TI state unknown (no TI_FEEDBACK yet)"
    elif ramped or (states - {3}):
      what = "RAMP_DOWN" if ramped else "/".join(TI_STATES.get(s, str(s)) for s in sorted(states - {3}))
      cause = f"TI stopped steering ({what})"
    elif limited_la > LIMITED_LAT_ACCEL:
      cause = "openpilot's own limit clipped the turn (safety limits, not the TI)"
    elif ti_peak >= 0.97 * self.ceiling():
      cause = f"TI at its {self.ceiling()}-count ceiling"
    else:
      cause = "TI below its ceiling (still climbing into the turn)"
    self.causes[cause] += 1
    self.alerts.append((t - self.t0, self.v, self.desired_la, self.actual_la, self.lac_output, self.ti_cmd, ti_peak,
                        TI_STATES.get(self.ti_state, str(self.ti_state)), self.ti_ramp, self.driver_torque, cause))


def report(route: Route):
  print(f"\n=== route {route.name}   build {route.commit} {route.branch}" + ("   (qlog only: coarse)" if route.used_qlog else ""))
  at_ceiling = 100 * route.ceiling_s / max(route.engaged_s, 1e-6)
  print(f"engaged {route.engaged_s / 60:.1f} min, TI peak {route.peak_ti} counts, TI at its ceiling {at_ceiling:.1f}% of engaged time")
  if route.dropouts:
    print("TI stopped steering while engaged: " + ", ".join(f"{k} x{v}" for k, v in route.dropouts.items()))
  if route.faults:
    codes = ", ".join(f"VIOL 0x{v:02x} ERROR 0x{e:02x} ({n} frames)" for (v, e), n in route.faults.most_common())
    print(f"TI reported faults while engaged: {codes}")
  if not route.alerts:
    print("no 'Take Control / Turn Exceeds Steering Limit' alerts")
    return
  print(f"{len(route.alerts)} 'Take Control' alert(s):")
  print("   time   mph  want m/s2  got m/s2  ctrl out  TI now  TI peak1s     TI state  drv  cause")
  for (tt, v, want, got, out, ti, peak, st, ramp, drv, cause) in route.alerts:
    state = st + (" RD" if ramp else "")
    print(f"  {tt:6.1f}  {v * 2.237:4.0f}  {want:9.2f}  {got:8.2f}  {out:8.2f}  {ti:6d}  {peak:9d}  {state:>11s}  {drv:4.0f}  {cause}")
  print("causes: " + "; ".join(f"{k}: {v}" for k, v in route.causes.most_common()))


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("route", nargs="?", help="route name (default: newest)")
  ap.add_argument("--root", default=DEFAULT_ROOT)
  ap.add_argument("--last", type=int, default=1, help="how many of the newest routes")
  ap.add_argument("--segments", type=int, default=0, help="only the last N minutes (segments) of each route")
  args = ap.parse_args()

  if not os.path.isdir(args.root):
    sys.exit(f"no log folder at {args.root} (pass --root)")
  routes = find_segments(args.root)
  if not routes:
    sys.exit(f"no routes under {args.root}")
  if args.route:
    names = [n for n in routes if n == args.route or n.endswith(args.route)]
    if not names:
      sys.exit(f"route {args.route} not found under {args.root}")
  else:
    names = sorted(routes, key=lambda n: os.path.getmtime(routes[n][-1][1]))[-args.last:]

  for name in names:
    route = Route(name)
    segments = routes[name][-args.segments:] if args.segments else routes[name]
    for n, (_, seg_path) in enumerate(segments, 1):
      print(f"\rreading {name} segment {n}/{len(segments)}", end="", file=sys.stderr, flush=True)
      fn = log_file(seg_path)
      if fn is None:
        continue
      route.used_qlog |= os.path.basename(fn).startswith("qlog")
      try:
        for evt in _LogFileReader(fn):
          route.feed(evt)
      except Exception as e:  # a segment still being written, or corrupt
        print(f"\n  (skipped {os.path.basename(seg_path)}: {e})", file=sys.stderr)
    print(file=sys.stderr)
    report(route)


if __name__ == "__main__":
  main()
