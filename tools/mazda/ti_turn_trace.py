#!/usr/bin/env python3
"""Frame-by-frame trace of what openpilot, the Torque Interceptor and the EPS did around every moment
the steering let go (Mazda + TI). Read-only: it only reads rlogs.

  python3 tools/mazda/ti_turn_trace.py                  newest route, every event found
  python3 tools/mazda/ti_turn_trace.py <route>          e.g. 00000067--e6f0214770
  python3 tools/mazda/ti_turn_trace.py <route> --at 118 --at 34     just those seconds into the route

Columns: want = the model's lateral accel request, sp = the controller's setpoint (the request delayed by the
steering lag), got = what the car did (m/s^2, left negative); CCtq/ctrl = openpilot's torque request;
TIcmd/stock = counts sent to the TI and to the stock LKAS channel; TIdrv = the driver's hands as the TI reads them;
EPSsen = the torque the EPS thinks it sees (TI + hands); EPSmot follows how fast the wheel is turning.

Events looked for (merged when closer than 3 s):
  LAT OFF      openpilot's lateral went inactive while moving, with the reason
  TI DROP      the TI command fell 200+ counts in half a second while openpilot still asked for 80%+
  TI STATE     the TI left RUN, raised RAMP_DOWN (raw byte, any firmware version), or reported VIOL/ERROR
  ALERT        a Take Control / Turn Exceeds Steering Limit alert
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
TI_STATES = {0: "DISC", 1: "OFF", 2: "DRVOV", 3: "RUN"}
PRE_S, POST_S, MERGE_S, ROW_HZ = 2.0, 3.0, 3.0, 10


def decode(addr: int, dat: bytes) -> dict:
  msg = DBC_2017.addr_to_msg[addr]
  out = {}
  for name, sig in msg.sigs.items():
    raw = get_raw_value(dat, sig)
    if sig.is_signed:
      raw -= ((raw >> (sig.size - 1)) & 1) * (1 << sig.size)
    out[name] = raw * sig.factor + sig.offset
  return out


def lkas_request(dat: bytes) -> int:
  return (((dat[0] & 0x0F) << 8) | dat[1]) - 2048


def find_route(root: str, want: str | None):
  routes = {}
  for name in os.listdir(root):
    path = os.path.join(root, name)
    route, _, seg = name.rpartition("--")
    if os.path.isdir(path) and route and seg.isdigit():
      routes.setdefault(route, []).append((int(seg), path))
  if not routes:
    sys.exit(f"no routes under {root}")
  if want:
    names = [n for n in routes if n == want or n.startswith(want) or n.endswith(want)]
    if not names:
      sys.exit(f"route {want} not found under {root}")
    name = names[0]
  else:
    name = max(routes, key=lambda n: os.path.getmtime(max(routes[n])[1]))
  return name, [p for _, p in sorted(routes[name])]


def log_file(seg_path: str):
  for fn in ("rlog.zst", "rlog.bz2", "rlog"):
    if os.path.isfile(os.path.join(seg_path, fn)):
      return os.path.join(seg_path, fn)
  return None


class State:
  def __init__(self):
    self.v = self.angle = self.drv = 0.0
    self.pressed = self.flt_t = self.flt_p = self.standstill = False
    self.blink = ""
    self.lat_active = self.enabled = False
    self.cc_tq = self.out_tq = self.lac_out = 0.0
    self.want = self.got = self.sp = 0.0
    self.sd_active = False
    self.alert = ""
    self.lat_check = True
    self.aol = False
    self.pause_lat = False
    self.ti_state, self.ti_ver, self.ti_rd, self.ti_viol, self.ti_err, self.ti_drv = None, 0, 0, 0, 0, 0
    self.ti_cmd = self.stock_cmd = 0
    self.eps_sensor = self.eps_motor = 0.0
    self.lkas_block = self.lkas_eff = 0
    self.events = []


def reason(s: State) -> str:
  why = []
  if not (s.enabled and s.sd_active) and not s.aol:
    why.append("openpilot not engaged" + ("" if s.enabled else " (disengaged)"))
  if s.flt_t:
    why.append("steerFaultTemporary (LKAS_BLOCK and TI not allowed)")
  if s.flt_p:
    why.append("steerFaultPermanent")
  if not s.lat_check:
    why.append("StarPilot lateral pause (PauseLateralSpeed/OnSignal/resume delay or button)")
  if s.pause_lat:
    why.append("StarPilot pauseLateral button")
  return "; ".join(why) or "unknown (none of the known gates)"


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("route", nargs="?")
  ap.add_argument("--root", default=DEFAULT_ROOT)
  ap.add_argument("--at", type=float, action="append", default=[], help="seconds into the route to trace (repeatable)")
  ap.add_argument("--max", type=int, default=10, help="most events to print")
  args = ap.parse_args()
  if not os.path.isdir(args.root):
    sys.exit(f"no log folder at {args.root} (pass --root)")
  name, segs = find_route(args.root, args.route)

  s = State()
  rows = []                     # (t, snapshot) at ROW_HZ
  events = []                   # (t, kind, detail)
  t0 = None
  commit = ""
  last_row_t = -1.0
  hist = deque()                # 0.5 s of (t, ti_cmd, cc_tq) at 100 Hz
  prev = {"lat": None, "state": None, "rd": 0, "viol": 0, "err": 0, "alert": False}

  for i, seg in enumerate(segs, 1):
    print(f"\rreading {name} segment {i}/{len(segs)}", end="", file=sys.stderr, flush=True)
    fn = log_file(seg)
    if fn is None:
      continue
    try:
      reader = _LogFileReader(fn)
    except Exception as e:
      print(f"\n  (skipped {os.path.basename(seg)}: {e})", file=sys.stderr)
      continue
    for evt in reader:
      w = evt.which()
      t_abs = evt.logMonoTime * 1e-9
      if t0 is None:
        t0 = t_abs
      t = t_abs - t0
      if w == "initData":
        commit = evt.initData.gitCommit[:7]
      elif w == "carState":
        cs = evt.carState
        s.v, s.angle, s.drv, s.pressed = cs.vEgo, cs.steeringAngleDeg, cs.steeringTorque, cs.steeringPressed
        s.flt_t, s.flt_p, s.standstill = cs.steerFaultTemporary, cs.steerFaultPermanent, cs.standstill
        s.blink = "L" if cs.leftBlinker else ("R" if cs.rightBlinker else "")
      elif w == "carControl":
        cc = evt.carControl
        s.lat_active, s.enabled, s.cc_tq = cc.latActive, cc.enabled, cc.actuators.torque
      elif w == "carOutput":
        s.out_tq = evt.carOutput.actuatorsOutput.torque
      elif w == "selfdriveState":
        ss = evt.selfdriveState
        s.sd_active = ss.active
        s.alert = ss.alertText1
      elif w == "starpilotPlan":
        s.lat_check = evt.starpilotPlan.lateralCheck
      elif w == "starpilotCarState":
        s.aol, s.pause_lat = evt.starpilotCarState.alwaysOnLateralEnabled, evt.starpilotCarState.pauseLateral
      elif w == "modelV2":
        s.want = evt.modelV2.action.desiredCurvature * max(s.v, 0.3) ** 2
      elif w == "onroadEvents":
        alert = any(e.name == "steerSaturated" for e in evt.onroadEvents)
        if alert and not prev["alert"]:
          events.append((t, "ALERT", "Take Control / Turn Exceeds Steering Limit"))
        prev["alert"] = alert
      elif w == "can":
        for c in evt.can:
          if c.src == 1 and c.address == TI_FEEDBACK and len(c.dat) >= 7:
            d = c.dat
            s.ti_drv, s.ti_ver, s.ti_state, s.ti_viol, s.ti_err, s.ti_rd = d[0] - 127, d[2], d[3], d[4], d[5], d[6]
          elif c.src == 0 and c.address == STEER_TORQUE:
            v = decode(STEER_TORQUE, c.dat)
            s.eps_sensor, s.eps_motor = v["STEER_TORQUE_SENSOR"], v["STEER_TORQUE_MOTOR"]
          elif c.src == 0 and c.address == STEER_RATE:
            v = decode(STEER_RATE, c.dat)
            s.lkas_block, s.lkas_eff = int(v["LKAS_BLOCK"]), int(v["LKAS_EFFECTIVE"])
      elif w == "sendcan":
        for c in evt.sendcan:
          if c.src == 1 and c.address == TI_COMMAND:
            s.ti_cmd = lkas_request(c.dat)
          elif c.src == 0 and c.address == STOCK_COMMAND:
            s.stock_cmd = lkas_request(c.dat)
      elif w == "controlsState":
        ctl = evt.controlsState
        lcs = ctl.lateralControlState
        lac = getattr(lcs, lcs.which())
        s.lac_out = float(getattr(lac, "output", 0.0))
        s.sp = float(getattr(lac, "desiredLateralAccel", 0.0))
        s.got = ctl.curvature * max(s.v, 0.3) ** 2

        # --- event detection at the controls rate ---
        moving = s.v > 0.5
        if prev["lat"] and not s.lat_active and moving:
          events.append((t, "LAT OFF", reason(s)))
          prev["lat_off"] = len(events) - 1
        elif s.lat_active and not prev["lat"] and prev.get("lat_off") is not None:
          te, kind, detail = events[prev["lat_off"]]
          events[prev["lat_off"]] = (te, kind, f"{detail}  [back after {t - te:.1f} s]")
          prev["lat_off"] = None
        prev["lat"] = s.lat_active
        if s.ti_state is not None:
          if prev["state"] == 3 and s.ti_state != 3:
            events.append((t, "TI STATE", f"TI left RUN -> {TI_STATES.get(s.ti_state, s.ti_state)}"))
          if s.ti_rd and not prev["rd"]:
            note = "; openpilot ignores it below version 2" if s.ti_ver <= 1 else ""
            events.append((t, "TI STATE", f"RAMP_DOWN byte set (TI firmware version {s.ti_ver}{note})"))
          if (s.ti_viol, s.ti_err) != (prev["viol"], prev["err"]) and (s.ti_viol or s.ti_err):
            events.append((t, "TI STATE", f"VIOL 0x{s.ti_viol:02x} ERROR 0x{s.ti_err:02x}"))
          prev["state"], prev["rd"], prev["viol"], prev["err"] = s.ti_state, s.ti_rd, s.ti_viol, s.ti_err
        hist.append((t, s.ti_cmd, s.cc_tq))
        while hist and t - hist[0][0] > 0.5:
          hist.popleft()
        if s.lat_active and abs(s.cc_tq) >= 0.8:
          peak_cmd = max(abs(h[1]) for h in hist)
          if peak_cmd - abs(s.ti_cmd) >= 200:
            events.append((t, "TI DROP", f"TI command {peak_cmd} -> {abs(s.ti_cmd)} counts while openpilot asked {s.cc_tq:+.2f}"))

        if t - last_row_t >= 1.0 / ROW_HZ - 1e-6:
          last_row_t = t
          rows.append((t, s.v, s.angle, s.want, s.sp, s.got, s.lat_active, s.cc_tq, s.lac_out, s.ti_cmd, s.stock_cmd,
                       s.ti_state, s.ti_rd, s.ti_viol, s.ti_drv, s.eps_sensor, s.eps_motor, s.lkas_block, s.lkas_eff,
                       s.pressed, s.flt_t, s.lat_check, s.blink, s.alert))
  print(file=sys.stderr)

  # merge events (first detail of each kind), or use the requested times
  if args.at:
    windows = [(a, {"AT": "requested"}) for a in sorted(args.at)]
  else:
    windows = []
    for te, kind, detail in sorted(events):
      if windows and te - windows[-1][0] < MERGE_S:
        windows[-1][1].setdefault(kind, detail)
      else:
        windows.append((te, {kind: detail}))

  print(f"route {name}  build {commit}  {len(rows) / ROW_HZ / 60:.1f} min  {len(events)} raw events, {len(windows)} windows")
  hdr = "     t   mph   angle  want    sp   got lat  CCtq  ctrl  TIcmd  stock  TIst RD vi TIdrv EPSsen EPSmot BLK  eff  prs flt chk bl alert"
  for n, (te, kinds) in enumerate(windows[:args.max], 1):
    print(f"\n#{n}  t={te:.1f}s  " + "  |  ".join(f"{k}: {d}" for k, d in kinds.items()))
    print(hdr)
    for r in rows:
      if te - PRE_S <= r[0] <= te + POST_S:
        (t, v, ang, want, sp, got, lat, cctq, lac, ticmd, stock, tist, rd, vi, tidrv, esen, emot, blk, eff, prs, flt,
         chk, bl, alert) = r
        mark = ">" if abs(t - te) < 0.5 / ROW_HZ else " "
        cols = [f"{mark}{t:6.1f}", f"{v * 2.237:5.1f}", f"{ang:7.1f}", f"{want:5.2f}", f"{sp:5.2f}", f"{got:5.2f}", f"{'Y' if lat else '-':>3}",
                f"{cctq:5.2f}", f"{lac:5.2f}", f"{ticmd:6d}", f"{stock:6d}", f"{TI_STATES.get(tist, '?'):>5}", f"{rd:2d}",
                f"{vi:2x}", f"{tidrv:5.0f}", f"{esen:6.0f}", f"{emot:6.0f}", f"{blk:3d}", f"{eff:4d}",
                f"{'Y' if prs else '-':>4}", f"{'Y' if flt else '-':>3}", f"{'Y' if chk else '-':>3}", f"{bl:>2}", alert[:28]]
        print(" ".join(cols))
  if len(windows) > args.max:
    print(f"\n({len(windows) - args.max} more windows; use --max or --at)")


if __name__ == "__main__":
  main()
