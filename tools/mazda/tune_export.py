#!/usr/bin/env python3
"""Export what the drives did, compactly, for tuning offline (Mazda + Torque Interceptor + radar emulation /
hybrid longitudinal). Read-only: it only reads rlogs. Each route is read once.

  python3 tools/mazda/tune_export.py                 every route from the last 7 days
  python3 tools/mazda/tune_export.py --days 3        the last 3 days
  python3 tools/mazda/tune_export.py <route> ...     just these (a prefix is enough)

Writes to --out (default /data/tune_export), per route:
  <route>.csv.gz         rows over the whole route: ROW_HZ while openpilot or the cruise is engaged, IDLE_ROW_HZ otherwise
  <route>.events.csv.gz  every discrete event with its exact time (switches, buttons, cruise, alerts, TI faults)
  <route>.json           build, car parameters, the tune-related params, segments read
Routes already exported are skipped (delete their files to redo one), so it can be stopped and run again.
Run it with the car off: it competes with openpilot for the CPU.

Row columns (units: m/s, m/s^2, deg, TI/EPS counts; flags 0/1):
  t seg v a                    seconds since the route start, segment, speed, longitudinal accel (carState)
  steer rate drv prs           steering angle, rate, driver torque (EPS), steeringPressed
  lat en long                  carControl latActive / enabled / longActive
  cc_tq out_tq                 openpilot's torque request (-1..1), and as sent after limits (carOutput)
  sp got err p i d f out sat   torque controller: setpoint and measured lateral accel, error, terms, output, saturated
  dcurv curv mcurv             desired curvature (controls), measured curvature, the model's desired curvature
  roll laf fric                road roll (rad), live torque latAccelFactor and friction (filtered)
  ti_cmd stock_cmd             counts sent to the TI (0x249) and to the stock LKAS channel (0x243)
  ti_st ti_vi ti_drv           TI state (0 DISC 1 OFF 2 DRIVER_OVER 3 RUN), VIOL byte, driver torque the TI reads
  eps_sen eps_mot blk eff      EPS torque sensor and motor, LKAS_BLOCK, LKAS_EFFECTIVE
  blink flt                    blinker (-1 left, 1 right), steerFaultTemporary
  cc_acc out_acc lstate atgt   openpilot's accel request, as sent, longControlState, the planner's aTarget
  gas brk gas_pct brk_p acc_x  pedals pressed, gas pedal %, brake pressure, the car's own accel sensor (BRAKE)
  cruise vset                  car cruise engaged (PEDALS.ACC_ACTIVE), set speed (m/s, carState)
  st_cmd st_ok op_cmd          stock radar ACCEL_CMD (blank = standby), stock radar ACC_SET_ALLOWED, ours (echo)
  st_alive op_alive rlead      stock radar sending CRZ_INFO, openpilot's replacement frames on the bus, radar lead
  ld ld_d ld_v                 vision lead status, distance, relative speed (radarState.leadOne)
  exp pers                     experimental mode, driving personality
  stand res sstop              car standstill, openpilot asking to resume (carControl.cruiseControl.resume), planner shouldStop
  st_hold op_hold op_unl       CRZ_INFO stop request (STOPPING_MAYBE) from the stock radar / ours, ours RESUME_UNLATCHING_MAYBE
  st_ph op_ph                  CRZ_CTRL stop phase (RADAR_LEAD_RELATIVE_DISTANCE: 4 hold, 3 stop-go/resume), stock / ours
  hold epb haz wipe lamp       the car's auto hold (EPB.HOLD_STATE: 2 standby, 3 holding), parking brake applied, hazards on,
                               front wipers sweeping, exterior lamps on
  laoff ltp_ok                 torqued's live lateral-accel offset (m/s2) and whether its live params are valid
  ltp_valid ltp_use            the liveTorqueParameters message valid (all of torqued's inputs valid), torqued's useParams flag:
                               controlsd applies the live tune only with both (or Force Auto Tune for the second)
  pose_ok cal lag_ok           livePose valid, calibration status (1 calibrated), liveDelay valid: torqued's inputs
  fat fato cust_laf cust_fric  the toggles controlsd runs on, from the StarPilot toggle broadcast (starpilotPlan): Force Auto
                               Tune, Force Auto Tune Off, and whether the hand-set latAccelFactor / friction replace the live
                               ones. The live tune reaches the controller only with ltp_valid and (ltp_use or fat), and the
                               live factor / friction only without cust_laf / cust_fric. The route's params snapshot (json) is
                               the logger's copy of the params and can miss keys; these columns are what ran.
  ltp_dt                       seconds since the previous liveTorqueParameters message (torqued sends at 4 Hz; controlsd
                               also drops the live tune while the message's rate or age is off)
Each engaged stop is also an event ("stop") saying how it ended: openpilot RES, on its own, the driver's gas or RES,
or never (disengaged at a standstill), when openpilot first asked to go, and whether the car's auto hold held or the
hazards were on during the stop (either can explain a resume that did not happen).
"""
import argparse
import gzip
import json
import os
import re
import sys
import time

from openpilot.tools.lib.logreader import _LogFileReader
from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value

try:
  from openpilot.system.hardware.hw import Paths
  DEFAULT_ROOT = Paths.log_root()
except Exception:
  DEFAULT_ROOT = "/data/media/0/realdata"

ROW_HZ = 20          # while openpilot or the car's cruise is engaged
IDLE_ROW_HZ = 2      # otherwise: context only
ALIVE_S = 0.06       # a 50 Hz CRZ_INFO stream counts as gone after three missed frames (as in hybrid.py)
STOCK, OP_ECHO = 0, 128
TI_FEEDBACK, TI_COMMAND, STOCK_COMMAND, STEER_TORQUE, STEER_RATE = 0x24A, 0x249, 0x243, 0x240, 0x241
TI_CEILING = 700     # CarControllerParams.TI_CEILING: the car controller's clamp on the TI command (was the 800 scale max)
CRZ_INFO, CRZ_CTRL, CRZ_BTNS, PEDALS, ENGINE_DATA, BRAKE, RADAR_UDS = 0x21B, 0x21C, 0x09D, 0x165, 0x202, 0x078, 0x764
EPB, TURN_SWITCH, BLINK_INFO, LAMPS = 0x079, 0x091, 0x09A, 0x09F   # the body switches openpilot now reads (auto hold, hazards, wipers, lamps)
BUTTONS = ((0x01, "CANCEL"), (0x04, "RES"), (0x10, "SET+"), (0x20, "SET-"), (0x80, "DIST_DECREASE"), (0x40, "DIST_INCREASE"))
UDS_NAMES = {(0x10, 0x02): "programming session (silence)", (0x10, 0x01): "default session (restart)"}
PARAM_KEYS = re.compile("|".join(("Steer", "Lateral", "Torque", "Lat", "NNFF", "Longitudinal", "Personality", "Experimental",
                                   "Conditional", "Accel", "Decel", "Mazda", "Radar", "Hybrid", "Interceptor", "FLM", "MinSet",
                                   "Follow", "Stop", "Speed", "AutoTune", "Tune")), re.I)

COLUMNS = ("t seg v a steer rate drv prs lat en long cc_tq out_tq sp got err p i d f out sat dcurv curv mcurv roll laf fric " +
           "ti_cmd stock_cmd ti_st ti_vi ti_drv eps_sen eps_mot blk eff blink flt cc_acc out_acc lstate atgt " +
           "gas brk gas_pct brk_p acc_x cruise vset st_cmd st_ok op_cmd st_alive op_alive rlead ld ld_d ld_v exp pers " +
           "stand res sstop st_hold op_hold op_unl st_ph op_ph hold epb haz wipe lamp laoff ltp_ok ltp_valid ltp_use pose_ok cal lag_ok " +
           "fat fato cust_laf cust_fric ltp_dt").split()
ROUND = {"t": 3, "v": 3, "a": 3, "steer": 2, "rate": 1, "drv": 1, "cc_tq": 4, "out_tq": 4, "sp": 3, "got": 3, "err": 3,
         "p": 4, "i": 4, "d": 4, "f": 4, "out": 4, "dcurv": 6, "curv": 6, "mcurv": 6, "roll": 4, "laf": 3, "fric": 3,
         "eps_sen": 1, "eps_mot": 1, "cc_acc": 3, "out_acc": 3, "atgt": 3, "gas_pct": 1, "acc_x": 2, "vset": 2,
         "ld_d": 1, "ld_v": 2, "laoff": 3, "ltp_dt": 3}

DBC_2017 = DBC("mazda_2017")


def has_sig(addr: int, name: str) -> bool:
  """Whether the DBC on this device knows the signal: the newest decodes may be ahead of an older checkout."""
  msg = DBC_2017.addr_to_msg.get(addr)
  return msg is not None and name in msg.sigs


def sig(addr: int, dat: bytes, name: str) -> float:
  s = DBC_2017.addr_to_msg[addr].sigs[name]
  raw = get_raw_value(dat, s)
  if s.is_signed:
    raw -= ((raw >> (s.size - 1)) & 1) * (1 << s.size)
  return raw * s.factor + s.offset


# the body decodes that only the newest DBC has; an older checkout leaves those columns blank instead of crashing
HAVE_BODY = {"hold": has_sig(0x079, "HOLD_STATE") and has_sig(0x079, "EPB_ACTIVE"), "lamp": has_sig(0x09F, "LAMPS_ON")}


def accel_cmd(dat: bytes):
  """CRZ_INFO.ACCEL_CMD as the panda unpacks it; None for the standby frame (4094: no command)."""
  cmd = ((((dat[2] & 0x3) << 11) | (dat[3] << 3) | (dat[4] >> 5)) - 4096)
  return cmd if -2000 <= cmd <= 2000 else None


def hybrid_log_event(text: str):
  """The dict openpilot logged with carlog.warning({...}), if this logMessage is a mazdaHybrid* event (as hybrid_report)."""
  if "mazdaHybrid" not in text:
    return None
  try:
    stack = [json.loads(text)]
  except ValueError:
    return None
  while stack:
    x = stack.pop()
    if isinstance(x, dict):
      if str(x.get("event", "")).startswith("mazdaHybrid"):
        return x
      stack.extend(x.values())
    elif isinstance(x, str) and x.startswith("{"):
      try:
        stack.append(json.loads(x))
      except ValueError:
        pass
  return None


def lkas_request(dat: bytes) -> int:
  return (((dat[0] & 0x0F) << 8) | dat[1]) - 2048


def routes_under(root: str) -> dict:
  routes = {}
  for name in os.listdir(root):
    path = os.path.join(root, name)
    route, _, seg = name.rpartition("--")
    if os.path.isdir(path) and route and seg.isdigit():
      routes.setdefault(route, []).append((int(seg), path))
  return {r: sorted(s) for r, s in routes.items()}


def log_file(seg_path: str):
  for fn in ("rlog.zst", "rlog.bz2", "rlog", "qlog.zst", "qlog.bz2", "qlog"):
    if os.path.isfile(os.path.join(seg_path, fn)):
      return os.path.join(seg_path, fn)
  return None


def fmt(name: str, value) -> str:
  if value is None:
    return ""
  if isinstance(value, bool):
    return "1" if value else "0"
  if isinstance(value, float):
    digits = ROUND.get(name, 3)
    return f"{value:.{digits}f}".rstrip("0").rstrip(".") if digits else str(int(round(value)))
  return str(value)


class Export:
  def __init__(self, events_out):
    self.r = dict.fromkeys(COLUMNS)
    flags = ("lat", "en", "long", "prs", "sat", "gas", "brk", "cruise", "st_alive", "op_alive", "rlead", "ld", "exp", "flt", "blink",
             "st_ok", "stand", "res", "sstop", "st_hold", "op_hold", "op_unl", "hold", "epb", "haz", "wipe", "lamp")
    self.r.update(dict.fromkeys(flags, 0))
    self.stop = None                # the engaged stop in progress (see track_stop)
    self.ti_ran = False             # the TI has been in RUN this route (its power-up VIOL byte is not a fault)
    self.events_out = events_out
    self.t0 = None
    self.t = 0.0
    self.tc = 0.0                   # CAN clock (see hybrid_report: liveness judged on the CAN stream's own clock)
    self.stock_last = self.op_last = -1e9
    self.ltp_last = None
    self.next_row = 0.0
    self.prev = {}
    self.btn_prev = {STOCK: 0, OP_ECHO: 0}
    self.meta = {"build": None, "branch": None, "carParams": None, "params": {}, "toggles": None, "segments": []}
    self.counts = {"rows": 0, "lat_s": 0.0, "ceil_s": 0.0, "sat_s": 0.0, "lowspeed_lat_s": 0.0, "takeovers": 0, "handbacks": 0,
                   "ti_faults": 0, "stops": 0, "stops_openpilot_went": 0, "stops_driver_went": 0, "stops_never_went": 0}

  def feed_toggles(self, text: str) -> None:
    """The StarPilot process broadcasts its toggles as JSON on starpilotPlan every few seconds (and right after a
    change); between broadcasts the field is empty and the last values stand. Keep the gate-related ones."""
    if not text:
      return
    try:
      tog = json.loads(text)
    except ValueError:
      return
    r = self.r
    r["fat"], r["fato"] = int(bool(tog.get("force_auto_tune"))), int(bool(tog.get("force_auto_tune_off")))
    r["cust_laf"], r["cust_fric"] = int(bool(tog.get("use_custom_latAccelFactor"))), int(bool(tog.get("use_custom_friction")))
    keep = ("force_auto_tune", "force_auto_tune_off", "use_custom_latAccelFactor", "use_custom_friction", "latAccelFactor",
            "friction", "steerRatio", "use_custom_steerRatio", "steerActuatorDelay", "use_custom_steerActuatorDelay", "liveValid",
            "nnff", "nnff_lite", "flm_trial_applied", "flm_active_profile_id", "mazda_steer_limit_classifier")
    gate = {k: tog.get(k) for k in keep}
    if self.meta["toggles"] is None:
      self.meta["toggles"] = gate
    self.edge("toggles", json.dumps(gate, sort_keys=True), "toggles",
              lambda v: {k: tog.get(k) for k in keep if tog.get(k) != self.meta["toggles"].get(k)} or "(same as the route start)")

  def event(self, kind: str, detail) -> None:
    self.events_out.write(f"{self.t:.3f},{kind},{json.dumps(detail)}\n")

  def edge(self, key: str, value, kind: str, detail_fn) -> None:
    if key in self.prev and self.prev[key] != value:
      self.event(kind, detail_fn(value))
    self.prev[key] = value

  def feed(self, evt, seg: int, rows_out) -> None:
    w = evt.which()
    t_abs = evt.logMonoTime * 1e-9
    if self.t0 is None:
      self.t0 = t_abs
    self.t = max(self.t, t_abs - self.t0)
    r = self.r
    if w == "initData":
      init = evt.initData
      self.meta["build"], self.meta["branch"] = init.gitCommit[:7], init.gitBranch
      try:
        for e in init.params.entries:
          if PARAM_KEYS.search(e.key) and len(e.value) <= 200:
            self.meta["params"][e.key] = e.value.decode("utf-8", errors="replace")
      except Exception:
        pass
    elif w == "carParams" and self.meta["carParams"] is None:
      try:
        cp = evt.carParams.to_dict()
        keep = ("brand", "carFingerprint", "flags", "steerRatio", "steerActuatorDelay", "lateralTuning", "mass", "wheelbase",
                "openpilotLongitudinalControl", "pcmCruise", "longitudinalTuning", "safetyConfigs", "radarUnavailable")
        self.meta["carParams"] = {k: cp.get(k) for k in keep}
      except Exception as e:
        self.meta["carParams"] = {"error": str(e)}
    elif w == "carState":
      cs = evt.carState
      r["seg"] = seg
      r["v"], r["a"], r["steer"], r["rate"] = cs.vEgo, cs.aEgo, cs.steeringAngleDeg, cs.steeringRateDeg
      r["drv"], r["prs"], r["gas"], r["brk"] = cs.steeringTorque, cs.steeringPressed, cs.gasPressed, cs.brakePressed
      r["vset"], r["flt"], r["stand"] = cs.cruiseState.speed, cs.steerFaultTemporary, cs.standstill
      r["blink"] = -1 if cs.leftBlinker else (1 if cs.rightBlinker else 0)
      if self.t >= self.next_row:
        busy = r["en"] or r["lat"] or r["cruise"]
        self.next_row = self.t + 1.0 / (ROW_HZ if busy else IDLE_ROW_HZ)
        r["t"] = self.t
        rows_out.write(",".join(fmt(c, r[c]) for c in COLUMNS) + "\n")
        self.count_row()
      self.track_stop()
    elif w == "carControl":
      cc = evt.carControl
      r["lat"], r["en"], r["long"] = cc.latActive, cc.enabled, cc.longActive
      r["cc_tq"], r["cc_acc"] = cc.actuators.torque, cc.actuators.accel
      r["lstate"] = str(cc.actuators.longControlState)
      r["res"] = cc.cruiseControl.resume
      self.edge("en", bool(cc.enabled), "engaged", lambda v: "openpilot engaged" if v else "openpilot disengaged")
      self.edge("lat", bool(cc.latActive), "lateral", lambda v: "lateral active" if v else "lateral off")
    elif w == "carOutput":
      r["out_tq"], r["out_acc"] = evt.carOutput.actuatorsOutput.torque, evt.carOutput.actuatorsOutput.accel
    elif w == "controlsState":
      ctl = evt.controlsState
      r["dcurv"], r["curv"] = ctl.desiredCurvature, ctl.curvature
      lcs = ctl.lateralControlState
      lac = getattr(lcs, lcs.which())
      for col, field in (("sp", "desiredLateralAccel"), ("got", "actualLateralAccel"), ("err", "error"), ("p", "p"),
                         ("i", "i"), ("d", "d"), ("f", "f"), ("out", "output")):
        r[col] = float(getattr(lac, field, 0.0))
      r["sat"] = bool(getattr(lac, "saturated", False))
    elif w == "modelV2":
      r["mcurv"] = evt.modelV2.action.desiredCurvature
    elif w == "liveParameters":
      r["roll"] = evt.liveParameters.roll
    elif w == "liveTorqueParameters":
      ltp = evt.liveTorqueParameters
      r["laf"], r["fric"] = ltp.latAccelFactorFiltered, ltp.frictionCoefficientFiltered
      r["ltp_dt"] = None if self.ltp_last is None else t_abs - self.ltp_last
      self.ltp_last = t_abs
      r["laoff"], r["ltp_ok"] = ltp.latAccelOffsetFiltered, int(ltp.liveValid)
      # whether controlsd will take them: the message must be valid (every torqued input valid) and useParams set
      r["ltp_valid"], r["ltp_use"] = int(evt.valid), int(ltp.useParams)
    elif w == "starpilotPlan":
      self.feed_toggles(evt.starpilotPlan.starpilotToggles)
    elif w == "livePose":
      r["pose_ok"] = int(evt.valid)
    elif w == "liveCalibration":
      r["cal"] = int(evt.liveCalibration.calStatus.raw)
    elif w == "liveDelay":
      r["lag_ok"] = int(evt.valid)
    elif w == "longitudinalPlan":
      r["atgt"] = float(getattr(evt.longitudinalPlan, "aTarget", 0.0))
      r["sstop"] = bool(getattr(evt.longitudinalPlan, "shouldStop", False))
    elif w == "radarState":
      lead = evt.radarState.leadOne
      r["ld"], r["ld_d"], r["ld_v"] = lead.status, lead.dRel, lead.vRel
    elif w == "selfdriveState":
      ss = evt.selfdriveState
      r["exp"] = ss.experimentalMode
      r["pers"] = str(ss.personality)
      self.edge("exp", bool(ss.experimentalMode), "experimental", lambda v: "experimental ON" if v else "experimental OFF")
      text = (ss.alertText1 or "").strip()
      self.edge("alert", text, "alert", lambda v: v or "(alert cleared)")
    elif w == "onroadEvents":
      names = sorted({str(e.name) for e in evt.onroadEvents})
      added = [n for n in names if n not in self.prev.get("onroad", [])]
      if added and "onroad" in self.prev:
        self.event("onroad", " ".join(added))
      self.prev["onroad"] = names
    elif w == "logMessage" and "mazdaHybrid" in evt.logMessage:
      msg = hybrid_log_event(evt.logMessage)
      if msg is not None:
        self.event("hybrid", msg)
        if msg.get("event") == "mazdaHybridTakeover":
          self.counts["takeovers"] += 1
        elif msg.get("event") == "mazdaHybridHandedBack":
          self.counts["handbacks"] += 1
    elif w == "can":
      self.tc = max(self.tc, t_abs - self.t0)
      for c in evt.can:
        a, d, src = c.address, c.dat, c.src
        if src == 1 and a == TI_FEEDBACK and len(d) >= 7:
          if d[3] == 3:
            self.ti_ran = True
          if d[4] and not r["ti_vi"] and self.ti_ran:
            # a VIOL code raised after the TI has run (0x11: the ceiling held too long). Its power-up frame carries
            # 0x12 for one frame before it clears, and that is not a fault
            self.counts["ti_faults"] += 1
          r["ti_drv"], r["ti_st"], r["ti_vi"] = d[0] - 127, d[3], d[4]
          self.edge("ti", (d[3], d[4], d[5]), "ti", lambda v: f"TI state {v[0]} VIOL 0x{v[1]:02x} ERROR 0x{v[2]:02x}")
        elif src == STOCK and a == STEER_TORQUE:
          r["eps_sen"], r["eps_mot"] = sig(a, d, "STEER_TORQUE_SENSOR"), sig(a, d, "STEER_TORQUE_MOTOR")
        elif src == STOCK and a == STEER_RATE:
          r["blk"], r["eff"] = int(sig(a, d, "LKAS_BLOCK")), int(sig(a, d, "LKAS_EFFECTIVE"))
        elif a == CRZ_INFO and src in (STOCK, OP_ECHO) and len(d) >= 8:
          if src == STOCK:
            self.stock_last = self.tc
            r["st_cmd"], r["st_ok"] = accel_cmd(d), bool(d[4] & 0x04)
            r["st_hold"] = int(sig(a, d, "STOPPING_MAYBE"))
          else:
            self.op_last = self.tc
            r["op_cmd"] = accel_cmd(d)
            r["op_hold"], r["op_unl"] = int(sig(a, d, "STOPPING_MAYBE")), int(sig(a, d, "RESUME_UNLATCHING_MAYBE"))
        elif a == CRZ_CTRL and src in (STOCK, OP_ECHO) and len(d) >= 8:
          if src == STOCK:
            r["rlead"] = bool(sig(a, d, "RADAR_HAS_LEAD"))
            r["st_ph"] = int(sig(a, d, "RADAR_LEAD_RELATIVE_DISTANCE"))
          else:
            r["op_ph"] = int(sig(a, d, "RADAR_LEAD_RELATIVE_DISTANCE"))
        elif src == STOCK and a == PEDALS and len(d) >= 1:
          r["cruise"] = bool(d[0] & 0x08)
          self.edge("cruise", r["cruise"], "cruise", lambda v: "cruise ENGAGED (car)" if v else "cruise DROPPED (car)")
        elif src == STOCK and a == ENGINE_DATA and len(d) >= 8:
          r["gas_pct"] = sig(a, d, "PEDAL_GAS")
        elif src == STOCK and a == BRAKE and len(d) >= 8:
          r["brk_p"], r["acc_x"] = int(sig(a, d, "BRAKE_PRESSURE")), sig(a, d, "VEHICLE_ACC_X")
        elif src == STOCK and a == EPB and len(d) >= 8 and HAVE_BODY["hold"]:
          r["hold"], r["epb"] = int(sig(a, d, "HOLD_STATE")), int(sig(a, d, "EPB_ACTIVE"))
        elif src == STOCK and a == TURN_SWITCH and len(d) >= 8:
          r["haz"] = int(sig(a, d, "HAZARD"))
        elif src == STOCK and a == BLINK_INFO and len(d) >= 8:
          r["wipe"] = int(bool(sig(a, d, "WIPER_LO")) or bool(sig(a, d, "WIPER_HI")))
        elif src == STOCK and a == LAMPS and len(d) >= 8 and HAVE_BODY["lamp"]:
          r["lamp"] = int(sig(a, d, "LAMPS_ON"))
        elif a == CRZ_BTNS and src in (STOCK, OP_ECHO) and len(d) >= 1:
          b = d[0] & 0xF5
          changed = b ^ self.btn_prev[src]
          for mask, name in BUTTONS:
            if changed & mask:
              who = "driver" if src == STOCK else "openpilot"
              self.event("button", f"{who} {name} {'down' if b & mask else 'up'}")
              if self.stop is not None and b & mask and name == "RES":
                self.stop["driver_res" if src == STOCK else "openpilot_res"] = True
          self.btn_prev[src] = b
        elif src == OP_ECHO and a == RADAR_UDS and len(d) >= 3 and d[0] == 2 and (d[1], d[2]) in UDS_NAMES:
          self.event("uds", f"openpilot -> radar: {UDS_NAMES[(d[1], d[2])]}")
      st_alive, op_alive = self.tc - self.stock_last < ALIVE_S, self.tc - self.op_last < ALIVE_S
      r["st_alive"], r["op_alive"] = st_alive, op_alive
      self.edge("st_alive", st_alive, "radar", lambda v: "stock radar BACK" if v else "stock radar SILENT")
      self.edge("op_alive", op_alive, "radar", lambda v: "openpilot frames START" if v else "openpilot frames STOP")
    elif w == "sendcan":
      for c in evt.sendcan:
        if c.src == 1 and c.address == TI_COMMAND:
          r["ti_cmd"] = lkas_request(c.dat)
        elif c.src == 0 and c.address == STOCK_COMMAND:
          r["stock_cmd"] = lkas_request(c.dat)

  def track_stop(self) -> None:
    """An engaged stop runs from the car standing still to it moving off (or openpilot disengaging)."""
    r, s = self.r, self.stop
    v = r["v"] or 0.0
    if s is None:
      if r["en"] and r["stand"]:
        self.stop = {"t": self.t, "mode": "openpilot" if r["op_alive"] else "MRCC", "asked": None, "gas": False,
                     "driver_res": False, "openpilot_res": False, "auto_hold": False, "hazards": False}
      return
    if r["res"] and s["asked"] is None:
      s["asked"] = self.t
    s["gas"] |= bool(r["gas"])
    s["auto_hold"] |= r["hold"] == 3
    s["hazards"] |= bool(r["haz"])
    if v > 0.5 or not r["en"]:
      self.end_stop(moved=v > 0.5)

  def end_stop(self, moved: bool, why: str = "") -> None:
    s, k = self.stop, self.counts
    self.stop = None
    if not moved:
      went, key = why or "never: openpilot disengaged at a standstill", "stops_never_went"
    elif s["gas"]:
      went, key = "driver's gas", "stops_driver_went"
    elif s["driver_res"]:
      went, key = "driver's RES", "stops_driver_went"
    elif s["openpilot_res"]:
      went, key = "openpilot's RES", "stops_openpilot_went"
    else:
      went, key = "on its own", "stops_openpilot_went"
    k["stops"] += 1
    k[key] += 1
    asked = None if s["asked"] is None else round(self.t - s["asked"], 1)
    self.event("stop", {"mode": s["mode"], "stopped_s": round(self.t - s["t"], 1), "went": went,
                        "openpilot_asked_to_go_s_before": asked, "auto_hold": s["auto_hold"], "hazards": s["hazards"]})

  def count_row(self) -> None:
    r, dt, k = self.r, 1.0 / ROW_HZ, self.counts   # lateral is only active on busy (ROW_HZ) rows
    k["rows"] += 1
    if r["lat"]:
      k["lat_s"] += dt
      k["sat_s"] += dt * bool(r["sat"])
      k["ceil_s"] += dt * (r["ti_cmd"] is not None and abs(r["ti_cmd"]) >= TI_CEILING - 25)   # within one STEER_DELTA_DOWN
      k["lowspeed_lat_s"] += dt * ((r["v"] or 0.0) < 9.0)


def export_route(name: str, segs: list, out_dir: str) -> dict:
  rows_path, events_path, meta_path = (os.path.join(out_dir, f"{name}{ext}") for ext in (".csv.gz", ".events.csv.gz", ".json"))
  tmp = [p + ".part" for p in (rows_path, events_path)]
  start = time.monotonic()
  with gzip.open(tmp[0], "wt", compresslevel=6) as rows_out, gzip.open(tmp[1], "wt", compresslevel=6) as events_out:
    rows_out.write(",".join(COLUMNS) + "\n")
    events_out.write("t,kind,detail\n")
    ex = Export(events_out)
    for i, (seg, path) in enumerate(segs, 1):
      print(f"\r  {name}: segment {i}/{len(segs)}", end="", file=sys.stderr, flush=True)
      fn = log_file(path)
      if fn is None:
        ex.meta["segments"].append({"seg": seg, "file": None})
        continue
      ex.meta["segments"].append({"seg": seg, "file": os.path.basename(fn)})
      try:
        for evt in _LogFileReader(fn):
          ex.feed(evt, seg, rows_out)
      except Exception as e:   # a truncated last segment is normal after a power cut
        ex.meta["segments"][-1]["error"] = str(e)
    if ex.stop is not None:
      ex.end_stop(moved=False, why="never: the route ended at a standstill")
  for p, final in zip(tmp, (rows_path, events_path), strict=True):
    os.replace(p, final)
  ex.meta.update({"route": name, "seconds": round(ex.t, 1), "export_s": round(time.monotonic() - start, 1), "rowHz": ROW_HZ, "idleRowHz": IDLE_ROW_HZ,
                  "summary": {k: round(v, 1) for k, v in ex.counts.items()}})
  with open(meta_path, "w") as f:
    json.dump(ex.meta, f, indent=1, default=str)
  print(file=sys.stderr)
  return ex.meta


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("routes", nargs="*")
  ap.add_argument("--root", default=DEFAULT_ROOT)
  ap.add_argument("--out", default="/data/tune_export")
  ap.add_argument("--days", type=float, default=7.0, help="routes whose newest segment is at most this old (ignored with routes)")
  args = ap.parse_args()
  if not os.path.isdir(args.root):
    sys.exit(f"no log folder at {args.root} (pass --root)")
  os.makedirs(args.out, exist_ok=True)
  routes = routes_under(args.root)
  if args.routes:
    names = sorted({n for want in args.routes for n in routes if n == want or n.startswith(want) or n.endswith(want)})
  else:
    cutoff = time.time() - args.days * 86400  # noqa: TID251  wall clock: compared with the segments' file times
    names = sorted(n for n, segs in routes.items() if max(os.path.getmtime(p) for _, p in segs) >= cutoff)
  names.sort(key=lambda n: max(os.path.getmtime(p) for _, p in routes[n]))
  print(f"{len(names)} route(s), {sum(len(routes[n]) for n in names)} segment(s) -> {args.out}", file=sys.stderr)
  for n in names:
    if os.path.exists(os.path.join(args.out, f"{n}.json")):
      print(f"  {n}: already exported", file=sys.stderr)
      continue
    meta = export_route(n, routes[n], args.out)
    s = meta["summary"]
    print(f"{n}  build {meta['build']}  {meta['seconds'] / 60:.1f} min  lateral {s['lat_s'] / 60:.1f} min " +
          f"(below 20 mph {s['lowspeed_lat_s'] / 60:.1f})  TI near its ceiling {s['ceil_s']:.0f} s  saturated {s['sat_s']:.0f} s  " +
          f"TI faults {s['ti_faults']:.0f}  stops {s['stops']:.0f} (openpilot went {s['stops_openpilot_went']:.0f}, " +
          f"you went {s['stops_driver_went']:.0f}, never {s['stops_never_went']:.0f})  takeovers {s['takeovers']:.0f} hand-backs {s['handbacks']:.0f}  " +
          f"({meta['export_s']:.0f} s to read)")


if __name__ == "__main__":
  main()
