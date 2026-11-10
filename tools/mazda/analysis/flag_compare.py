"""Smarter Steer Limit Flag (MazdaSteerLimitClassifier) on vs off, from tune_export rows of the same build.

The toggle's state per row comes from the StarPilot toggle broadcast the export records: the route json's first
toggles (meta["toggles"]) and its "toggles" events, whose detail lists the keys that differ from the route start.
With the toggle on, controlsd's integrator freeze fires only on a deepening mismatch, not on every rate-limited
step (lib/steer_limit.py). What that should show, hands-off steering by speed bin: fewer frozen integrator frames,
lower |error|, possibly more saturation alerts (the rail no longer hides them).

  frz   share of rows whose integrator did not move from the previous row although |err| >= 0.05 (the PID's
        own clip and the low-speed reset excluded): mostly steer-limit freezes, both ways read the same
  mm    share of rows with controlsd's raw mismatch |cc_tq - out_tq| > 0.01 (what the stock flag fires on)
  err   |lateral accel error| (m/s2), p95; i: |integrator|; sat: saturated share; ceil: |TI cmd| >= 675
  prs   driver-pressed share of all lateral-active rows (hands on, corrections)
"""
# ruff: noqa: E701, E702  -- one-off analysis scripts, kept as written for the write-ups
import sys
import glob
import json
import os
if len(sys.argv) > 2:
  sys.path.insert(0, sys.argv[2])   # an extra directory with pandas, if it is not installed
import numpy as np
import pandas as pd

KEY = "mazda_steer_limit_classifier"
d = sys.argv[1]


def read_events(path):
  """t,kind,detail with the detail JSON written unquoted (it holds commas): split on the first two commas."""
  rows = []
  if os.path.exists(path):
    import gzip
    with gzip.open(path, "rt") as fh:
      next(fh, None)
      for line in fh:
        t, kind, detail = (line.rstrip("\n").split(",", 2) + ["", ""])[:3]
        try:
          rows.append((float(t), kind, json.loads(detail)))
        except ValueError:
          rows.append((float(t), kind, detail))
  return pd.DataFrame(rows, columns=["t", "kind", "detail"])


def state_timeline(meta, events_path):
  start = bool((meta.get("toggles") or {}).get(KEY))
  times, states = [0.0], [start]
  ev = read_events(events_path)
  for _, e in ev[ev["kind"] == "toggles"].iterrows():
    detail = e["detail"]
    states.append(bool(detail.get(KEY, start)) if isinstance(detail, dict) else start)
    times.append(float(e["t"]))
  return start, pd.DataFrame({"t": times, "flag": states})


def alerts(events_path):
  ev = read_events(events_path)
  rows = [(float(e["t"]), name) for _, e in ev[ev["kind"] == "onroad"].iterrows() for name in str(e["detail"]).split()]
  return pd.DataFrame(rows, columns=["t", "name"])


frames, per_route, alert_rows = [], [], []
for f in sorted(f for f in glob.glob(os.path.join(d, "*.csv.gz")) if ".events." not in f):
  route = os.path.basename(f)[:20]
  meta = json.load(open(os.path.join(d, route + ".json")))
  ev_path = os.path.join(d, route + ".events.csv.gz")
  df = pd.read_csv(f)
  if df.empty: continue
  start, tl = state_timeline(meta, ev_path)
  df = pd.merge_asof(df.sort_values("t"), tl.sort_values("t"), on="t", direction="backward")
  df["flag"] = df["flag"].fillna(start).astype(bool)
  df["route"], df["build"] = route, meta.get("build")
  frames.append(df)
  al = alerts(ev_path)
  if len(al):
    al = pd.merge_asof(al.sort_values("t"), tl.sort_values("t"), on="t", direction="backward")
    al["route"] = route
    alert_rows.append(al)
  per_route.append((route, meta.get("build"), sorted(set(tl["flag"])), len(tl) - 1))

df = pd.concat(frames, ignore_index=True)
df["di"] = df.groupby("route")["i"].diff().abs()
df["dt"] = df.groupby("route")["t"].diff()
lat_rows = df[(df["lat"] == 1) & (df["en"] == 1)]
act = lat_rows[(lat_rows["prs"] == 0) & lat_rows["err"].notna()].copy()
act["aerr"] = act["err"].abs(); act["ai"] = act["i"].abs()
act["mm"] = ((act["cc_tq"] - act["out_tq"]).abs() > 0.01).astype(float)
act["ceil"] = (act["ti_cmd"].abs() >= 675).astype(float)
movable = act[(act["aerr"] >= 0.05) & (act["dt"] < 0.08) & (act["sat"] == 0) & (act["v"] > 1.0) & act["di"].notna()]
bins = [0, 5, 8.75, 11.75, 15, 20, 27, 40]
act["bin"] = pd.cut(act["v"], bins); movable = movable.assign(bin=pd.cut(movable["v"], bins))
lat_rows = lat_rows.assign(bin=pd.cut(lat_rows["v"], bins))

print("== routes: build, flag states seen, toggle changes while recorded")
for r in per_route:
  print(f"  {r[0]}  {r[1]}  flag {['off' if not s else 'on' for s in r[2]]}  changes {r[3]}")

rows = []
for (g, b), x in act.groupby(["flag", "bin"], observed=True):
  if len(x) < 200: continue
  m = movable[(movable["flag"] == g) & (movable["bin"] == b)]
  p = lat_rows[(lat_rows["flag"] == g) & (lat_rows["bin"] == b)]
  rows.append(dict(flag="on" if g else "off", v=str(b), min=round(len(x) / 20 / 60, 1), err=round(x["aerr"].mean(), 3),
                   err95=round(x["aerr"].quantile(.95), 3), i=round(x["ai"].mean(), 3),
                   frz=round((m["di"] < 5e-5).mean(), 3) if len(m) else np.nan, mm=round(x["mm"].mean(), 3),
                   sat=round(x["sat"].mean(), 4), ceil=round(x["ceil"].mean(), 4), prs=round(p["prs"].mean(), 3)))
pd.set_option("display.width", 250); pd.set_option("display.max_columns", 30)
print("\n== hands-off steering by speed bin (m/s), 20 Hz rows; min = minutes")
print(pd.DataFrame(rows).sort_values(["v", "flag"]).to_string(index=False))

print("\n== overall")
for g, x in act.groupby("flag"):
  m = movable[movable["flag"] == g]
  hw = x[x["v"] > 20]
  print(f"  flag {'on ' if g else 'off'}: {len(x)/20/60:6.1f} min, err {x['aerr'].mean():.3f}, frozen {(m['di'] < 5e-5).mean():.3f}, "
        f"highway signed err {hw['err'].mean():+.3f} i {hw['i'].mean():+.3f}, sat {x['sat'].mean():.4f}")

if alert_rows:
  al = pd.concat(alert_rows, ignore_index=True)
  hours = {g: len(x) / 20 / 3600 for g, x in lat_rows.groupby("flag")}
  print("\n== steering-related onroad events per hour of lateral-active driving")
  for name in ("steerSaturated", "steerTempUnavailable", "steerTempUnavailableSilent", "steerUnavailable", "laneChangeBlocked"):
    s = al[al["name"] == name]
    if not len(s): continue
    print(f"  {name:28s} " + "  ".join(f"flag {'on' if g else 'off'}: {(s['flag'] == g).sum() / max(h, 1e-6):5.1f}/h"
                                         for g, h in sorted(hours.items())))
