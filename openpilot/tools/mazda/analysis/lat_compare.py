"""Old torque settings (build 17c2993) vs new (7f9512b, 1230769): steering by speed bin, from the tune export rows."""
# ruff: noqa: E701, E702  -- one-off analysis scripts, kept as written for the write-ups
import sys
import glob
import json
import os
if len(sys.argv) > 2:
  sys.path.insert(0, sys.argv[2])   # an extra directory with pandas, if it is not installed
import numpy as np
import pandas as pd

d = sys.argv[1]
frames = []
for f in sorted(f for f in glob.glob(os.path.join(d, "*.csv.gz")) if ".events." not in f):
  route = os.path.basename(f)[:20]
  meta = json.load(open(os.path.join(d, route + ".json")))
  df = pd.read_csv(f)
  df["route"], df["build"] = route, meta["build"]
  frames.append(df)
df = pd.concat(frames, ignore_index=True)
df["grp"] = np.where(df["build"] == "17c2993", "old", "new")
act = df[(df["lat"] == 1) & (df["en"] == 1) & (df["prs"] == 0) & df["err"].notna()].copy()
act["aerr"] = act["err"].abs(); act["ai"] = act["i"].abs(); act["aout"] = act["out"].abs()
act["acmd"] = act["ti_cmd"].abs(); act["asp"] = act["sp"].abs(); act["agot"] = act["got"].abs()
act["ceil"] = (act["acmd"] >= 675).astype(float)
act["top"] = (act["acmd"] >= 700).astype(float)
bins = [0, 5, 8.75, 11.75, 15, 20, 27, 40]
act["bin"] = pd.cut(act["v"], bins)
rows = []
for (g, b), x in act.groupby(["grp", "bin"], observed=True):
  if len(x) < 100: continue
  rows.append(dict(grp=g, v=str(b), s=round(len(x) / 20, 0), err=round(x["aerr"].mean(), 3), err95=round(x["aerr"].quantile(.95), 3),
                   i=round(x["ai"].mean(), 3), out=round(x["aout"].mean(), 3), cmd=round(x["acmd"].mean(), 0),
                   ceil=round(x["ceil"].mean(), 3), top=round(x["top"].mean(), 3), sat=round(x["sat"].mean(), 3),
                   sp=round(x["asp"].mean(), 3), track=round(x["agot"].mean() / max(x["asp"].mean(), 1e-6), 2),
                   eps=round(x["eps_sen"].abs().mean(), 1)))
out = pd.DataFrame(rows).sort_values(["v", "grp"])
pd.set_option("display.width", 250); pd.set_option("display.max_columns", 30)
print("rows at 20 Hz while steering, driver hands off. err: |lateral accel error| (m/s2), i: |integrator|, out: |command| (of 1),")
print("cmd: |TI counts|, ceil: share >= 675, top: share >= 700, sat: share flagged saturated, sp: |setpoint| (m/s2),")
print("track: |achieved| / |setpoint| lateral accel, eps: |EPS torque sensor counts|")
print(out.to_string(index=False))
# runs at the top of the ceiling on the new build
print("\n== longest unbroken runs at >= 700 counts (new build), seconds; the guard should dip within ~1.2 s")
for route, x in act[act["grp"] == "new"].groupby("route"):
  top = (x["acmd"] >= 700).astype(int).values
  runs, cur = [], 0
  for v in top:
    cur = cur + 1 if v else 0
    if cur: runs.append(cur)
  longest = max(runs) / 20 if runs else 0
  print(f"  {route}: top time {top.sum() / 20:.1f} s, longest run {longest:.2f} s")
print("\n== overall by build")
for g, x in act.groupby("grp"):
  print(f"  {g}: {len(x)/20/60:.1f} min hands-off steering, err {x['aerr'].mean():.3f}, sat {x['sat'].mean():.3f}, ceil {x['ceil'].mean():.3f}")
