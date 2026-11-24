"""Tune-export analysis (see README.md). Usage: python3 <script> <export dir> [pandas dir]"""
# ruff: noqa: E501, E701, E702  -- one-off analysis scripts, kept as written for the write-ups
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
  route = os.path.basename(f)[:20]; meta = json.load(open(os.path.join(d, route + ".json")))
  df = pd.read_csv(f); df["route"], df["build"] = route, meta["build"]; frames.append(df)
df = pd.concat(frames, ignore_index=True)
act = df[(df["lat"] == 1) & (df["en"] == 1) & df["ti_cmd"].notna() & df["ti_drv"].notna()].copy()
act["acmd"] = act["ti_cmd"].abs()
act["cbin"] = pd.cut(act["acmd"], [-1, 50, 100, 200, 300, 400, 500, 600, 700, 800])
print("== TI torque sensor against the TI command, all steering rows (prs: share flagged driver-pressed, |drv|>6)")
rows = []
for b, x in act.groupby("cbin", observed=True):
  if len(x) < 40: continue
  # the sensor's component along the command's sign: positive = with the command, negative = reaction
  along = np.sign(x["ti_cmd"]) * x["ti_drv"]
  rows.append(dict(cmd=str(b), s=round(len(x) / 20, 1), drv_mean=round(x["ti_drv"].abs().mean(), 1), drv95=round(x["ti_drv"].abs().quantile(.95), 1),
                   along=round(along.mean(), 1), prs=round(x["prs"].mean(), 2), v=round(x["v"].mean(), 1)))
print(pd.DataFrame(rows).to_string(index=False))
# slope of the reaction: along = k * |cmd| on frames the driver is plausibly off the wheel (prs 0)
h = act[act["prs"] == 0]
k = np.polyfit(h["acmd"], np.sign(h["ti_cmd"]) * h["ti_drv"], 1)
print(f"\nfit on hands-off rows: sensor-along-command = {k[0]:+.4f} * |cmd| {k[1]:+.2f}  (n={len(h)})")
print("\n== pid log check at > 20 m/s hands off: mean of p+i+d+f vs out")
hh = act[(act["v"] > 20) & (act["prs"] == 0)]
print(f"  p {hh['p'].mean():+.3f} i {hh['i'].mean():+.3f} d {hh['d'].mean():+.3f} f {hh['f'].mean():+.3f} sum {(hh['p']+hh['i']+hh['d']+hh['f']).mean():+.3f} out {hh['out'].mean():+.3f} cc_tq {hh['cc_tq'].mean():+.3f} out_tq {hh['out_tq'].mean():+.3f}")
print(f"  share out>0 {(hh['out']>0).mean():.2f}  share steer<0 {(hh['steer']<0).mean():.2f}  share i>0 {(hh['i']>0).mean():.2f}")
