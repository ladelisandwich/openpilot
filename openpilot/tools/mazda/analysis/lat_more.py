"""Tune-export analysis (see README.md). Usage: python3 <script> <export dir> [pandas dir]"""
# ruff: noqa: E501, E701, E702, ISC002  -- one-off analysis scripts, kept as written for the write-ups
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
  df = pd.read_csv(f); df["route"], df["build"] = route, meta["build"]; frames.append(df)
df = pd.concat(frames, ignore_index=True)
df["grp"] = np.where(df["build"] == "17c2993", "old", "new")
act = df[(df["lat"] == 1) & (df["en"] == 1)].copy()
bins = [0, 5, 8.75, 11.75, 15, 20, 27, 40]
act["bin"] = pd.cut(act["v"], bins)
pd.set_option("display.width", 250)
print("== driver-pressed share while steering (prs), and TI torque sensor |ti_drv| mean / p95, by speed bin")
rows = []
for (g, b), x in act.groupby(["grp", "bin"], observed=True):
  if len(x) < 60: continue
  rows.append(dict(grp=g, v=str(b), s=round(len(x) / 20), prs=round(x["prs"].mean(), 2), ti_drv=round(x["ti_drv"].abs().mean(), 1),
                   ti_drv95=round(x["ti_drv"].abs().quantile(.95), 1), cmd=round(x["ti_cmd"].abs().mean()),
                   corr_drv_cmd=round(np.corrcoef(x["ti_drv"].fillna(0), x["ti_cmd"].fillna(0))[0, 1], 2),
                   track=round(x["got"].abs().mean() / max(x["sp"].abs().mean(), 1e-6), 2), ceil=round((x["ti_cmd"].abs() >= 675).mean(), 3)))
print(pd.DataFrame(rows).sort_values(["v", "grp"]).to_string(index=False))
print("\n== signed means at > 20 m/s, hands off (bias check): i, p, f, out, roll, err, steer angle")
h = act[(act["v"] > 20) & (act["prs"] == 0)]
for g, x in h.groupby("grp"):
  print(f"  {g}: i {x['i'].mean():+.3f} (share i>0 {(x['i'] > 0).mean():.2f})  p {x['p'].mean():+.3f}  f {x['f'].mean():+.3f}  out {x['out'].mean():+.3f}  "
        f"roll {x['roll'].mean():+.4f}  err {x['err'].mean():+.3f}  steer {x['steer'].mean():+.2f} deg  laf {x['laf'].mean():.2f} fric {x['fric'].mean():.3f}")
print("\n== per route at > 20 m/s hands off: signed i and steer angle")
for r, x in h.groupby("route"):
  print(f"  {r[:8]} {x['build'].iloc[0]} {len(x)/20/60:5.1f} min  i {x['i'].mean():+.3f}  steer {x['steer'].mean():+.2f}  roll {x['roll'].mean():+.4f}  err {x['err'].mean():+.3f}")
