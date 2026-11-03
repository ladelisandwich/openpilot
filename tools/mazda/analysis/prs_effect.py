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
  route = os.path.basename(f)[:20]; meta = json.load(open(os.path.join(d, route + ".json")))
  df = pd.read_csv(f); df["route"], df["build"] = route, meta["build"]; frames.append(df)
df = pd.concat(frames, ignore_index=True)
df["grp"] = np.where(df["build"] == "17c2993", "old", "new")
en = df[(df["en"] == 1) & (df["v"] > 5) & (df["v"] < 20)].copy()
print("== engaged rows 5-20 m/s: what happens on frames flagged driver-pressed (prs=1) vs not")
for (g, p), x in en.groupby(["grp", "prs"]):
  print(f"  {g} prs={p}: {len(x)/20:6.1f} s  lat(active) {x['lat'].mean():.2f}  |cc_tq| {x['cc_tq'].abs().mean():.3f}  |out_tq| {x['out_tq'].abs().mean():.3f}  "
        f"|ti_cmd| {x['ti_cmd'].abs().mean():5.0f}  |drv| {x['ti_drv'].abs().mean():4.1f}  |i| {x['i'].abs().mean():.3f}  |sp| {x['sp'].abs().mean():.3f}")
print("\n== the pressed episodes on the new build below 20 m/s: length and what the command did across them")
x = en[(en["grp"] == "new")].sort_values(["route", "t"])
ep, cur = [], None
for r in x.itertuples():
  if r.prs == 1:
    if cur is None: cur = dict(route=r.route, t0=r.t, n=0, cmd=[], drv=[], v=r.v)
    cur["n"] += 1; cur["cmd"].append(abs(r.ti_cmd) if r.ti_cmd == r.ti_cmd else 0); cur["drv"].append(r.ti_drv)
  elif cur is not None:
    ep.append(cur); cur = None
ep = [e for e in ep if e["n"] >= 3]
print(f"  {len(ep)} episodes of >= 0.15 s; first 12:")
for e in ep[:12]:
  print(f"  {e['route'][:8]} t {e['t0']:7.1f} v {e['v']:4.1f}  {e['n']/20:4.2f} s  |cmd| max {max(e['cmd']):4.0f} mean {np.mean(e['cmd']):4.0f}  drv first/peak {e['drv'][0]:+.0f}/{max(e['drv'], key=abs):+.0f}")
