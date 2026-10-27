"""torqued's live lateral-accel offset against the integrator bias, per minute, on today's drives."""
# ruff: noqa: E501, E702  -- one-off analysis scripts, kept as written for the write-ups
import sys
import glob
import os
if len(sys.argv) > 2:
  sys.path.insert(0, sys.argv[2])   # an extra directory with pandas, if it is not installed
import pandas as pd
for f in sorted(f for f in glob.glob(os.path.join(sys.argv[1], "*.csv.gz")) if ".events." not in f):
  df = pd.read_csv(f)
  if "laoff" not in df or df["laoff"].notna().sum() == 0:
    print(os.path.basename(f)[:8], "no laoff column"); continue
  df["laoff"] = df["laoff"].ffill(); df["ltp_ok"] = df["ltp_ok"].ffill()
  h = df[(df["lat"] == 1) & (df["en"] == 1) & (df["prs"] == 0) & (df["v"] > 20)].copy()
  h["minute"] = (h["t"] // 60).astype(int)
  print(f"== {os.path.basename(f)[:8]}: live offset valid share {df['ltp_ok'].mean():.2f}; per minute at > 20 m/s hands off: offset (m/s2), integrator i, steer deg")
  rows = h.groupby("minute").agg(n=("t", "size"), laoff=("laoff", "mean"), i=("i", "mean"), steer=("steer", "mean"), laf=("laf", "mean"))
  rows = rows[rows["n"] >= 200]
  print(rows.round(3).to_string())
  print(f"   whole drive: offset {h['laoff'].mean():+.3f}  i {h['i'].mean():+.3f}  i+offset {(h['i'] + h['laoff']).mean():+.3f}")
