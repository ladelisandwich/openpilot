"""Is the learned offset in the feedforward? Fit f = a*sp + b*roll + c on hands-off highway rows: c near +0.21 means yes, near 0 no."""
# ruff: noqa: E701  -- one-off analysis scripts, kept as written for the write-ups
import sys
import glob
import os
if len(sys.argv) > 2:
  sys.path.insert(0, sys.argv[2])   # an extra directory with pandas, if it is not installed
import numpy as np
import pandas as pd
for f in sorted(f for f in glob.glob(os.path.join(sys.argv[1], "*.csv.gz")) if ".events." not in f):
  df = pd.read_csv(f)
  h = df[(df["lat"] == 1) & (df["en"] == 1) & (df["prs"] == 0) & (df["v"] > 20) & df["f"].notna() & df["roll"].notna()]
  if len(h) < 500: continue
  A = np.c_[h["sp"], h["roll"] * 9.81, np.ones(len(h))]
  a, b, c = np.linalg.lstsq(A, h["f"], rcond=None)[0]
  off = df["laoff"].dropna().mean() if "laoff" in df else float("nan")
  print(f"{os.path.basename(f)[:8]}: f = {a:+.2f}*sp {b:+.2f}*roll_g {c:+.3f}   (n={len(h)}, learned offset {off:+.3f}, mean i {h['i'].mean():+.3f})")
