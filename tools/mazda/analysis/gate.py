"""Which gate kept torqued's live tune from the controller? Per route, over engaged rows: the toggles controlsd ran on
(fat = Force Auto Tune, fato = Force Auto Tune Off, cust_* = hand-set factor / friction in use), the share of frames where
the liveTorqueParameters message was valid (ltp_valid: every torqued input valid), torqued's own useParams (ltp_use) and
its inputs (pose_ok, cal == 1, lag_ok), when the message first went valid, and the feedforward intercept (ff_fit.py):
near +0.21 the live offset reached the controller, near 0 it did not. controlsd takes the live tune on a frame when
ltp_valid and (ltp_use or fat); once taken, the values stay in the controller until the next start."""
# ruff: noqa: E701  -- one-off analysis scripts, kept as written for the write-ups
import sys
import glob
import os
import json
if len(sys.argv) > 2:
  sys.path.insert(0, sys.argv[2])   # an extra directory with pandas, if it is not installed
import numpy as np
import pandas as pd

GATE = ["fat", "fato", "cust_laf", "cust_fric", "ltp_valid", "ltp_use", "ltp_ok", "pose_ok", "lag_ok"]

def share(df, col):
  if col not in df or df[col].notna().sum() == 0: return "  -  "
  return f"{df[col].mean():5.2f}"

print("route     build   | " + " ".join(f"{c:>9}" for c in GATE) + "   cal=1  first_valid_s  ff_intercept  mean_i  learned_off  verdict")
for f in sorted(f for f in glob.glob(os.path.join(sys.argv[1], "*.csv.gz")) if ".events." not in f):
  df = pd.read_csv(f)
  meta_path = f.replace(".csv.gz", ".json")
  build = json.load(open(meta_path)).get("build", "?") if os.path.exists(meta_path) else "?"
  e = df[df["en"] == 1]
  if len(e) < 100: continue
  cal = f"{(e['cal'] == 1).mean():5.2f}" if "cal" in e and e["cal"].notna().any() else "  -  "
  first_valid = df.loc[(df["ltp_valid"] == 1), "t"].min() if "ltp_valid" in df else float("nan")
  h = df[(df["lat"] == 1) & (df["en"] == 1) & (df["prs"] == 0) & (df["v"] > 20) & df["f"].notna() & df["roll"].notna()]
  if len(h) >= 500:
    A = np.c_[h["sp"], h["roll"] * 9.81, np.ones(len(h))]
    c = np.linalg.lstsq(A, h["f"], rcond=None)[0][2]
    mean_i = h["i"].mean()
    off = df["laoff"].dropna().mean() if "laoff" in df else float("nan")
    verdict = "applied" if abs(c - (-off if off == off else 0.21)) < 0.08 else ("not applied" if abs(c) < 0.08 else "unclear")
    fit = f"{c:+.3f}        {mean_i:+.3f}  {off:+.3f}       {verdict}"
  else:
    fit = "(too little highway)"
  print(f"{os.path.basename(f)[:8]}  {build:7} | " + " ".join(f"{share(e, c):>9}" for c in GATE) + f"   {cal}  {first_valid:9.1f}      {fit}")
