#!/usr/bin/env bash
# The Mazda container: the car, and (world = metadrive) a virtual display for the renderer.
set -euo pipefail
export MAZDA_SIM_CONFIG=${MAZDA_SIM_CONFIG:-/config/sim.json}
export PYTHONPATH=/sim:/sim/opendbc_repo

if python3 - <<'PY'
import json, os, sys
p = os.environ["MAZDA_SIM_CONFIG"]
cfg = json.load(open(p)) if os.path.isfile(p) else {}
sys.exit(0 if cfg.get("world", {}).get("world") == "metadrive" else 1)
PY
then
  export DISPLAY=:1
  rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
  Xvfb :1 -screen 0 1280x720x24 -nolisten tcp > /tmp/xvfb.log 2>&1 &
  for _ in $(seq 1 50); do [ -S /tmp/.X11-unix/X1 ] && break; sleep 0.1; done
fi

exec python3 -m mazda_sim.car.main
