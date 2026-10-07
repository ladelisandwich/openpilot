#!/usr/bin/env bash
# The comma container: get the build ready, give it a screen, run it against the car.
set -euo pipefail
export MAZDA_SIM_CONFIG=${MAZDA_SIM_CONFIG:-/config/sim.json}
export MAZDA_SIM_WORK=/work

mkdir -p /work/logs
python3 /sim/mazda_sim/device/prepare_build.py

# device state (params, StarPilot's data) survives restarts with the build
mkdir -p /work/home/.comma
ln -sfn /work/home/.comma /root/.comma
touch /root/.Xauthority   # the UI probes X auth for its desktop mouse helper

# the comma 3X's screen, in a browser at http://localhost:6080/vnc.html
export DISPLAY=:0
# Xvfb only speaks Mesa's software GL. With an NVIDIA GPU passed through, the container also gets NVIDIA's GL
# libraries, which cannot draw on Xvfb: keep the UI on Mesa (the driving model uses the GPU via OpenCL, unaffected)
export __GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1
[ -f /usr/share/glvnd/egl_vendor.d/50_mesa.json ] && export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json
rm -f /tmp/.X0-lock /tmp/.X11-unix/X0
Xvfb :0 -screen 0 2160x1080x24 -nolisten tcp > /work/logs/xvfb.log 2>&1 &
for _ in $(seq 1 50); do [ -S /tmp/.X11-unix/X0 ] && break; sleep 0.1; done
x11vnc -display :0 -forever -shared -nopw -quiet -localhost -rfbport 5900 > /work/logs/x11vnc.log 2>&1 &
websockify --web /usr/share/novnc 6080 localhost:5900 > /work/logs/novnc.log 2>&1 &

cd /work/openpilot
# shellcheck disable=SC1091
source /work/venv/bin/activate
export PYTHONPATH=/work/openpilot:/sim
export MAZDA_SIM_CAR_HOST=${MAZDA_SIM_CAR_HOST:-mazda}
export MAZDA_SIM_OPENDBC=/work/openpilot/opendbc_repo
export MAZDA_SIM_SAFETY_BUILD=/work/safety_build
echo "[comma] starting the build's manager (comma screen: http://localhost:6080/vnc.html?autoconnect=1&resize=scale)"
exec python -m mazda_sim.device.sim_manager
