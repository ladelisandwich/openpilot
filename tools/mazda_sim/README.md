# Mazda sim: a CX-9 2023 + TI1 for your comma builds

A program you run on your PC to test openpilot builds without driving. You pick a build (a branch on GitHub, or
the checkout you're editing). The sim then compiles that build for your PC and plugs it into a simulated
**Mazda CX-9 2023 with a Torque Interceptor (TI1)**. The build has to treat it like the real car: it fingerprints
it over UDS, runs its real panda safety code, steers through the TI and the EPS, and follows the radar,
PCM and FSC. A browser page lets you drive: steer, use the pedals and stalk buttons, flip faults, and watch
what every module and openpilot are doing.

```
 ┌────────────── comma container ───────────────┐        ┌──────────── mazda container ────────────┐
 │ your build, compiled for x86, unmodified:    │        │ CX-9 2023 ECUs on simulated CAN:        │
 │ card, controlsd, selfdrived, plannerd,       │  CAN   │ PCM, EPS, ABS, TCM, body, cluster,      │
 │ locationd, paramsd, ui, the Galaxy, ...      │◄──────►│ MRCC radar, FSC, TI1 on the AUX lines   │
 │ in place of hardware:                        │ :7000  │ (UDS / ISO-TP on every module)          │
 │  pandad  → harnessd  virtual panda running   │        │ vehicle physics: bicycle model, EPS     │
 │            the build's own safety code       │ frames │ assist, column, TI injection, brakes    │
 │  camerad → opticsd   frames from the world   │◄───────│ road, lead car, driver (you or auto)    │
 │  modeld  → ground truth, or the build's own  │ :7001  │ optional MetaDrive rendering            │
 │            model on your GPU                 │        │ dashboard API  :8770                    │
 │ screen: noVNC :6080 · Galaxy :8082           │        │                                         │
 └──────────────────────────────────────────────┘        └─────────────────────────────────────────┘
                     ▲                                                     ▲
                     └────────── launcher on your PC (:8765) ──────────────┘
```

## Quick start

1. Install **Docker Desktop** (Windows/macOS) or **Docker Engine** (Linux), plus **Python 3.10+**.
   On Bazzite, see [below](#bazzite-and-other-fedora-atomic-systems-with-an-nvidia-gpu).
2. Get this repository (any checkout of this branch) and run:

   ```
   python tools/mazda_sim/launcher/mazdasim.py
   ```

   Your browser opens `http://localhost:8765`.
3. **Setup** tab: choose the build.
   - **From GitHub:** a repo and a branch, tag or commit.
   - **A checkout on this PC:** the folder you edit in. Uncommitted changes are included.

   Tick what is fitted to the car (TI, radar emulation, hybrid long, …) and press **Save**.
4. Press **Start**. The first start builds the images and compiles the build (roughly 20–40 minutes, depending
   on your PC). After that, a start takes seconds.
5. **Drive** tab. The car starts in D on the highway track. The auto driver keeps the lane at the speed limit
   until you press **SET−**, then openpilot is driving.

To test a code change: edit your checkout, then press **Reload build**. The sim copies only what changed and
scons rebuilds only what changed, and the comma restarts while the car keeps running.

### Bazzite (and other Fedora Atomic systems) with an NVIDIA GPU

You don't need to install Python: Bazzite already has it. Check with `python3 --version`.

**Don't use Docker Desktop on Bazzite.**
- It can't pass an NVIDIA GPU to containers on Linux.
- Its installer uses `dnf install`, which Bazzite blocks.
- It isn't supported on immutable systems.

**Use Bazzite's developer image instead.** It has Docker Engine and compose built in, and the NVIDIA image
already includes the NVIDIA Container Toolkit.

1. Check your GPU and current image:

   ```
   nvidia-smi --query-gpu=name --format=csv,noheader
   rpm-ostree status
   ```

   The developer image uses NVIDIA's open driver, so it needs a GTX 16xx or any RTX card. On a GTX 900/1000,
   follow "Older NVIDIA cards" below instead.
   If `rpm-ostree status` lists LayeredPackages, remove them first with `rpm-ostree uninstall …`. Layered
   packages can block a rebase.
2. Keep your current setup as a fallback, then rebase. Use the GNOME image if you're on GNOME:

   ```
   sudo ostree admin pin 0
   brh rebase bazzite-dx-nvidia:stable          # KDE   (GNOME: bazzite-dx-nvidia-gnome:stable)
   systemctl reboot
   ```

3. Join the docker group. The developer image is meant to do this itself, but it has an open bug where the
   group never gets created, so do it by hand:

   ```
   getent group docker || sudo groupadd --system docker
   sudo usermod -aG docker "$USER"
   sudo systemctl restart docker.socket
   systemctl reboot
   ```

4. Check that Docker works, that the GPU reaches containers, and that `docker info` lists `nvidia.com/gpu=all`.
   No `nvidia-ctk runtime configure` step is needed with Docker 29.2 or later.

   ```
   docker run --rm hello-world
   docker run --rm --gpus all ubuntu:24.04 nvidia-smi
   docker info | grep -i cdi
   ```

   If the GPU check fails after a driver update, run
   `sudo systemctl restart nvidia-cdi-refresh.service docker` and try again.
5. Get this branch, then start the launcher from a terminal opened after the reboot:

   ```
   git clone --branch mazda-long-ti1-testing https://github.com/ladelisandwich/openpilot.git
   python3 openpilot/tools/mazda_sim/launcher/mazdasim.py
   ```

   In Setup, under *This PC*, set GPU to **NVIDIA** and press Save, then Start.

**Things the developer image changes.** It is built from Bazzite's handheld (deck) image:
- The login manager changes.
- Steam may start at every login.
- To undo it, rebase back to your old image (the one `rpm-ostree status` showed), e.g.
  `brh rebase bazzite-nvidia-open:stable`. `brh rollback` also works.

**Older NVIDIA cards (GTX 900/1000).** There is no developer image for the legacy driver. Bazzite calls
layering a last resort, but it works: Docker's Fedora packages on top of your current `bazzite-nvidia` image,
which already has the NVIDIA toolkit.

```
sudo dnf5 config-manager addrepo --from-repofile=https://download.docker.com/linux/fedora/docker-ce.repo
sudo rpm-ostree install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
systemctl reboot
sudo systemctl enable --now docker
```

Then do steps 3–5 above. To remove it later:
`rpm-ostree uninstall docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin`.

## Driving

| | keyboard | gamepad | page |
|---|---|---|---|
| steer (your hands on the wheel) | ← → or A D | left stick | slider (springs back) |
| gas / brake | ↑ ↓ or W S | right / left trigger | sliders |
| SET− · SET+ · RES · CANCEL · DIST · MODE | 1 · 2 · 3 · 4 · 5 · 6 | A · – · X · B · Y · – | buttons ("hold" keeps one pressed) |
| blinkers | Q E | LB RB | ◀ ⚠ ▶ |

- **Driver modes.**
  - *Auto:* keeps the lane and a speed whenever openpilot isn't steering or holding speed, and takes over (overriding openpilot) if openpilot lets the car leave its lane.
  - *Hands resting:* resists quick wheel movements a little.
  - *Manual:* only your inputs.

  Your inputs add on top in every mode. Holding a wheel key is how you test steering overrides: watch the TI go to `DRIVER_OVER`.
- **Car:** gear, ignition, seatbelt, door, high beams, blind-spot warnings, driver distracted (the driver monitoring is faked as attentive otherwise), pause.
- **Lead car:** none, steady cruise, stop & go, or hard brake, with a speed and a gap.
- **Faults:**
  - TI: unplugged, sensor error, ignores commands.
  - EPS LKAS lockout.
  - Radar: refuses programming, restarts in standby, restart now.
  - Harness relay stuck.
- **Telemetry:** what openpilot is doing (state, alerts, events, torque and accel commands), the car, TI, EPS, PCM and radar, the panda (safety mode, controls allowed, blocked messages), and a 30 s strip chart.
- **Views:** a top-down road (lite world, mouse wheel zooms) or the MetaDrive chase camera, next to the comma's own screen. **comma screen ↗** opens it full size; **Galaxy ↗** opens StarPilot's web UI for toggles.

## Setup options

| Option | Effect |
|---|---|
| **Car** | Each box maps to the param the build reads: `TorqueInterceptorEnabled`, `RadarEmulationEnabled`, `MazdaHybridLong`, `NoFSC`, `NoMRCC`, `ManualTransmission`, `ExperimentalMode`, `LowerMinSetSpeed`, `IsMetric`. `car.extra_params` (in the JSON box) sets any other param before the build starts. TI version 2+ reports `RAMP_DOWN`. |
| **Scene** | *Lite* runs on any PC, with no rendering. *Rendered (MetaDrive)* draws the same road for the cameras and the chase view. Without a GPU it manages only a few frames a second (MetaDrive's terrain shader on Mesa's software rasterizer). |
| **Track** | highway (8 km loop), loop, twisty, city, straight. Lanes, lane width and start speed. |
| **Driving model** | *Ground truth* (any PC): a perfect model output computed from the road: plan, lanes, edges, lead, and the v15 `action`. It tests everything downstream of the model. *The build's own model* needs the rendered scene and a GPU. See the note below. |
| **This PC** | Docker or native (Linux). GPU: NVIDIA (Container Toolkit / WSL2), Intel through `/dev/dri`, AMD ROCm through `/dev/kfd`. Reachable from this PC only, or from your network. |
| **plant** (JSON) | The car's physics: mass, EPS assist curve, TI and LKAS torque scales, column friction, LKAS speed window, hands-off lockout, TI thresholds. The defaults reproduce this CX-9's learned lateral response (`LAT_ACCEL_FACTOR` ≈ 1.76 m/s²). Tune them against your own logs. |

Build changes take effect on **Reload build**; car and world changes on **Restart car**.
**Logs → delete the build cache** wipes the comma's volume: source copy, venv, build, compiled model and device state.

### The build's own driving model

This fork ships its driving model compiled only for the comma's GPU, and the ONNX it came from (the "rdf43"
v15 supercombo) is not in the repo. So it can't be rebuilt for a PC. In vision mode the sim compiles the repo's
`driving_vision.onnx` + `driving_policy.onnx` for your GPU (tinygrad: OpenCL, AMD or CPU) as a stand-in.

On 4 CPU cores that model takes about 360 ms per frame, so use a GPU. Everything after the model (planner,
controls, car port, safety) is your build either way.

## What is simulated

- **Fingerprinting.** VIN over OBD (`09 02`) and UDS `F190`. FW versions (`22 F188`) on eps 0x730, engine 0x7e0,
  fwdRadar 0x764, abs 0x760, fwdCamera 0x706 and transmission 0x7e1. They match `MAZDA_CX9_2021` exactly.
- **Panda.** The build's own `opendbc/safety` code, compiled on start and run with the firmware's wiring:
  - Relay intercept for car modes; pass-through for SILENT, NOOUTPUT and ELM327.
  - Bus 1 switched to the harness OBD lines for ELM327 param 0 and for MAZDA GEN1 + TI.
  - Forwarding, returned (+128) and rejected (+192) echoes.
  - 1 Hz safety tick.

  pandad's handshake (ELM327 → ObdMultiplexing → FirmwareQueryDone/ControlsReady → CarParams safety + StarPilot param bits) is mirrored too.
- **TI1** on the AUX lines.
  - States: DISCOVER, OFF, RUN, DRIVER_OVER, with KEY/CHKSUM/rate violations and a command timeout.
  - Feedback: driver torque in `TI_FEEDBACK`.
  - The injected torque goes through the EPS assist like a driver's input, so it steers at any speed.

  This is a model of the firmware's behaviour, not a dump of it.
- **EPS.** CAM_LKAS checksum and counter checks, a 52/45 km/h LKAS window, hands-off lockout, step faults, motor slew.
- **PCM.** Main switch and SET/RES engagement from the cruise master's `ACC_SET_ALLOWED`. Cancel reasons: brake, cancel, gear, belt, door, a lost or inactive master. Also standstill hold, and executing whoever sends `CRZ_INFO`.
- **MRCC radar.** Boot and restart warm-up, UDS programming session (silence) with an S3 timeout, distance bars, stop & go, and an optional restart-in-standby.
- **FSC.** `CAM_LKAS` idle frames, the radar presence check, and an SCBS malfunction latch.
- **Sensors.**
  - IMU on the comma 3X's chip axes.
  - GPS on `gpsLocation`, or `gpsLocationExternal` when `UbloxAvailable` is set.
  - Road and wide cameras in the VENUS NV12 layout modeld expects.

Replaced on the comma side: `pandad`, `camerad`, `modeld` (ground-truth mode) and `sensord` (sensors come from the car).
Not run: encoders, mic, driver-monitoring model (faked attentive, with a distracted toggle), webrtc and bridge,
StarPilot's `mapd` (it ships as an aarch64 binary) and `soundd` (no sound card). Everything else is the build as-is.

## Other ways to run it

- **Native (Linux, no Docker).** Install openpilot's Ubuntu dependencies (`tools/install_ubuntu_dependencies.sh`),
  `uv` and `rsync`. Then set *run with* to *native* in Setup. The build is prepared in `~/.mazda_sim/work`, and
  the comma's UI opens as a window on your desktop. A rendered scene uses your GPU through your X display.
- **Compose by hand.**

  ```
  MAZDA_SIM_HOME=~/.mazda_sim docker compose -f tools/mazda_sim/docker/compose.yml -p mazda-sim up --build
  ```

  Add `-f compose.nvidia.yml`, `compose.dri.yml` or `compose.amd.yml` for a GPU. `~/.mazda_sim/sim.json` is the
  configuration, and the dashboard is also served by the car at `http://localhost:8770`.

## Tests

```
cd tools/mazda_sim && python -m pytest
```

What's covered:
- The wire protocol.
- ISO-TP/UDS, and the FW versions against the build's fingerprint table.
- The TI and EPS fed by the build's own `mazdacan` message builders.
- Vehicle and ground-truth sign conventions; the tracks close.
- The MetaDrive map mapping.
- Device-binary detection.
- The panda with the build's safety library.
- The launcher's API.

## Findings in this build

The sim turned these up while being built against `mazda-long-ti1-testing`. They're worth a look:

- **GEN1 TI commands bypass the panda's torque checks.** `mazda.h` checks `CAM_LKAS2` (0x249, bus 1) only for
  GEN2/GEN3. On a GEN1 car with a TI, the panda forwards any TI torque, even with controls not allowed
  (`tests/test_panda.py`, xfail).
- **`ZeroDivisionError` at `starpilot/controls/starpilot_planner.py:215`** (`1 / abs(self.road_curvature)`) when the road curvature is exactly 0 (a perfectly straight
  model path).
- **modeld picks `DEV='LLVM'` on PC** (`selfdrive/modeld/modeld.py:7`). This tinygrad has no LLVM device (it is `CPU:LLVM`), so the stock model
  can't start off-device.
- **The Galaxy runs Flask with the werkzeug reloader off-device** (`starpilot/system/the_galaxy/the_galaxy.py`). The reloader re-executes the manager's argv,
  which gives a second manager. The sim sets `SP_GALAXY_DEBUG=0` and `SP_GALAXY_RELOAD=0`.
- **`python-dateutil` is imported but not declared.** StarPilot's theme manager imports it, but it only comes in
  through the `dev` extra (matplotlib).
- **StarPilot backs up the whole install at every boot.** It writes a ~1 GB tarball, keeps three, and costs
  minutes of CPU on a slow device. The sim switches it off with `MinimumBackupSize`.
- **Radar emulation's teardown in `CarInterface.init` falls inside the FSC's radar-presence window**, the first
  ~10 s after power-on. A slow first message can latch the SCBS malfunction.
