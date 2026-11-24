# Heatdeath

`./heatdeath` exports the joined replay fixture used during StarPilot Controls CPU optimization. It runs actual Card, SelfdriveD, Controls, RadarD and planner processes, feeds recorded CAN and peer messages at their recorded cadence, and captures bounded timing and native-safety evidence. It changes only owned benchmark processes' affinity. It does not change production scheduling, IRQ affinity, CPU governors, online cores, installed preferences or source files.

This first export retains the existing **active-long Ioniq 6 steady-state fixture**. It is a CPU-placement benchmark, not a general vehicle replay test. It explicitly restores the recorded remembered cruise speed and post-ECU-handoff initialization phase; it never restores an armed latch, command, native permission or healthy receipt. Discovery, physical ECU handoff and startup performance are excluded. Native permission must be earned by fresh replay inputs.

## Requirements

- A parked, offroad Linux ARM64 comma bench device with the normal compiled StarPilot dependencies and development compiler installed. Build the checkout normally before preparation. Execution requires root for the production realtime setup and private namespaces.
- A clean source checkout. A bundle is pinned to that exact revision and source hashes. Prepare/build a separate bundle when comparing a different revision.
- Local full rlogs, including startup CarParams, from a compatible Ioniq 6 active-long drive. Recordings and compiled binaries are not bundled in Git.
- An explicit JSON preference snapshot, mapping parameter names to base64 bytes. Provide the actual intended benchmark settings, including `OpenpilotEnabledToggle=1` and `IsOffroad=0` for the **private** Params namespace. Production readiness/CarParams keys cannot be supplied. No physical device preferences are changed.
- Explicit source clock context: the source boot-minus-monotonic offset and whether `deviceMotion` timestamps use monotonic or boottime. Do not infer this from your current device clock.

The root command selects `/usr/local/venv/bin/python3` when installed; override with `HEATDEATH_PYTHON`. Run it directly, outside `./dev`, which can enable simulation environment variables. `./heatdeath --help` works without native dependencies or root.

## Prepare and build

Use a 60-second window with two clean, stable cruise-speed samples immediately before its start. Times below are examples; choose the timestamps from your recording. Include startup and any adjacent rlogs needed to cover the window.

```sh
./heatdeath prepare \
  --rlog /data/bench/startup/rlog.zst \
  --rlog /data/bench/window/rlog.zst \
  --start-mono-ns 220000000000 --duration 60 \
  --source-boot-minus-mono-ns 0 --device-motion-clock monotonic \
  --params-json /data/bench/params.json \
  --output /data/bench/heatdeath-bundle

sudo ./heatdeath build --bundle /data/bench/heatdeath-bundle
```

Use zero boot offset only when verified appropriate for the recording. The bundle retains input hashes, actual CarParams, source/helper hashes and the explicit checkpoint. Build compiles a virtual transport around current production pandad and its safety test library. USB/SPI implementations are not linked; it cannot send commands to the vehicle. Build output remains in the external bundle, with a receipt and logs.

## Compare CPU placements

First capture the production placement baseline, then change only the actor placements being compared. Run outputs must be new directories outside the checkout and bundle.

```sh
sudo ./heatdeath run --bundle /data/bench/heatdeath-bundle \
  --output /data/bench/baseline-1

sudo ./heatdeath run --bundle /data/bench/heatdeath-bundle \
  --placement '{"card":[4],"controlsd":[6],"selfdrived":[5],"radard":[3,5],"plannerd":[3,5],"peer":[1],"observer":[0,1]}' \
  --native-core2 --reserve-core 6 --reserve-ms 8 --reserve-period-ms 50 \
  --output /data/bench/placement-1

./heatdeath report --output /data/bench/placement-1 --skip-seconds 10
```

Actor names in `--placement` are optional: omitted actors retain their original fixture/production placement. Priority remains the current production policy. `--native-core2` changes only the virtual native endpoint's placement after validating its actual startup thread state; it does not move physical pandad. CPU choices must already be online and allowed. No cores are enabled automatically.

The reserve is an **optional synthetic CFS CPU workload**, with measured dose and missed-pulse reporting. It is not inference, soundd or Android Auto. A clean baseline and a loaded run answer different questions. Use repeated baseline/candidate/candidate/baseline runs with identical inputs, reserve dose, peer/native placement, thermals and power state to compare distributions. Do not infer a scheduler improvement from one favorable sample.

## Interpret results

`report` shows phase call rate, wall-time p50/p95/p99/max, thread CPU p95, largest start gap, rolling lag flags, native receipt and active-authority observations. Raw binary capture, hashes, CPU/scheduler records, source-feed lateness, child logs and timing rings stay in the run directory. Raw observations can also be decoded using the bundled driver's `decode-observations` command.

A completed command is not a headroom or steering pass. Check native RX/TX admission, sustained actual active output, source-feed completeness/lateness, source ages, lag episodes and scheduling records together. The fixture refuses unsupported/mixed CarParams and stale native/authority evidence; do not disable those checks to obtain prettier timings. Timing rings that overflow are rejected by the report.

Recorded model and driver-monitoring messages are inputs. Actual model inference, camera decoding, audio and Android Auto are not launched by this harness. This benchmark cannot alone establish real co-load headroom, ECU acceptance, physical firmware-heartbeat equivalence, or road behavior. The bounded observer and virtual endpoint themselves consume CPU and RAM; include their cost.

Runs stop if the physical device leaves offroad/disengaged state and have a 180-second supervisor bound. Cleanup targets only actors carrying that run's unique token and only the journaled private message namespace. A per-device lock prevents concurrent Heatdeath runs/builds. It does not coordinate other developers' unrelated bench harnesses; keep the test device exclusively assigned while measuring. Interrupt normally with Ctrl-C; do not delete another active run's files.
