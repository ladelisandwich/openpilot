# Mazda CX-9 (GEN1 + TI1) work: handoff

State of the work on `mazda-long-ti1-testing` as of 2026-10-27 (evening, PC session), written so a new session,
local or cloud, can continue without the old conversation.


## StarPilot 7: the first drives and what changed after them (2026-11-25, PC session)

The owner's findings after the first sp7 drives (routes `0000000a--0fb6848ddf` and `0000000b--62399ad21f`, 7 min
each, in `/data/media/0/realdata`; the new tree logs there, not to `realdata_konik`): low-speed steering over-tuned,
highway steering in an over-steer/correct loop, the long-switch icon gone, emulation not always in Experimental Mode.
Baseline: the old branch's `000000ca`/`000000cb` (2026-11-23). Measured from the rlogs (lateralTorqueParameters,
lateralDelay, controlsState.lateralControlState.torqueState):

- **Lateral delay.** Both sp7 drives ran on `lateralDelay` 0.535 s, "unestimated", the whole time. The old branch ran
  on its learned 0.216 s (std 0.007, 50 blocks) plus 0.1 s lateral smoothing = 0.316 s into the model and the
  controller. 0.11's lagd starts at steerActuatorDelay + 0.2 (the fork's 0.335 gave 0.535), learns only above 50 mph
  (the old one from 15 m/s), and its version bump threw the old estimate away. On the sp7 highway stretch the wheel
  wove about 4 deg at roughly 1.7 Hz with the model's own desired lateral accel oscillating with it; the old routes at
  the same speeds are smooth. Fix: the TI's steerActuatorDelay is 0.115, so lagd starts at 0.315 s (the old total)
  until it learns its own.
- **Controller.** The device has `LateralControllerSelection` = `{"MAZDA_CX9_2021": "starpilot"}`, saved between the two
  drives: `0a` ran Dom's standard controller (torqueState version 1), `0b` the StarPilot one (version 2). The StarPilot
  controller (starpilot/lateral/hkg_torque_policy.py, GenericTorquePolicy for this car) is StarPilot 6's controller
  almost line for line (flat KP 0.6, KI 0.35, LOW_SPEED_X/Y, MAX_LAT_JERK_UP, the unwind and center-chatter logic),
  but the port hooked the Mazda TI shaping only into the standard path, so `0b` ran with no error scale (about 5x the
  old P at 5 m/s) and no speed shaping of latAccelFactor/friction. Note: StarPilot 6's SteerKP toggle pinned the KP
  at a flat 0.6 every frame (controlsd), so its P per m/s^2 of error was (0.6 + LSF) * scale: 1.13 at 5 m/s, 1.08 at
  7.5, 1.11 at 10 and 13.5, 0.76 at 20, 0.63 at 30; the port's sp6_effective_gain had dropped the division by KP.
  Fix: GenericTorquePolicy applies the TI speed shape and StarPilot 6's error scale exactly as the old controller did,
  the StarPilot controller is this car's default, and the standard controller's conversion uses the corrected gain.
  Both are selectable in Dom's settings ("Steering Controller": StarPilot Controller = the StarPilot 6 tune, Stock
  Controller = 0.11's controller with the converted shaping).
- **Live learning.** torqued's `useParams` was true, but its filtered values were still the CarParams ones (calPerc
  9-25), and FLM (steerlearnerd, `starpilotLateralState.learningEvidence`) only recorded: neither changed the drives
  yet. The old branch never fed the live tune to this car. Learning is now off by default for the Mazda TI (Dom's "Use
  StarPilot Auto Learn" row turns it on per vehicle; learning_preferences.py).
- **Experimental Mode under emulation.** The planner follows `selfdriveState.experimentalMode`, which with Dom's
  Conditional Mode on (its default, Automatic, whenever `ConditionalModeConfig` is absent, as on this device) is
  Conditional Mode's own decision; the car controller picks emulation from the raw ExperimentalMode param. Fix:
  selfdrived publishes the hybrid master choice as the planner's mode for hybrid long
  (hybrid_buttons.planner_experimental), StarPilot 6's forced CEStatus.
- **Icon.** The device's screen is the StarPilot shell (starpilot/ui/runtime_app.py, large layout on the tizi); it
  uses the stock AugmentedRoadView only as a camera/model painter, so the stock HUD's icon hook never ran. The shell
  now draws it from its extra overlays, left of its steering wheel (or in the wheel's place when hidden), not under an
  alert (onroad_mazda_hybrid.render_shell). `/dev/shm/params/dx` was missing only because the device had rebooted.

**Second drive (route `0000000e`, 2026-11-25 22:23, build 4c026b03).** lagd learned 0.208 s within minutes (the old
branch's 0.216), the StarPilot controller ran with the TI shaping, fixed tune (useParams false). Rocking in the
0.8-3 Hz band, median angle rms per 4 s window: 0.75 deg at 45-56 mph and 0.48 at 56-67 (old branch 0.19 and 0.15;
the first sp7 drive 1.9 and 2.7); from 67 mph equal to the old branch. The owner felt it around 22:26 and it stopped
when the driver held the wheel. Everything in the loop was 2-3x the old branch at 45-67 mph: the model's desired curvature
(0.039 vs 0.017 m/s^2), the controller output (0.071 vs 0.022), the car (0.069 vs 0.022). The one loop difference left
was openpilot 0.11 dropping the 0.1 s lateral smoothing (modeld's desired-curvature filter plus the same 0.1 s on the
model's and the controller's delay). starpilot/car/mazda/model_smoothing.py restores it for the TI through the hook
modeld and controlsd already call for the Kona (hyundai/model_smoothing.py), so neither file changes (modeld.py is
hash-pinned by FLM's producer_clock_equivalence). The amount is the owner's to tune: Toggles, "Steering Smoothing"
(MazdaLatSmoothSeconds, 0.01-0.10 s in 0.01 s steps, default 0.05 s, half of StarPilot 6's), read when modeld and
controlsd start. The stepper subclasses InitialCruiseSpeedControl so the StarPilot shell's settings adapter shows it. Scan script: PC scratch osc_scan.py (band-passed angle/output/
desired per 4 s window, episodes with wall-clock times).

**Lower Minimum Set Speed (2026-11-26).** On the StarPilot 6 fork the toggle only ever worked with plain radar
emulation (8a17f8aa2 made hybrid long ignore it); on sp7 nothing read it. It works again while openpilot's emulated
radar is the ACC master: starpilot/car/mazda/low_set_speed.py tells card's cruise helper to own the set speed
(VCruiseHelper.owner_set_speed): seeded from the PCM's speed when emulation takes over, stepped by SET+/SET- down to
V_CRUISE_MIN (8 km/h), back to the PCM's speed when MRCC takes over (a disengage always hands back). MRCC itself keeps
the car's 19 mph floor; no software can change that. Untested on the car: cruising far below 19 mph with no lead
(the PCM's reaction is unknown; the old fork had the same exposure).

Also fixed the same day, from an audit of the port: card now sends inactive frames on any invalid carControl while
radar emulation is configured (Dom sends nothing then, and a bus fault anywhere left the PCM without its ACC frames
and the radar without tester-present; ti_startup.inactive_keepalive); the hybrid button ignores a press while CAN is
invalid (a stale DISTANCE_MORE could read as a long press on recovery); the tools in `tools/mazda/` import again
(opendbc's `get_raw_value` moved onto `Signal`); the Wipers Mean Rain text no longer promises weather offsets; the
dead `'mazda'` in `learning_allowed`'s native_brand was reverted (a Mazda's policy is 'generic', which returns first).

Open from that audit (not done; each needs its own change and a drive):
- **Panda: the TI steering command (0x249, bus 1) is not checked on GEN1** (mazda.h torque-checks it only for
  gen2/gen3; only the stock 0x243 is checked). Needs its own decode and limit check (0 without controls_allowed,
  |cmd| <= 800), separate from the 0x243 rate state, and a firmware rebuild.
- Panda: the radar programming-session request (UDS 0x10 0x02 to 0x764) is allowed at any time; gate it on
  controls_allowed (keep 0x10 0x01 and tester-present). The GEN2 ignition rule (0x274) runs on this GEN1 car too;
  and the bus-1 CRZ_CTRL engagement rule is a radar-interceptor leftover. Same firmware rebuild.
- Hazards do not hold the RES spam while MRCC is master (carcontroller RES path), unlike the emulation path.
- Other writers of ExperimentalMode switch the ACC master too (Dom's Conditional Mode commit writes it as its
  baseline; the onroad experimental button). With the planner now following the master this stays consistent, but
  a dedicated param would make the master independent of them.
- Dead code for this car: GEN2/GEN3 platforms, the radar interceptor (RI, BlendedACC), NoMRCC/NoFSC/manual,
  radar_session.py, ce_status_* in hybrid.py, the Lower Minimum Set Speed toggle (no reader), plain radar emulation
  without hybrid (EcuDisableFailed path). The stock AOL pieces stay (Dom's tests depend on them).
- selfdrived always initializes through its 6 s timeout with the TI (CarState invalid until ControlsReady).
- FLM: the TI speed shaping rewrites torque_params each frame below 21 m/s, which resets FLM's baseline; moot while
  learning is off (the new default), but FLM cannot hold an effect on this car if it is turned on.

Analysis scripts used for the above (lat_summary.py, trace.py, delay_flm.py) were PC scratch tools; the same reads
are easy with `openpilot.tools.lib.logreader._LogFileReader`.


## The StarPilot 7 branch: `sp7-mazda-long-ti1` (made 2026-11-24)

The fork's work re-ported onto StarPilotHQ/StarPilot branch `Dom` ("StarPilot 7.0", openpilot 0.11.2 base, the
big model, prebuilt). Read this before anything else on that branch; `mazda-long-ti1-testing` stays the StarPilot 6
branch and is untouched.

**How the branch was made.** Dom's own history could not be pushed to this fork (GitHub refuses it: old Dom commits
point at Git LFS objects that live only in StarPilot's repository), so the branch starts from a single root commit
holding the tree of Dom at `2c9a556f` (2026-11-17), and the port sits on top as ordinary commits. Later Dom changes
come over as diffs against that root (the Dom commit is fetchable from this fork by SHA, `git fetch origin 2c9a556f`;
the two repositories share GitHub's object store), never as a `git merge`.

**What carried over (and how it moved).**
- opendbc Mazda: the Torque Interceptor, radar emulation, hybrid longitudinal, the body inputs, the DBC decodes,
  the tests. Dom's car code no longer takes StarPilot toggles and no longer returns a StarPilot car state, so the
  car state keeps the body inputs as attributes and `openpilot/starpilot/car/mazda/dashboard.py` publishes them on
  `starpilotCarState` (fields renumbered @40-@46; `openpilot/cereal/gen/cpp/custom.capnp.*` regenerated).
  Upstream's flag class is `MazdaFlags`; ours is the same enum under both names. The wheel's RES/SET+ and SET- now
  always raise accelCruise/decelCruise button events, as every Mazda on Dom does.
- The panda safety (`opendbc_repo/opendbc/safety/modes/mazda.h`): our TI/RI/RE/hybrid modes plus Dom's stock
  always-on-lateral hooks, which only arm for an unmodified GEN1 (safety param 0). Unlike upstream, a flagged
  configuration keeps its transmit tables when the AOL alternative experience is set. The firmware was rebuilt for
  every variant in `panda/board/obj/` (debug cert, as Dom ships it); pandad flashes on the signature mismatch at
  the first boot. The Mazda CAN-ignition exception (0x9E) lives in `opendbc_repo/opendbc/safety/ignition.h` on
  Dom and is intact; the GEN2 one (0x274) was added there.
- Params: Dom's compiled `libparams_c.so` does not know our keys (TorqueInterceptorEnabled, RadarEmulationEnabled,
  MazdaHybridLong, the body-input toggles, MazdaHybridMaster, TetheringEnabled, EcuDisableFailed), so the
  header-only fallback was re-done on Dom's ctypes `Params` (`openpilot/common/params.py`): the same `<params>/dx`
  file store as the StarPilot 6 fork, so the device's settings carry over, plus the StarPilot 6 era names
  (`get_int`/`put_int`, `*_nonblocking`). Keys declared in `params_keys.h` with a CLEAR_ON_* flag are not served.
- The hybrid mode switch: Dom has no CEStatus; the distance-increase button now flips the stock `ExperimentalMode`
  param (long press toggles, a tap leaves emulation), and the car controller reads that
  (`openpilot/starpilot/car/mazda/hybrid_buttons.py`, run from card.py's Mazda branch). Conditional Experimental
  Mode's automatic triggers never touch it. The horn-tap bookmark is counted in selfdrived from
  `starpilotCarState.hornPressed` (the port first published `userBookmark` from card, but the UI publishes that
  service too, and msgq's second publisher raises on send, which would have taken card down; fixed 2026-11-25).
- Steering: the TI ceiling (700/800) and the speed-shaped plant moved to `openpilot/starpilot/lateral/
  mazda_ti_shaping.py` and Dom's `latcontrol_torque.py`. Dom's controller has no low-speed curvature factor; it
  raises the proportional gain itself (KP_INTERP, 14x at 5 m/s). The fork's low-speed error scale was tuned
  against the old factor, so the port converts the fork's *effective* gain (a flat 1.2-1.5x of the highway gain
  from 5 to 13.5 m/s, 1.0 from 30 m/s) onto Dom's schedule (`mazda_ti_error_scale`). This has not been driven:
  the first drives should be compared against the fork's routes with `ti_steer_report.py`. The steer-limit
  classifier is wired into Dom's controlsd behind the same toggle (read at startup).
- The learned tune: Mazda is admitted to torqued (`ALLOWED_CARS`) and to StarPilot's `learning_allowed`, so the
  live tune reaches the controller on every drive without Force Auto Tune; this was the open finding on the fork.
  Dom reads the Force Auto Tune files directly, so the fork's cache-reader fix is not needed there.
- selfdrived: TCS off (espDisabled) filtered unless MazdaTcsOffAlert, read with the other params. modeld: the
  wiper gate (no automated lane change while wiping, none for 3 s after the washer) is applied to Dom's
  `auto_ready` through `openpilot/starpilot/car/mazda/wipers.py`; modeld subscribes `starpilotCarState` for it.
- UI: the twelve Mazda toggles sit in Dom's Toggles page, shown only when the car is a Mazda. The hybrid master
  icon draws left of the experimental-mode button (`openpilot/starpilot/ui/onroad_mazda_hybrid.py`). The hotspot
  Off/Always/Onroad modes, the NAT rules and the "share cellular unless on a comma SIM" rule were re-applied to
  Dom's network widget and Wi-Fi manager. The build-metadata fallback in `openpilot/common/version.py`.
- Tools: `openpilot/tools/mazda/` and `openpilot/tools/mazda_sim/` as they were (imports checked); this file and
  CONTROLLABILITY.md.

**Upstream conventions the port had to meet (and the tests that enforce them).**
- A stock GEN1 runs with safety param 0 on openpilot 0.11: the panda now treats 0 as GEN1, and the interface sets
  the GEN1 bit only together with a fork flag (TI, RI, RE, NoMRCC, NoFSC), so a stock car stays `qualified()` for
  StarPilot's AOL and the firmware's TI bus switch still sees GEN1|TI. The fork's steerActuatorDelay (0.335) and
  "no dashcam-only" apply with the Torque Interceptor toggle; without it upstream's 0.1 and dashcam rule hold
  (the GEN2/GEN3 platforms steer only through the interceptor anyway).
- `CarParams.vEgoStarting/vEgoStopping/startAccel/startingState` and `longitudinalTuning.kpBP/kpV` are deprecated
  on 0.11 (`ret.deprecated.*`, unused): the emulation's resume threshold lives in the car controller
  (`RADAR_EMULATION_V_EGO_STARTING`), and its longitudinal loop runs on the I term and feedforward only, as every
  car's does on 0.11. The stop-and-go feel of the emulation is on the "needs the car" list for that reason.
- `CAR.with_flags()` is gone; the GEN1/GEN2/GEN3 sets are built from the platform configs.
- Dom's Mazda tests feed a fixed frame set and enumerate every platform as a stock GEN1: the harness now also sends
  the body-input messages the fork reads (TURN_SWITCH, MSG_05, MSG_11, EPB, TRACTION), the dashboard camera and
  the blind-spot test enumerate GEN1 only, and the stock-AOL safety test encodes this fork's policy that a flagged
  safety word keeps its own transmit table (upstream empties it under the AOL experience).
- Dom's card initializes the car interface, and so signals `ControlsReady`, only once CarState is valid with no
  bus timeout ("keep stock ECU output intact until CAN and controls are ready"). With the Torque Interceptor
  that never happens on its own: its feedback (TI_FEEDBACK, bus 1) exists only once pandad has set the Mazda
  safety mode with the TI flag, which routes bus 1 to the OBD-C port, and pandad waits for `ControlsReady`.
  The first drive on this branch sat in `elm327` safety with "CAN Bus Disconnected: Likely Faulty Cable".
  `openpilot/starpilot/car/mazda/ti_startup.py` lets card initialize on the car bus alone while bus 1 is the
  only silent parser, which is what the fork's previous base did implicitly (it set `ControlsReady`
  unconditionally once selfdrived had initialized). The same gate also wants a valid `carControl`, which
  controlsd marks valid only with a valid CarState, so during that startup the message's liveness stands in.
  After the init Dom's card also sends controls only with a valid `carControl`; the interceptor reports only
  once it is addressed, so during the same pending state card applies an inactive CarControl (Dom's own
  keepalive pattern) until the feedback arrives. Verified on the CX-9 on 2026-11-25: with d8590976 the alert
  is gone and the interceptor reports.
- Verified here (x86 builds of the libraries, restored to the device's aarch64 binaries before the commit): 482 tests
  in the Mazda, params, controller, card-hook, steer-limit, tethering and prime-state suites and 85 in Dom's Mazda
  safety, params, torque-controller, CAN-subscription and cache suites. Dom's AOL deadline test needs the acados
  solver, which was not built here.

**Not carried, and why.**
- The rain driving offsets from the wipers (WeatherChecker.apply_wipers): Dom has no weather driving offsets
  to attach them to; the lane-change half of "Wipers Mean Rain" is there.
- Lower Minimum Set Speed's StarPilot params part (redneckCruiseAvailable / pcmCruiseSpeed): Dom moved
  `get_starpilot_params` out of opendbc and the feature is off with hybrid long anyway. The toggle is listed but inert.
- The install-backup changes (offroad hold, no git history), the Galaxy dashboard stats and params fix, the
  lateral_maneuvers tool edits: Dom's replacements of those modules are different code.
- card.py's strip-the-long-flag fallback when the radar refuses to go silent (EcuDisableFailed): Dom has no
  such block; hybrid long never silences the radar at init, so it does not arise on this car.
- BlendedACC's reset on a mode change in longcontrol (radar interceptor path only).
- The StarPilot 6 `mici` (comma four) settings entries.

**Dom's prebuilt rules.** `prebuilt` is present, so the device never builds. Every commit that changes a
"protected" file (anything but .py under the UI/software/galaxy roots) must refresh the receipt, or Dom's fast
updater refuses the update: run `python3 openpilot/common/prebuilt_manifest.py .` after staging, then commit the
new `prebuilt.json`. Every committed shared library is aarch64; never commit a locally built one. After any x86
test build, check before committing that `git ls-files -z | xargs -0 file | grep ELF | grep -v aarch64` prints
nothing. The first push of this branch shipped the x86 `libqpOASES_e.so.3.1` from the test build: the device's
loader skips a library of the wrong architecture and reports it as missing, so every process that imports the
longitudinal MPC (the UI included) died on import and the boot logo stayed on screen until the file was restored
from Dom's snapshot. Dom's own channel-migration hook (Dom -> testing) was removed from the launcher on this
branch.

**Needs the car.** Steering feel at low speed on the new controller (above); the hybrid switch on the distance
button and the onroad icon; the horn bookmark; the hotspot modes; that the learned tune now shows `useParams`
on every drive (`tune_export.py`'s `ltp_use` column); that pandad flashes the rebuilt firmware once.

**Installing.** On the device: `cd /data/openpilot && git fetch origin sp7-mazda-long-ti1 && git checkout
sp7-mazda-long-ti1`, reboot. The first boot on the 0.11 base recalibrates and may take a few minutes.

## Rules the owner set

- Never push to `mazda-long-ti1`. All testing goes to `mazda-long-ti1-testing`, the only branch to push.
- Do not touch `mazda-hybrid-long`. Never force-push. Leave `stash@{0}` (a blocked lead-based WIP) alone.
- If something does not match, stop and report rather than guess.
- No model identifiers in commits or code.

## The device

- comma three on the car, SSH as `comma` at `192.168.50.226` on the home Wi-Fi (key `~/.ssh/comma` on the PC
  `LinuxSandwich`); on its own hotspot it is `192.168.43.1`.
- Logs: `/data/media/0/realdata_konik/<route>--<segment>/rlog.zst`, one segment a minute.
- The automatic updater installs anything pushed to the branch as soon as the car is off, then reboots. A push
  while a long tool runs on the device kills the run. Push, wait for the reboot, then run.
- Running a tool over plain ssh: `cd /data/openpilot && source launch_env.sh && export PYTHONPATH=/data/openpilot`
  first (the system python has no capnp). A copy of a tool at `/data/<tool>.py` also works and survives updates.
  Long runs: `nohup ... > log 2>&1 &`; `tune_export.py` takes about 15 s per segment. `Params()` from such a
  shell reads `/tmp/params`, not `/data/params/d`: read the param files directly.
- The PC clone `~/openpilot` has the fork as `origin` with this branch checked out (no `mine` remote needed).

## Tools in `tools/mazda/`

- `tune_export.py`: per-route 20 Hz rows while engaged (steering, longitudinal, hybrid, body switches, the
  live tune, its gates: the toggles controlsd ran on from the logged toggle broadcast, the message's validity,
  torqued's inputs and the message interval) plus an events file. `--days N --out <new dir>`; it skips
  routes already in the out dir. Tar the dir and copy it to the PC.
- `can_census.py`: what is on the buses, with `--watch` (change timeline for addresses) and `--dump --window`
  (every frame of a stretch to a gzipped CSV).
- `can_dump_events.py`: a dump to a timeline of decoded-signal and undecoded-byte changes.
- `analysis/`: the offline readers used for the write-ups (see its README).
- `CONTROLLABILITY.md`: what each decoded CAN signal is, and what the fork now does with it.

## What changed this week (newest last)

1. TI ceiling 700 with a guard, speed-shaped lateral gain and friction for the TI, correct CX-9 wheelbase and
   mass, zoompilot's steer-limit classifier behind `MazdaSteerLimitClassifier` (off by default).
2. The resume-from-stop fix for radar emulation (keep the brake released until the car moves).
3. DBC decodes from two parked action recordings: light switch and lever, wipers and washers, horn, lock
   switch, window switch, wheel buttons, lane-keep and DSC OFF buttons, off-road traction button, lamps,
   reverse, parking brake, auto hold states, liftgate position, ignition bits.
4. Car state reads them: parking brake blocks engagement, liftgate counts as a door, TCS off is espDisabled
   (alert only with `MazdaTcsOffAlert`), blinkers from the stalk, hazards hold the hybrid resume, the
   lane-keep button is the LKAS button (and the camera's lane-keep state is ignored on TI cars), gear is kept
   through the gap between positions. Toggles `MazdaWiperWeatherMode` and `MazdaHornBookmark`. The custom
   car state gained six fields, so a device rebuild happens on first boot after the update.
5. The live-tune gate found and fixed (190f74d0, on the device since the evening of 27 Oct): the StarPilot
   process read torqued's cached `LiveTorqueParameters` with the wrong capnp struct, so `has_auto_tune` and
   `liveValid` were bits of the cache's timestamp. `read_live_torque_cache()` in `starpilot_variables.py` reads
   it as the event torqued writes, with a regression test. The export gained the `fat fato cust_laf cust_fric
   ltp_dt` columns and a `toggles` event; `analysis/gate.py` reads them.
6. Hotspot shares the LTE connection (1 Nov). Two commits. The first widened the forwarding rule (off only on a
   comma prime data SIM, `PrimeState.uses_comma_sim()`); harmless but not the cause here, since this device
   reports prime type 2 (lite) and forwarding was already on. The cause, from NetworkManager's journal: its
   shared mode runs dnsmasq fine (phones get 192.168.43.x) but every `/usr/sbin/iptables` call it makes fails
   with exit code 4, because that binary is the nf_tables build and the comma three's 4.9 kernel has no
   nf_tables, so the MASQUERADE rule never lands and nothing is routed out. `system/ui/lib/tethering_nat.py`
   installs NetworkManager's three rules through `iptables-legacy` (which works on this kernel) 5 s after
   tethering comes up and removes them when it goes down; the Wi-Fi manager logs a "tethering nat" event with
   the per-rule result. Unit tests in `system/ui/lib/tests/test_tethering_nat.py`. To confirm on the car:
   turn tethering on, join a phone; afterwards `grep "tethering nat" /data/log/swaglog.*` should show "added"
   for all three rules and `sudo iptables-legacy -t nat -S POSTROUTING` the MASQUERADE line while it is on.
7. Tethering by driving state (3 Nov, owner's request: hotspot on while driving, off when parked). The network
   panel's on/off toggle is now an Off / Always / Onroad selector on the existing `TetheringEnabled` param (0/1/2;
   the old Qt UI had the same values, the new UI had dropped it). `system/ui/lib/tethering_auto.py` runs on its
   own thread from the panel (built at UI start), every 2 s compares the wanted state (mode + `ui_state.started`)
   with the hotspot and switches on a mismatch, setting forwarding first; retries a non-taking attempt after
   20 s, waits for the Wi-Fi backend (`WifiManager.tethering_available()`), logs "tethering auto". Offroad the
   device leaves the hotspot and rejoins its saved Wi-Fi. Tests in `system/ui/lib/tests/test_tethering_auto.py`.
   The compact (comma four) UI is unchanged.

## Findings from the exports (3 days + today, 15 routes)

- The interceptor never faulted on the new ceiling; the guard dips within about a second. (The export used to
  count the TI's power-up frame as a fault; fixed.)
- Highway steering is unchanged by the torque change, as intended. Low speed is still too thin to judge: a
  town loop with hands off in turns is the drive that would settle it.
- Driver-pressed at low speed is real hands in sharp turns, not the sensor.
- The important one, now explained: the learned lateral tune (gain about 2.1 vs the fixed 1.76, offset -0.22
  m/s^2, both steady across days) reached the controller on 2 of 8 long drives (routes 58 and 61, from their
  first highway minute to the end: the controller latches the live values once taken). Mazda is not in
  torqued's `ALLOWED_CARS`, so the live tune applies only through Force Auto Tune, and that toggle has been ON
  since 2026-10-06 (`/data/params/d/ForceAutoTune` = 1, `ForceAutoTuneOff` = 0; the older export tool did not
  record those keys, which is why the snapshots looked identical). The toggle broadcast controlsd runs on
  (`starpilotPlan.starpilotToggles`) nevertheless carried `force_auto_tune` False on the drives without the tune
  and True on the two with it, and it flipped to False 90 s into route 61. Cause: `starpilot_variables.py`
  parsed the cached `LiveTorqueParameters` (a whole capnp *event*, as torqued writes it) as the bare
  `LiveTorqueParametersData` struct, so `has_auto_tune` was bit 1 and `liveValid` bit 0 of the cache's
  `logMonoTime`: a coin flip at every cache write (every 12 s of driving). `has_auto_tune` True hides Force Auto
  Tune, hence the random drives; the random `liveValid` also started torqued in decimated mode at random.
  Verified on the device by parsing the cache both ways, and by the re-export of routes 50-63 with the new
  columns (`~/tune_export_gate_20261027.tar` on the PC, `analysis/gate.py`): the toggle broadcast (`fat`) was 0
  for the whole of every drive without the tune and 1 for the whole of route 58; in route 61 it was 1 until a
  recompute 31 s in and 0 after, by which time controlsd had already latched the live values (first valid
  message at 15 s; engagement at 125 s). The message itself was valid from about 12 s into every route
  (`ltp_valid`, `pose_ok`, `lag_ok`, `cal` all 1.00), so validity never was the gate. Without the live tune the
  integrator holds the whole 0.23 bias and the gain is 20% lower:

  | route | build | fat | ff intercept | mean i | learned offset | verdict |
  |---|---|---|---|---|---|---|
  | 50, 52, 53, 54, 57 | 17c2993 | 0.00 | -0.04 .. -0.00 | +0.22 .. +0.28 | -0.21 .. -0.25 | not applied |
  | 58 | 17c2993 | 1.00 | +0.234 | -0.005 | -0.244 | applied |
  | 5b | 17c2993 | 0.00 | -0.007 | +0.245 | -0.220 | not applied |
  | 5d | 7f9512b | 0.00 | -0.008 | +0.219 | -0.215 | not applied |
  | 60 | 1230769 | 0.00 | -0.001 | +0.246 | -0.210 | not applied |
  | 61 | 07edfa1 | 1 then 0 at 31 s | +0.208 | +0.106 | -0.219 | applied (latched) |
  | 62 | 07edfa1 | 0.00 | -0.001 | +0.226 | -0.223 | not applied |

## Next steps

1. Done 27 Oct: the cache-reader fix and the export/analysis changes are pushed (190f74d0, d04f6aa4) and
   installed; after the reboot the device's toggle broadcast reads Force Auto Tune on and liveValid from the
   real cache. With the fix, Force Auto Tune ON makes the live tune apply on
   every drive as soon as torqued's message is valid, which is the "apply on every drive" option without
   touching `ALLOWED_CARS`. Adding `mazda` there would additionally show the torque-calibration progress in the
   UI and make the toggle moot; not required.
2. After the next drives on the fixed build: `analysis/gate.py` on a fresh export should show `fat` = 1 all
   route and the `ff_fit` intercept near +0.21 on every route; `ltp_valid` and `ltp_dt` (3.2-4.8 Hz is what
   controlsd accepts) cover the remaining gate.
3. A town-loop drive for the low-speed shape; then `lat_compare.py` on it. (Still open: the 2-9 Nov drives
   have under a minute of hands-off steering per state below 9 m/s.)
4. Auto hold at engaged stops: the export's `hold` column and the stop events' `auto_hold`/`hazards` fields
   are waiting for the first engaged stops on the new build.
5. Still unidentified: the three sensor-like bytes of 0x09e; the climate bits; `DOORS_LOCKED` is tentative.
6. Earlier open items: the panda GEN1 TI torque-check gap in `opendbc/safety/modes/mazda.h`; the owner
   confirmed the long press on the mode switch works.

## Export of 2-9 Nov (routes 82-99, `~/tune_export_smart_20261109.tar`; build 5547cf2 / 8338e42, same lateral code)

- **Live tune: fixed for good.** `gate.py`: Force Auto Tune reached controlsd (`fat` = 1) and the ff_fit
  intercept is +0.17..+0.28 ("applied") on all 15 routes with enough highway. Highway mean integrator fell from
  +0.21..+0.23 (October) to about +0.06, and steering-limit alerts from 24.6/h (October) to about 3/h.
- **Smarter Steer Limit Flag** (`MazdaSteerLimitClassifier`): off on 2 Nov (routes 82-85, 16 min lateral), on
  from 3 Nov (94 min). `analysis/flag_compare.py`: integrator frozen 5.7% -> 3.8% of movable frames, as designed;
  |error| 20-27 m/s 0.060 -> 0.052, 15-20 m/s 0.075 -> 0.061, 27-40 m/s 0.050 -> 0.053; speed-mix-weighted
  0.057 -> 0.055. Saturation alerts 3.3/h -> 2.5/h (5 alerts in all, too few to rank). A small gain, no harm
  seen; the off sample is one day. Left on. Below 9 m/s too thin to judge (under a minute per state).
- **TI violation codes, decoded** from the original TI's documentation (MoreTorque "TI General Documentation",
  Google Doc 17faej3N3h-GKf32WHjsLmEe10jIoHXrlPkugUnHBZcA, Part 2): the VIOL byte is the green-LED code as two
  hex digits. Class 2 recovers by itself 30 s after the condition clears, then RUN 2 s later (the ~31 s outage).
  Table in `opendbc/car/mazda/torque_interceptor.py` (`VIOLATIONS`, `violation_name()`); the export and
  `ti_fault_dump.py` now print the names.
  - 0x11 `LKAS_STUK`: "LKAS request input does not change when not 0 or +/-600", i.e. any held nonzero value,
    not only the ceiling. Route 97 tripped it at 0.5 mph with -1 count held 2.0 s. `TiCeilingGuard` now also
    lowers any nonzero command held unchanged for 1.2 s by 20 counts toward zero (to zero when smaller) for
    0.2 s. Across 22 drives only the ceiling (guard already) and |cmd| <= 2 at a creep were held >= 0.5 s.
  - 0x31 `HIN_ERR`: EPS hall input out of window; route 8e at standstill at the end of the drive (power-down).
  - 0x33 `TK_WLOAD`: the TI's measured output out of window under load, a check of its own analog output stage.
    Route 99 (9 Nov 20:03): 10 times in 13 minutes at small commands (-190..+139 counts), 8-60 mph, no driver
    torque, clean 50 Hz timing; each time the 30 s recovery, RUN, and again within 0.1-10 s; then it stopped.
    The drive before had none. Not something openpilot sends: wiring/connector or the unit itself. Owner to
    reseat the TI connectors; if it comes back, MoreTorque support (Discord) with route 99's dump.

## Upstream survey: StarPilotHQ/StarPilot branch `Dom` (looked at 2026-10-30)

- `Dom` is the devs' live branch for "StarPilot 7.0 — Domathon": a rebase of StarPilot onto current openpilot
  (commaai 2ab952d5 of 2026-10-19, i.e. 0.11.2 plus the 0.11.3 Autospeed work; big 880M model, external-GPU
  model runtime `tinygrad_model_runtime/`, comma four). Its README and RELEASES.md say it is not a finished
  release: "feature migration, vehicle qualification, and Auto Learn validation are still in progress".
  `Codename-Domi` stopped 2026-09-18 and `Dom-3.24.26` 2026-03-21; neither is the one meant.
- History is rewritten: no common ancestor with our branch (GitHub compare 404), so a merge or rebase is a
  re-port, not a `git merge`. The tree is restructured too: openpilot code under `openpilot/`, StarPilot
  split into `openpilot/starpilot/{aol, flm, galaxy, lateral, longitudinal, controllers, companion, ...}`,
  vendored `mapd_repo`, `jetlink_repo`, `sentry_sdk_repo`. 1709 commits since our upstream snapshot
  (last firestar5683 commit in our history: 2026-08-25), 1488 of them in October.
- Against our tree (measured on the default branch `StarPilot`, which Dom extends): 1325 files differ,
  +143k/-40k lines; 399 files new upstream (147 in the Galaxy web app alone), 98 params keys added
  (Bluetooth audio, wheel-control slots, standby wake, model lab, custom accel profile breakpoints, screen
  brightness, Subaru/Tesla/Toyota options), 25 of ours absent there.
- Features new since our snapshot (commit subjects): Galaxy web app rewrite (localization, language
  selection, calibration reset, battery voltage, toggle-defaults reset, iOS/Android companion apps), Blendful
  lateral policy (alpha, from sunnypilot), both torque controllers offered on every torque car, torque
  tuning for every controller, lane centering on by default, Experimental Speed Nudge and group driving
  personalities, stopped-lead CEM on by default, Turn Signal Guidance on by default, Bluetooth audio and
  wheel controls daemons, Android Auto, fast updater with scheduled install, SpacePilot parked mode, many
  Toyota/GM/Ford/HKG/Subaru/Tesla vehicle additions, StarPort head-unit layouts.
- Live torque tune on Dom: rewritten (`openpilot/starpilot/lateral/torque_runtime.py` TorqueHost,
  `controller_selection.learning_allowed`, service renamed `lateralTorqueParameters`). The Force Auto Tune
  files are read directly, so our cache-reader coin flip does not exist there in that form. But torqued's
  `ALLOWED_CARS` is still toyota/hyundai/rivian/honda/volkswagen and `force_auto` comes only from the GM
  geometry runtime, so a Mazda on Dom gets no learned tune at all unless ported.
- Mazda on Dom: 13 commits in October, all stock Always-On-Lateral (`opendbc/car/mazda/stock_aol.py`,
  `safety/modes/mazda_stock_aol.h`, `starpilot/car/mazda/{aol,stock_intent,dashboard}.py`): MAIN-on lateral
  start, lateral kept through Cancel/Reverse/standstill, camera stop-sign observations. `qualified()` admits
  only CX-5 2022 / CX-9 2021 with flags exactly GEN1, pcmCruise and no openpilot longitudinal, safetyParam 0:
  it excludes our Torque Interceptor flag and our hybrid longitudinal. Nothing upstream about the TI, radar
  emulation, the hybrid handover, or the body-switch inputs.
- Our fork-only work vs upstream: 105 files / about 20k lines (opendbc mazda TI + radar session + tests,
  `safety/modes/mazda.h` TI section, steer-limit classifier, `tools/mazda*`, the Mazda toggles). Moving to
  Dom means re-porting all of it onto the restructured tree, plus the Mazda-specific live-tune admission.
- The StarPilot repo is a read-only remote `starpilot` in the PC clone (default branch fetched; a full
  fetch of `Dom` was still running when this was written). Survey facts came from the GitHub API.

## Compact log of the last days (what was asked, what was done)

**Fri 24 Oct, steering.** Owner: still running out of torque on steering commands, low speed sluggish and slow.
From the tune export: the interceptor faults (STATE OFF, VIOL 0x11) after about 1.9 s of an unbroken 800 command,
and the EPS torque sensor stops answering above about 650 counts. Done: TI ceiling 700 with a dip guard,
speed-shaped lateral gain and friction for the TI, the CX-9's real wheelbase and mass, zoompilot's steer-limit
classifier behind a toggle (owner asked to look at zoompilot and at the other branches' torque controllers:
nothing TI-specific anywhere). Also: resume from a stop was not happening on its own; fixed by keeping the brake
released until the car moves. 185 lateral tests, 59 Mazda tests, panda harness runs.

**Sat 25 Oct, the bus.** Owner asked what lives on the CAN bus and could be controlled (blinkers, high beams).
Built `can_census.py`; three minutes of one drive identified RPM, speed, acceleration and steering-angle bytes,
multiplexed frames, the VIN text broadcast, node beacons. Owner: "dig into the unknown third." Gave an action
recording protocol; owner recorded at 8:05 pm (route 00000060) and later an all-parked afternoon session
(route 0000005f). Added `--dump` and `can_dump_events.py`; matched every byte change to the switch operated;
about forty signals added to the DBC (light switch and lever, wipers, washers, horn, lock switch, window
switch, wheel buttons, lane-keep and DSC OFF buttons, off-road traction button, lamps, reverse, parking brake,
auto-hold states, liftgate position, ignition bits), the last five named from the owner's answers. The device
rebooted twice mid-run: its updater installs each push when the car is off.

**Sun 26 Oct, using the inputs.** Owner asked what each signal could control: wrote `CONTROLLABILITY.md`
(only the four existing outputs are commandable; everything decoded is an input). Owner asked for ideas,
then to implement them all, and noted the mode-switch long press works. Done: parking brake blocks
engagement, liftgate counts as a door, TCS off as espDisabled behind a toggle, blinkers from the stalk,
hazards hold the hybrid resume, the lane-keep button as the LKAS button (camera lane-keep state ignored on
TI cars at the owner's request), gear kept through the lever gap, auto hold and lamps logged, "Wipers Mean
Rain" and "Horn Tap Bookmarks" toggles, export columns. Three-day export read: no interceptor faults (the
counted one per route was its power-up frame, export fixed), highway unchanged, low speed too thin to judge,
a steady +0.23 integrator bias at highway speed, driver-pressed at low speed is real hands.

**Mon 27 Oct (UTC; Sun evening PDT), the live tune.** Today's two drives exported with the new offset column: torqued's offset is
-0.22 and steady; the integrator cancels it. Fitting the feedforward showed the live tune reaches the
controller on only some drives (2 of 8). Mazda is not in torqued's allowed list, so it applies only through
Force Auto Tune; which gate flips between drives is not yet known. Export instrumented to say; owner asked to
re-export today's drives and whether Force Auto Tune was changed. Owner then asked to move the session off
the cloud: analysis scripts and this note went into the repo; the PC's `~/openpilot` clone has no remote for
the fork yet (add `mine` for ladelisandwich/openpilot, or use whichever clone already has it).

**Mon 27 Oct, PC session, the gate.** Owner moved the work to the PC (clone ready, fork as `origin`) and the
comma came onto the home Wi-Fi. Traced every gate between torqued and the controller; the old exports showed the
offset present from the first highway minute of routes 58 and 61 only. On the device: Force Auto Tune has been on
since 6 Oct, yet the logged toggle broadcast said off on the drives without the tune. Found the wrong-struct parse
of the live-tune cache (bits of the timestamp as `useParams`/`liveValid`), fixed it with `read_live_torque_cache()`
and a regression test, verified on the device (19 fixture-free tests pass, lint unchanged). Export gained the
toggle-broadcast and message-interval columns; `analysis/gate.py` added. Re-exported routes 50-63 on the device
into `/data/tune_export_gate` with the new tool (`/data/tune_export.py`). Owner said to push once the export
finished: pushed 190f74d0 and d04f6aa4 over ssh (the https remote has no credential helper from a plain shell),
signalled the updater (it checks only once per boot; SIGHUP is what the Download button sends), the device
rebooted onto d04f6aa4 and its broadcast now reads Force Auto Tune on. This note's wording was corrected in a
later docs-only commit.

**Sun 9 Nov, the flag.** Owner drove with the Smarter Steer Limit Flag off, then on, and asked to pull logs and
adjust. Exported routes 82-99 on the device; the live-tune fix holds on every drive. The flag helps a little
and is left on. Found the TI's own violation table: route 97's stuck fault came from a -1 count held at a creep,
so the guard now covers any held command; route 99's ten 0x33 faults are the TI's output stage (hardware check
for the owner). `analysis/flag_compare.py` added; tools name the codes. Pushed.

