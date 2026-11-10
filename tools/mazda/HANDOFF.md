# Mazda CX-9 (GEN1 + TI1) work: handoff

State of the work on `mazda-long-ti1-testing` as of 2026-10-27 (evening, PC session), written so a new session,
local or cloud, can continue without the old conversation.

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

