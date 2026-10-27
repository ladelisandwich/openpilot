# Mazda CX-9 (GEN1 + TI1) work: handoff

State of the work on `mazda-long-ti1-testing` as of 2026-10-27, written so a new session, local or
cloud, can continue without the old conversation.

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
- The device runs `python3 tools/mazda/<tool>` from `/data/openpilot`. A copy of a tool at `/data/<tool>.py`
  also works and survives updates.

## Tools in `tools/mazda/`

- `tune_export.py`: per-route 20 Hz rows while engaged (steering, longitudinal, hybrid, body switches, the
  live tune and whether it reached the controller) plus an events file. `--days N --out <new dir>`; it skips
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

## Findings from the exports (3 days + today, 15 routes)

- The interceptor never faulted on the new ceiling; the guard dips within about a second. (The export used to
  count the TI's power-up frame as a fault; fixed.)
- Highway steering is unchanged by the torque change, as intended. Low speed is still too thin to judge: a
  town loop with hands off in turns is the drive that would settle it.
- Driver-pressed at low speed is real hands in sharp turns, not the sensor.
- The important one: the learned lateral tune (gain about 2.1 vs the fixed 1.76, offset -0.22 m/s^2, both
  steady across days) reaches the controller on some drives and not others. Mazda is not in torqued's
  `ALLOWED_CARS`, so `useParams` is false and the live tune applies only through Force Auto Tune
  (`ForceAutoTune` / `ForceAutoTuneOff` params, `starpilot_variables.py` ~line 715; controlsd ~line 428).
  `analysis/ff_fit.py` tells per drive: intercept near +0.21 means applied, near 0 means not. Without it the
  integrator holds the whole 0.23 bias and the gain is 20% lower. Of 8 long drives, 2 had it (one old-build
  drive and today's first), 6 did not, with identical settings in the snapshot.

## Next steps

1. Re-export today's drives (`--days 1`, a fresh out dir) once the device runs c1a4704 or later: the new
   `ltp_valid ltp_use pose_ok cal lag_ok` columns and the `ForceAutoTune*` params in the snapshot say which
   gate flipped. Ask the owner whether they changed Force Auto Tune between drives.
2. Then decide: make the live tune apply on every drive on this car (add mazda for this fork, or set the
   gate for TI cars) or pin the fixed tune and give it the offset by hand. The steady estimates argue for the
   former.
3. A town-loop drive for the low-speed shape; then `lat_compare.py` on it.
4. Auto hold at engaged stops: the export's `hold` column and the stop events' `auto_hold`/`hazards` fields
   are waiting for the first engaged stops on the new build.
5. Still unidentified: the three sensor-like bytes of 0x09e; the climate bits; `DOORS_LOCKED` is tentative.
6. Earlier open items: the panda GEN1 TI torque-check gap in `opendbc/safety/modes/mazda.h`; the owner
   confirmed the long press on the mode switch works.

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

**Mon 27 Oct, the live tune.** Today's two drives exported with the new offset column: torqued's offset is
-0.22 and steady; the integrator cancels it. Fitting the feedforward showed the live tune reaches the
controller on only some drives (2 of 8). Mazda is not in torqued's allowed list, so it applies only through
Force Auto Tune; which gate flips between drives is not yet known. Export instrumented to say; owner asked to
re-export today's drives and whether Force Auto Tune was changed. Owner then asked to move the session off
the cloud: analysis scripts and this note went into the repo; the PC's `~/openpilot` clone has no remote for
the fork yet (add `mine` for ladelisandwich/openpilot, or use whichever clone already has it).
