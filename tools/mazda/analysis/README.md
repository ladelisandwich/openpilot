# Mazda tune-export analysis scripts

Offline readers for what `tools/mazda/tune_export.py` writes (a directory of `<route>.csv.gz`,
`<route>.events.csv.gz` and `<route>.json`). They need pandas; pass a directory holding it as the
second argument if it is not installed. Each takes the export directory as its first argument.
Old build means 17c2993 (before the torque change); point several exports at one directory of
symlinks to compare across them.

| Script | What it prints |
|---|---|
| `lat_compare.py` | steering by speed bin, old vs new build: error, integrator, command, ceiling share, saturation, tracking |
| `lat_more.py` | driver-pressed share and the TI torque sensor by speed; signed biases at highway speed per route |
| `leak.py` | the TI torque sensor against the TI command (does the command leak into "driver pressed"), PID log sanity |
| `prs_effect.py` | what a driver-pressed flag does to the command below 20 m/s, and the pressed episodes |
| `offset.py` | torqued's live lateral offset against the integrator bias, per minute (needs the `laoff` column) |
| `gate.py` | per route, the gate inputs for the live torque tune (Force Auto Tune toggles, liveTorqueParameters validity and torqued's inputs), when the message first went valid, and the ff_fit intercept with a verdict |
| `ff_fit.py` | fits the logged feedforward to setpoint and roll: an intercept near the learned offset means the live tune reached the controller, near zero means it did not |
| `census_diff.py` | which bytes gained values between two `can_census.py` reports |
| `watch_parse.py` | reads a `can_census.py --watch` timeline for a window: context changes and rare byte changes per address |
