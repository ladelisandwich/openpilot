# CX-9 GEN1 CAN signals: what each one could control

A feasibility reference for the signals decoded on this car (2023 CX-9, GEN1 EPS + TI1),
built from the two parked action recordings on 2026-10-25 and the CAN census. It answers a
narrow question: for each discovered signal, could openpilot *control* the function behind
it, and what would that take.

Read the two gates first. They decide almost everything below.

## Gate 1: the panda transmit allowlist

The panda safety firmware only lets specific frames leave for the car. On this car's
configuration (`opendbc/safety/modes/mazda.h`, the TI + radar-emulation build) the entire
set openpilot may transmit is:

| Address | Name | Bus | Purpose |
|---|---|---|---|
| 0x243 | LKAS | 0 | steering torque command to the EPS |
| 0x249 | TI LKAS | 1 | steering torque to the Torque Interceptor |
| 0x09d | CRZ_BTNS | 0 | cruise-button presses |
| 0x440 | LKAS_HUD | 0 | lane-keep dash icons and chimes |
| 0x21b, 0x21c | CRZ_INFO, CRZ_CTRL | 0 | radar-emulation cruise frames |
| 0x361-0x366, 0x499 | RADAR_* | 0 | radar-emulation track frames |

Every other address is blocked. The panda drops it before it reaches the wire, whatever the
software asks. So a signal that is not in a frame on this list cannot be driven at all without
first widening the allowlist in the safety firmware, which is a safety change, not a tuning
change. Treat "would need a new TX entry" below as "blocked today."

## Gate 2: status report vs. command

A CAN signal is one of two things:

- **A command / request** that some module *listens for* and acts on. These are the only
  signals that control anything. Example: CRZ_BTNS is read by the cruise module, so sending it
  presses a cruise button for real.
- **A status report** that a module *broadcasts* about its own state. Almost everything we
  decoded is this: speed, RPM, gear, the lamp states, the wiper states, the door and lock
  states, the button-pressed bits. Sending a copy of one of these does not actuate anything.
  The real module keeps broadcasting the true value at the same time, so a listener sees two
  conflicting frames and usually faults or ignores the intruder. Spoofing a status frame can
  at best *lie to a listener* (make the dash think a door is open); it cannot *open the door*.

The button and switch bits we named (horn, wipers, lock switch, window switch, wheel buttons,
DSC OFF, lane-keep) are all in the second category. They are the body module *reporting that
you pressed something*. The actuator (horn relay, wiper motor, window motor, lock solenoid)
sits downstream of the module and obeys the module's own logic, not a replayed "button
pressed" bit. So replaying them does not honk, wipe, lock, or roll a window.

## The realistic controllable set

Given both gates, the functions openpilot can actually command on this car are:

1. **Steering** — 0x243 / 0x249, already used. Torque to the EPS and the TI, capped at the
   TI ceiling.
2. **Cruise buttons** — 0x09d CRZ_BTNS, already used. Resume, set+, set-, cancel, distance,
   mode. This is a true command: the cruise module acts on it. It is the one body-control
   input openpilot already owns, and the only general-purpose "press a button" channel that is
   both a real command and on the allowlist.
3. **Longitudinal via cruise** — the radar-emulation frames, already used. Speed and
   gap control by emulating the radar to the cruise module.
4. **Dash lane-keep feedback** — 0x440 LKAS_HUD, already used. The LKAS icons and the
   hands-on / lane-departure chimes. This is a command the instrument cluster obeys, so the
   chime and icon content is genuinely controllable.

Everything else below is either blocked by the panda, a status report, or both.

## Per-signal assessment

### Steering and cruise (already commanded)

| Signal | Msg | Control potential |
|---|---|---|
| LKAS torque | 0x243 | Commanded today. Steering. |
| TI LKAS torque | 0x249 | Commanded today. Steering via the interceptor. |
| CRZ_BTNS.* (RES, SET_P, SET_M, DISTANCE_*, MODE_*, CAN_OFF) | 0x09d | Commanded today. Real cruise-button presses. The distance and mode bits reach the cruise module; the wheel's own buttons are mirrored here. |
| LKAS_HUD content | 0x440 | Commanded today. Dash lane-keep icons and chimes. |
| CRZ_INFO / CRZ_CTRL / RADAR_* | 0x21b, 0x21c, 0x361-366, 0x499 | Commanded today via radar emulation. Longitudinal only. |

### Body status — read-only, not on the allowlist

These are useful to *read* (openpilot already reads several), but they report state; they do
not command it, and the panda blocks transmitting them regardless.

| Signal(s) | Msg | What it reports | Control potential |
|---|---|---|---|
| HORN | 0x091 | horn button pressed | None. The bit is the switch report; the horn relay is downstream of the body module. |
| WIPER_SWITCH, WIPER_MIST, REAR_WIPER_SWITCH, REAR_WASHER, FRONT_WASHER | 0x091, 0x154 | wiper stalk / knob position | None by replay. Wipers follow the body module, which reads the physical stalk, not the bus echo. |
| HEADLAMPS_REQ, PARKING_LAMPS_REQ, LIGHT_SWITCH_OFF, LEVER_PULLED, LEVER_FORWARD | 0x091 | light switch and lever position | None by replay. Lamp output follows the switch position the module reads directly. |
| LAMPS_ON, LAMPS_ON_2, ILLUMINATION, REVERSE_LAMP | 0x09f | lamp and illumination state | Read-only status. |
| LOCK_SWITCH, DOORS_LOCKED, DOORS_UNLOCKED, KEYFOB_* | 0x4e9, 0x43e, 0x1086 | lock switch pressed, lock state | None by replay. The lock solenoid obeys the body module and the key, not a replayed state bit. |
| WINDOW_SWITCH, WINDOW_SWITCH_ACTIVE | 0x274 | driver window switch direction | None by replay. Window motor follows the physical switch at the module. |
| LIFTGATE_OPEN, LIFTGATE_STATE, LIFTGATE_POSITION | 0x43e, 0x1139 | power liftgate motion and position | Read-only status. |
| STEERING_WHEEL_BUTTON | 0x274 | a left-spoke wheel button is pressed | Read-only, and it collapses all five buttons to one value, so it can't even distinguish them. |
| LAS_BUTTON, DSC_OFF_BUTTON, OFFROAD_TRACTION_BUTTON | 0x09e, 0x158 | those buttons pressed | None by replay. |
| TCS_OFF, TCS_OFF_2, BRAKE (auto hold), HOLD_STATE | 0x415, 0x529, 0x1045, 0x121 | traction and auto-hold state | Read-only status. |
| GEAR, GEAR_BOX, MORE_GEAR, SHIFT_IN_PROGRESS, REVERSE | 0x228, 0x445 | gear position | Read-only status. |
| BSM_MODE | 0x1143 | blind-spot monitor mode | Read-only status. |
| DRIVER_SEATBELT, FRONT_WASHER, DOORS.FL/FR/BL/BR | 0x340, 0x154, 0x43e | belt and door ajar | Read-only status. |
| SPEED, RPM, WHEEL_SPEEDS, STEER_ANGLE, accelerations | 0x202, 0x25e, 0x215, 0x075, 0x076 | motion sensors | Read-only. openpilot already consumes these. |
| IGNITION_ON (many msgs) | 0x09e, 0x274, ... | ignition state | Read-only status. |

### Still unidentified

| Signal | Msg | Note |
|---|---|---|
| UNKNOWN byte 2/5/6 (sensor-like) | 0x09e | Three analog bytes in the column module block, not yet tied to a function. |
| 0x4e9 LOCK_SWITCH tentative | 0x4e9 | Behaves as a lock-switch report; did not move DOORS_LOCKED, so the mapping is provisional. |
| climate bits | 0x436 | Fan/temp/defrost overlap; not separated into clean per-function bits. |

## Bottom line

- The only functions openpilot can command on this car are the four it already uses: steering,
  cruise buttons, the radar-emulated longitudinal path, and the lane-keep dash feedback.
- The newly decoded signals are valuable as **inputs** — knowing the real lock, gear, lamp,
  wiper, liftgate, and button states lets the fork *react* to them (e.g., refuse to engage with
  the liftgate up, read the physical RES button, mirror a blinker). That is where the payoff is.
- None of them is a new **output**. Honking, wiping, locking, or rolling a window would each
  need two things that do not exist today: a frame the relevant module accepts as a *command*
  (most body functions have none on the comfort bus the comma can see — they are driven inside
  the module from the physical switch), and a new entry in the panda transmit allowlist. The
  switch-report bits we named are not those command frames.
