# SpacePilot

In native Developer settings, tap the **version number** five times. Both the
comma four and Big UI provide the same gesture and native game. Release each tap
within 450 ms without dragging; tap at your own pace, with no more than 1.5 seconds
between taps and six seconds overall. Distinct releases are counted from their
touch timestamps even when delivered together in one UI frame. Scrolling, another
row, navigation, a long press, or loss of physical parked evidence resets progress.
Taps count while valid Park evidence is settling; the preparation screen still
requires the full physical dwell before it can request Offroad or accept input.
Holding the brake does not prevent entry. A pending C4 startup camera redirect
does not dismiss a game with fresh parked evidence; movement and unsafe exits
retain their normal behavior.

Keep the vehicle in Park and disengaged while the preparation screen verifies
its state and stops driving services. Play begins only after those services are
confirmed stopped and every connected Panda reports healthy `noOutput`.
Tap to start, boost, and retry. With ignition on, the wheel's + and − are observed
through an independent receive-only CAN decoder. Release a wheel button once to
arm it; each following press boosts once. Held/repeated inputs do not repeat.

Offroad with ignition off, use touch or a paired USB/Bluetooth controller. Enable
and configure the controller's existing **Increase Set Speed** / **Decrease Set
Speed** bindings in Controller settings. In SpacePilot those bindings only boost
the ship; no cruise request is dispatched. All controller input in the game's
input batch is consumed, even if the first press closes the game. Exiting restores
ordinary controller routing, subject to its original control authority checks.

## Lifecycle and physical evidence

Onroad admission retains the original one-second fresh physical Park/standstill,
zero ego/raw/all wheel speed, valid CAN, disengaged ACC/controls, healthy connected
Panda, and no native alert requirements. Replay/projection, unknown state and
host overrides cannot admit the game. The version row owns only the gesture;
the page owns its own admission and cleanup state.

The game requests Offroad through the existing session-bound `DriveStateOwner`.
It does not falsify vehicle state, change manager process predicates or native
safety, initialize a vehicle interface/controller, send CAN, or alter alerts.
Normal `pandad` retains raw CAN reception offroad and selects `noOutput`. A
separate bounded subscriber instantiates only the current vehicle's normal
`CarState` and CAN parsers, retaining checksum/counter/timeout checks. Required
CAN frames additionally must remain within 250 ms; unrelated packets cannot keep
a remembered Park or zero speed alive. Unsupported or incomplete decoding fails
closed. Current live-session `carParams` is required for ignition-on play.

The game waits for actual `card`, `selfdrived`, `controlsd`, `modeld`, and
`dmonitoringmodeld` shutdown. Fresh manager/device/Panda source and receipt clocks,
physical noOutput, independent Park evidence, clock continuity and input bounds
remain mandatory. With ignition off, fresh healthy noOutput and actual stopped
processes provide the existing physical offroad premise; wheel input is disabled.
A mode request alone never proves that the car is parked. Changing ignition,
mode, gear, speed, engagement, process state, faults, source freshness or clock
continuity ends the game. Entry/shutdown times out after eight seconds.

A successful onroad entry leaves Offroad selected on ordinary game exit. A small
UI observer retains only that exact game-owned request revision. After every
connected Panda continuously reports ignition off for one second, with fresh
source/receipt clocks, unchanged Panda topology/uptime and pandad process, healthy
noOutput and stopped driving services, it closes any game and requests Auto through
the existing owner. Auto remains offroad while ignition is off; the next ignition
uses normal startup checks. A single off report, stale/missing/faulted telemetry,
clock discontinuity, reconnect or external mode/session change cannot release the
request. The observer also runs while the normal offroad screen is asleep.

The observer belongs to this UI process. UI restart does not adopt a prior request;
if observation is interrupted, use the existing drive-mode selector to choose
Auto. Manager/system restart retains its existing Auto initialization. There is
no new persistent interlock or blanket Auto request on page exit/crash.

Successful active offroad game ticks renew the existing display activity timer,
including the ready and game-over screens. No saved timeout is changed. Exiting,
unsafe shutdown, render failure or stopped ticks stops renewal, and the configured
offroad timeout applies normally.

## Validation boundary

Tests cover physical admission, stale source/receipt clocks, process/noOutput
transitions, the actual drive-state owner, touch/wheel/controller isolation,
normal CAN decoding and refusal, both native version gestures, bounded simulation
and cleanup. Native GL previews and host CPU/RSS/frame-time measurements use
isolated synthetic telemetry. They do not establish physical C4/C3 performance,
actual wheel/controller hardware behavior, full-system headroom or deployment.
No road-device installation, restart, or driving test is part of host validation.

Clock reads interrupted by scheduling are retried up to three times, with the
same 1 ms sampling bound and resume/freshness checks on the accepted sample.
Cold receive-only decoder construction yields to the next normal UI update
before admission; it never grants gameplay or Offroad from an aged UI snapshot.
