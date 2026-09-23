"""Mazda software hybrid longitudinal: stock MRCC <-> radar emulation, switched at runtime.

What this is, and what it is not
--------------------------------
MoreTore's hybrid ("BlendedACC") runs on a hardware Radar Interceptor. The stock radar never
goes silent: its frames arrive through the board and openpilot rewrites ACCEL_CMD in flight,
so moving between openpilot and MRCC is a filter on one number. That is impossible without
the board, because the radar and the powertrain share bus 0.

Here a "switch" is a change of ACC master. Taking over puts the stock radar into a UDS
programming session (it goes silent and openpilot impersonates it); handing back takes it out
again (it restarts and resumes as the ACC master). Consequences that are inherent, not bugs:

* MRCC -> emulation mid-drive: the car sees its radar vanish for a few frames. On a 2023 CX-9
  a mid-drive silence has produced "front radar error". If the cruise engagement drops right
  after a takeover, takeovers are switched off for the rest of the drive.
* emulation -> MRCC: the radar restarts from scratch and may come back disengaged, in which
  case the cruise has to be set again. Hand-backs are therefore only made at a speed where
  MRCC can be set again, never in the middle of a braking event, and at a standstill only
  while the driver holds the brake (when the emulated HOLD disappears the car must not creep).

Components
----------
RadarWitness        watches the stock radar's raw frames. Never through a CANParser: reading a
                    radar message through one subscribes the parser to it, and once the radar
                    is silenced the parser marks the whole bus invalid (canError, shown as
                    "Unknown Vehicle Variant").
HybridArbiter       decides which master is wanted: Conditional Experimental Mode status,
                    standstill, pedals, speed, and debounce/dwell so CEM flicker cannot thrash
                    the radar.
HybridRadarManager  runs the transitions over UDS and says when replacement frames may be sent.
"""
from __future__ import annotations

from enum import StrEnum

from opendbc.car import DT_CTRL, make_tester_present_msg, uds
from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog

RADAR_BUS = 0
RADAR_UDS_ADDR = 0x764
RADAR_UDS_STEP = 50  # UDS traffic at 2 Hz: session control or tester present
CRZ_INFO_ADDR = 0x21B
CRZ_CTRL_ADDR = 0x21C  # the radar's own ACC state: tells us whether MRCC is really driving
RADAR_COUNTER_ADDR = 0x361  # heartbeat track whose CTR seeds ours on a takeover
HEARTBEAT_ADDRS = (0x361, 0x362, 0x363, 0x364, 0x365, 0x366, 0x499)
WATCHED_ADDRS = frozenset((CRZ_INFO_ADDR, CRZ_CTRL_ADDR) + HEARTBEAT_ADDRS)
ACCEL_CMD_VALID = (-2000, 2000)  # stock standby frames carry 4094, which is not a command

# starpilot/common/experimental_state.py: 0 = OFF (standard) and 1 = USER_DISABLED (user forced
# standard). 2 = USER_OVERRIDDEN (user forced experimental), 3-8 = CEM triggered experimental.
# A bare "non-zero" test (MoreTore's BlendedACC) reads USER_DISABLED as experimental.
CE_STANDARD_STATUSES = frozenset({0, 1})


def ce_status_is_experimental(status: int) -> bool:
  return int(status) not in CE_STANDARD_STATUSES


def radar_session_msg(session_type: int) -> CanData:
  """Single-frame UDS DiagnosticSessionControl, sent from the control loop without blocking.
  Matches the panda's 0x764 allowlist: [0x02, 0x10, 0x01 | 0x02]."""
  return CanData(RADAR_UDS_ADDR, bytes([2, uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, session_type, 0, 0, 0, 0, 0]),
                 RADAR_BUS)


class RadarWitness:
  """Stock radar liveness from raw bus-0 frames, fed from CarInterface.update().

  Our own transmitted frames come back from the panda with src = bus + 128, so every src-0
  CRZ_INFO here is the stock radar. Time is counted in CAN packets (pandad publishes one every
  10 ms), so a late control loop that drains two packets at once cannot fake a silence.
  CRZ_INFO is a 50 Hz message: a live radar leaves at most one empty packet between frames.
  """
  SILENT_PACKETS = 6  # 60 ms without CRZ_INFO = three missed 50 Hz frames

  def __init__(self):
    self.frames: dict[int, bytes] = {}
    self.seen = False
    self.packets_since_crz_info = self.SILENT_PACKETS
    self.run_frames = 0  # CRZ_INFO frames since the radar last went silent

  def update(self, can_packets) -> None:
    for _, frames in can_packets:
      got = 0
      for address, dat, src in frames:
        if src == RADAR_BUS and address in WATCHED_ADDRS:
          self.frames[address] = bytes(dat)
          if address == CRZ_INFO_ADDR:
            got += 1
      if got:
        if self.silent:
          self.run_frames = 0
        self.run_frames += got
        self.seen = True
        self.packets_since_crz_info = 0
      else:
        self.packets_since_crz_info = min(self.packets_since_crz_info + 1, 1_000_000)
        if self.silent:
          self.run_frames = 0

  @property
  def alive(self) -> bool:
    return self.seen and self.packets_since_crz_info < self.SILENT_PACKETS

  @property
  def silent(self) -> bool:
    return self.packets_since_crz_info >= self.SILENT_PACKETS

  @property
  def stock_accel_cmd(self) -> int | None:
    """Last ACCEL_CMD the stock radar commanded, raw units (same unpacking as the panda).
    None when there is no frame or the frame carries no command (standby)."""
    d = self.frames.get(CRZ_INFO_ADDR)
    if d is None or len(d) < 8:
      return None
    cmd = ((((d[2] & 0x3) << 11) | (d[3] << 3) | (d[4] >> 5)) - 4096)
    return cmd if ACCEL_CMD_VALID[0] <= cmd <= ACCEL_CMD_VALID[1] else None

  @property
  def stock_cruise_active(self) -> bool:
    """CRZ_CTRL.CRZ_ACTIVE from the stock radar: it says ACC is engaged.
    Only meaningful while the radar is alive -- the frame is the last one it sent."""
    d = self.frames.get(CRZ_CTRL_ADDR)
    return bool(d is not None and len(d) >= 8 and (d[0] & 0x08))

  @property
  def mrcc_driving(self) -> bool:
    """True only while the stock radar is really driving the car.

    Two independent signals, because the failure mode has to be caught with the one that was
    actually observed: a radar that restarts in standby keeps sending ACCEL_CMD 4094 (no
    command at all -- stock_accel_cmd None in every field log of the fault) while the PCM still
    reports cruise engaged and holds the throttle. CRZ_CTRL.CRZ_ACTIVE is the radar's own word
    for the same thing and is checked as well, but only when the frame has been seen, so a car
    that does not broadcast it cannot make this read "not driving" forever.
    """
    if self.stock_accel_cmd is None:
      return False
    d = self.frames.get(CRZ_CTRL_ADDR)
    if d is None or len(d) < 8:
      return True
    return bool(d[0] & 0x08)

  @property
  def crz_info_counter(self) -> int | None:
    d = self.frames.get(CRZ_INFO_ADDR)
    return (d[6] & 0x0F) if d is not None and len(d) >= 8 else None  # CRZ_INFO.CTR1

  @property
  def heartbeat_counter(self) -> int | None:
    d = self.frames.get(RADAR_COUNTER_ADDR)
    return (d[7] & 0x0F) if d is not None and len(d) >= 8 else None  # RADAR_361.CTR

  def heartbeat_frames(self) -> dict[int, bytes]:
    return {a: self.frames[a] for a in HEARTBEAT_ADDRS if a in self.frames}


class HybridArbiter:
  """Which ACC master is wanted: True = radar emulation (openpilot), False = stock MRCC."""
  DEBOUNCE_FRAMES = int(round(0.75 / DT_CTRL))            # a wish must hold this long to count
  DISENGAGED_DEBOUNCE_FRAMES = int(round(2.0 / DT_CTRL))  # tolerate a quick re-engage
  DWELL_FRAMES = int(round(3.0 / DT_CTRL))                # minimum time between switches
  TAKEOVER_WATCH_FRAMES = int(round(3.0 / DT_CTRL))       # a disengage this soon after a takeover blames it
  TAKEOVER_MIN_SPEED = 2.0    # m/s: never take over at a stop, stops belong to whoever is driving
  HANDBACK_MIN_SPEED = 8.5    # m/s (~19 mph): MRCC can be SET again here if the restart drops it
  HANDBACK_MAX_DECEL = -0.5   # m/s^2: never hand back in the middle of a braking event

  def __init__(self):
    self.want_emulation = False
    self.pending_frames = 0
    self.frames_since_switch = self.DWELL_FRAMES
    self.takeover_rejected = False  # latched for the drive: the car dropped cruise on a takeover
    self.takeover_watch_frames = 0
    self.engaged_prev = False
    # Set by the caller from MrccResync: MRCC came back from a hand-back and would not start
    # driving again. Until the driver re-engages, openpilot keeps the radar.
    self.mrcc_unavailable = False

  def desired(self, emulating: bool, engaged: bool, experimental: bool, standstill: bool,
              brake_pressed: bool, gas_pressed: bool, v_ego: float, accel: float) -> bool:
    if not engaged:
      # Not using ACC: give the car its own radar back, and with it factory AEB and SCBS.
      return False
    if standstill:
      # Leaving emulation at a stop removes the emulated HOLD command, so it is only done while
      # the driver is holding the car on the brake. A stop is never a reason to take over.
      return emulating and not brake_pressed
    if emulating:
      if experimental or self.mrcc_unavailable:
        return True
      # Standard mode: back to MRCC, unless openpilot is braking or MRCC could not be set again.
      return v_ego < self.HANDBACK_MIN_SPEED or accel < self.HANDBACK_MAX_DECEL
    return ((experimental or self.mrcc_unavailable) and v_ego > self.TAKEOVER_MIN_SPEED and
            not brake_pressed and not gas_pressed and not self.takeover_rejected)

  def note_takeover(self) -> None:
    self.takeover_watch_frames = self.TAKEOVER_WATCH_FRAMES

  def update(self, emulating: bool, engaged: bool, experimental: bool, standstill: bool,
             brake_pressed: bool, gas_pressed: bool, v_ego: float, accel: float) -> bool:
    if self.takeover_watch_frames > 0:
      self.takeover_watch_frames -= 1
      if self.engaged_prev and not engaged and not brake_pressed:
        # The engagement fell away with no brake press right after the car lost its radar: the
        # car does not accept the takeover. Stay on MRCC for the rest of the drive.
        self.takeover_rejected = True
        self.takeover_watch_frames = 0
        carlog.error({"event": "mazdaHybridTakeoverRejected", "vEgo": round(v_ego, 2)})
    self.engaged_prev = engaged

    self.frames_since_switch += 1
    want = self.desired(emulating, engaged, experimental, standstill, brake_pressed, gas_pressed, v_ego, accel)
    if want == self.want_emulation:
      self.pending_frames = 0
      return self.want_emulation

    self.pending_frames += 1
    debounce = self.DEBOUNCE_FRAMES if engaged else self.DISENGAGED_DEBOUNCE_FRAMES
    if self.pending_frames >= debounce and self.frames_since_switch >= self.DWELL_FRAMES:
      carlog.warning({"event": "mazdaHybridWant", "emulation": want, "engaged": engaged,
                      "experimental": experimental, "standstill": standstill, "vEgo": round(v_ego, 2)})
      self.want_emulation = want
      self.pending_frames = 0
      self.frames_since_switch = 0
    return self.want_emulation


class MrccResync:
  """Get MRCC driving again after the stock radar restarts.

  The radar is asleep while openpilot emulates it, so when it is let go it comes back in standby:
  the PCM still reports cruise engaged and the dash still shows a set speed, but the radar commands
  nothing. The car then holds the throttle and never brakes for traffic -- on the dash the cruise
  symbol goes grey, exactly as it does when the driver presses the accelerator.

  The radar says so itself (RadarWitness.mrcc_driving), so the fix is: notice it, press RES (the
  same button the stop-and-go resume uses; on this car RES is its own button, not SET+), and if the
  radar still will not take over, say so, so the caller can take the radar back rather than leave
  the car accelerating with nothing watching the road.

  It watches every silent -> alive edge, not just hand-backs, so a radar that resets on its own
  while MRCC is in charge is caught by the same logic.

  Worst case, with the radar ignoring both presses: 1.0 s to confirm, two presses 2.2 s apart,
  and the latch at 5.4 s after the hand-back; the arbiter's debounce then puts openpilot back on
  the radar about a second later. Normally the first press lands and it is over in 1.3 s.
  """
  CONFIRM_FRAMES = int(round(1.0 / DT_CTRL))   # standby must hold this long before the first nudge
  RETRY_FRAMES = int(round(0.4 / DT_CTRL))     # standby is already established: retry sooner
  PULSE_FRAMES = int(round(0.3 / DT_CTRL))     # length of one RES press
  SETTLE_FRAMES = int(round(1.5 / DT_CTRL))    # give the radar this long to react
  MAX_ATTEMPTS = 2                             # two ignored presses mean the radar is asleep

  def __init__(self):
    self.armed = False
    self.alive_prev = False
    self.seen_alive = False
    self.standby_frames = 0
    self.pulse_frames = 0
    self.settle_frames = 0
    self.attempts = 0
    self.failed = False        # latched: MRCC will not drive until the driver re-engages
    self.press_resume = False

  def note_handback(self) -> None:
    # The radar is coming back from emulation; the silent -> alive edge arms it in any case,
    # but a hand-back is a fresh start for the attempt count.
    self.armed = True
    self.standby_frames = 0
    self.pulse_frames = 0
    self.settle_frames = 0
    self.attempts = 0

  def note_engaged_by_driver(self) -> None:
    # A fresh engagement is the driver's own SET/RES: the radar is in charge again.
    self.armed = False
    self.failed = False
    self.attempts = 0

  def update(self, mrcc_master: bool, engaged: bool, witness: RadarWitness,
             brake_pressed: bool, gas_pressed: bool, v_ego: float) -> None:
    self.press_resume = False

    # Any radar that has just come back from a silence may be in standby, whether openpilot put
    # it to sleep or it reset on its own. The edge is read on every cycle so it is never missed.
    # The first frames of a drive are not a restart: nothing was interrupted.
    restarted = witness.alive and self.seen_alive and not self.alive_prev
    self.seen_alive |= witness.alive
    self.alive_prev = witness.alive

    if not mrcc_master or not engaged:
      self.armed = False
      self.standby_frames = 0
      self.pulse_frames = 0
      self.settle_frames = 0
      return

    if restarted:
      self.armed = True
      self.standby_frames = 0

    if not self.armed or self.failed or not witness.alive or brake_pressed or gas_pressed or \
       v_ego < HybridArbiter.TAKEOVER_MIN_SPEED:
      # The pedals are the driver's: never press a button on top of them.
      self.standby_frames = 0
      return

    if witness.mrcc_driving:
      if self.attempts:
        carlog.warning({"event": "mazdaHybridMrccResumed", "attempts": self.attempts})
      self.armed = False
      self.attempts = 0
      return

    if self.settle_frames > 0:
      self.settle_frames -= 1
      return

    if self.pulse_frames > 0:
      self.pulse_frames -= 1
      self.press_resume = True
      if self.pulse_frames == 0:
        self.settle_frames = self.SETTLE_FRAMES
      return

    self.standby_frames += 1
    if self.standby_frames >= (self.CONFIRM_FRAMES if not self.attempts else self.RETRY_FRAMES):
      self.standby_frames = 0
      if self.attempts >= self.MAX_ATTEMPTS:
        self.failed = True
        carlog.error({"event": "mazdaHybridMrccStuck", "attempts": self.attempts,
                      "vEgo": round(v_ego, 2)})
      else:
        self.attempts += 1
        self.pulse_frames = self.PULSE_FRAMES
        carlog.warning({"event": "mazdaHybridMrccResume", "attempt": self.attempts,
                        "crzActive": witness.stock_cruise_active,
                        "stockAccelCmd": witness.stock_accel_cmd, "vEgo": round(v_ego, 2)})


class RadarMaster(StrEnum):
  MRCC = "mrcc"            # stock radar is the ACC master; we transmit nothing
  SILENCING = "silencing"  # programming session requested; stock radar still in control
  EMULATING = "emulating"  # radar silent and ours; replacement frames + tester present
  RESTORING = "restoring"  # default session requested; frames held until the radar speaks


class HybridRadarManager:
  SILENCE_TIMEOUT_FRAMES = int(round(3.0 / DT_CTRL))
  RESTORE_TIMEOUT_FRAMES = int(round(10.0 / DT_CTRL))
  RESTORE_CONFIRM_FRAMES = 5  # stock CRZ_INFO frames in one unbroken run (~100 ms) before letting go
  # After a takeover is abandoned mid-request, keep watching this long before calling the radar
  # restored: a programming request that lands late silences it after we stopped looking.
  UNDO_SETTLE_FRAMES = int(round(1.0 / DT_CTRL))

  def __init__(self):
    self.state = RadarMaster.MRCC
    self.state_frames = 0
    self.request_sent = False
    self.restore_from_emulation = False
    self.silence_failed = False   # latched for the drive: stay on MRCC
    self.restore_failed = False   # latched for the drive: keep emulating
    self.diagnostic: CanData | None = None
    self.just_took_over = False
    self.just_handed_back = False
    self.transmitting = False

  @property
  def emulating(self) -> bool:
    return self.state in (RadarMaster.EMULATING, RadarMaster.RESTORING) and self.restore_from_emulation

  def _to(self, state: RadarMaster, reason: str) -> None:
    carlog.warning({"event": "mazdaHybridRadar", "from": str(self.state), "to": str(state), "reason": reason})
    if state in (RadarMaster.SILENCING, RadarMaster.RESTORING):
      self.request_sent = False
    if state == RadarMaster.EMULATING:
      self.restore_from_emulation = True
    if state == RadarMaster.MRCC:
      self.restore_from_emulation = False
    self.state = state
    self.state_frames = 0

  def _uds_due(self) -> bool:
    # Paced from state entry, so the first request goes out the cycle a transition starts
    # instead of waiting up to 0.5 s for an absolute frame boundary.
    return self.state_frames % RADAR_UDS_STEP == 0

  def update(self, want_emulation: bool, witness: RadarWitness, bus_healthy: bool, fsc_settled: bool) -> RadarMaster:
    self.diagnostic = None
    self.just_took_over = False
    self.just_handed_back = False

    if self.state == RadarMaster.MRCC:
      # Only take over from a radar we can actually see, on a healthy bus, past the FSC's
      # cold-boot radar check.
      if want_emulation and bus_healthy and fsc_settled and witness.alive and not self.silence_failed:
        self._to(RadarMaster.SILENCING, "takeover requested")

    if self.state == RadarMaster.SILENCING:
      if self.request_sent and witness.silent and bus_healthy:
        # Checked first: once the radar has gone quiet the PCM has no ACC master, so take
        # over now even if the wish is being withdrawn -- a hand-back from EMULATING holds
        # frames until the radar returns, an undo from here would leave a gap.
        self._to(RadarMaster.EMULATING, "stock radar silent")
        self.just_took_over = True
      elif not want_emulation or not bus_healthy:
        # A request may already be in flight: undo it rather than abandon it.
        self._to(RadarMaster.RESTORING if self.request_sent else RadarMaster.MRCC, "takeover withdrawn")
      elif self.state_frames >= self.SILENCE_TIMEOUT_FRAMES:
        self.silence_failed = True
        carlog.error({"event": "mazdaHybridSilenceFailed", "reason": "radar did not go silent"})
        self._to(RadarMaster.RESTORING if self.request_sent else RadarMaster.MRCC, "silence timed out")
      elif self._uds_due():
        self.diagnostic = radar_session_msg(uds.SESSION_TYPE.PROGRAMMING)
        self.request_sent = True

    elif self.state == RadarMaster.EMULATING:
      if witness.alive:
        # Stock CRZ_INFO while we hold the session: the radar left it on its own (reset, lost
        # session). Two masters must never command the PCM: our frames stop this cycle (see
        # transmitting below) and the radar is confirmed back as master. No more takeovers:
        # do not fight a radar that will not stay silent.
        self.silence_failed = True
        carlog.error({"event": "mazdaHybridRadarReturned", "reason": "stock radar spoke while emulating"})
        self._to(RadarMaster.RESTORING, "stock radar returned")
      elif not want_emulation and not self.restore_failed:
        self._to(RadarMaster.RESTORING, "hand back requested")
      elif self._uds_due():
        self.diagnostic = make_tester_present_msg(RADAR_UDS_ADDR, RADAR_BUS, suppress_response=True)

    # Evaluated in the same cycle a hand-back or undo starts, so the default-session request
    # goes out immediately.
    if self.state == RadarMaster.RESTORING:
      settle = 0 if self.restore_from_emulation else self.UNDO_SETTLE_FRAMES
      if witness.alive and witness.run_frames >= self.RESTORE_CONFIRM_FRAMES and self.state_frames >= settle:
        handed_back = self.restore_from_emulation
        self._to(RadarMaster.MRCC, "stock radar restored")
        self.just_handed_back = handed_back
      elif self.state_frames >= self.RESTORE_TIMEOUT_FRAMES:
        self.restore_failed = True
        carlog.error({"event": "mazdaHybridRestoreFailed", "reason": "stock radar did not come back"})
        # If we never owned the radar there is nothing to keep emulating.
        self._to(RadarMaster.EMULATING if self.restore_from_emulation else RadarMaster.MRCC, "restore timed out")
      elif self._uds_due() and not witness.alive:
        self.diagnostic = radar_session_msg(uds.SESSION_TYPE.DEFAULT)
        self.request_sent = True

    # Replacement frames only while the radar is genuinely ours and quiet. In RESTORING they
    # are held until the first stock frame so the PCM is never left without an ACC master,
    # and dropped the same cycle the stock radar speaks.
    self.transmitting = ((self.state == RadarMaster.EMULATING) or
                         (self.state == RadarMaster.RESTORING and self.restore_from_emulation)) and not witness.alive
    self.state_frames += 1
    return self.state
