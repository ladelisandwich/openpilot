"""Fail-closed admission using physical vehicle telemetry, never a UI road-mode flag."""

from dataclasses import dataclass
import math
import time
from typing import Any

from openpilot.starpilot.parked_evidence import PANDA_TTL_NS, RESUME_SKEW_NS

FAST_AGE_NS = 250_000_000
SLOW_AGE_NS = 1_000_000_000
DWELL_NS = 1_000_000_000
SERVICES = ('carState', 'selfdriveState', 'carControl', 'deviceState', 'pandaStates')


def sample_clocks() -> tuple[int, int]:
  """Discard a preempted clock read, never widen evidence or resume tolerances."""
  for _ in range(3):
    before = time.monotonic_ns()
    boot = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
    after = time.monotonic_ns()
    if before <= 0 or boot <= 0 or after < before:
      raise ValueError('Device clock moved backwards or is unavailable')
    if after - before <= 1_000_000:
      return after, boot
  raise ValueError('Waiting for a stable device clock')


@dataclass(frozen=True)
class Witness:
  mono_ns: int
  boot_ns: int
  stamps: tuple[int, ...]
  buttons: tuple[tuple[str, bool], ...]
  mode: str = 'onroad'


def read_witness(ui: Any, mono_ns: int, boot_ns: int, *, c4: bool) -> Witness | None:
  """Original message and receipt clocks must both be fresh; Panda uses BOOTTIME.

  This proves the pre-transition state only. Offroad gameplay uses independent
  CAN plus actual process shutdown and physical noOutput evidence below.
  """
  try:
    if (not c4 or mono_ns <= 0 or boot_ns <= 0 or not ui.started or ui.CP is None or ui.CP.notCar or
        ui.projection_read_only or ui.replay_clock is not None or getattr(ui.sm, 'replay_sample', None) is not None):
      return None
    sm = ui.sm
    stamps = []
    for name in SERVICES:
      limit = SLOW_AGE_NS if name == 'deviceState' else PANDA_TTL_NS if name == 'pandaStates' else FAST_AGE_NS
      now = boot_ns if name == 'pandaStates' else mono_ns
      stamp, received = int(sm.logMonoTime[name]), float(sm.recv_time[name])
      if (not sm.valid[name] or not sm.alive[name] or sm.recv_frame[name] <= ui.started_frame or stamp <= 0 or
          not 0 <= now - stamp <= limit or not math.isfinite(received) or
          not 0 <= mono_ns - int(received * 1e9) <= limit):
        return None
      stamps.append(stamp)
    cs, ss, cc, ds, pandas = (sm[name] for name in SERVICES)
    speeds = (cs.vEgo, cs.vEgoRaw, cs.wheelSpeeds.fl, cs.wheelSpeeds.fr, cs.wheelSpeeds.rl, cs.wheelSpeeds.rr)
    if (not cs.canValid or cs.canTimeout or not cs.standstill or str(cs.gearShifter) != 'park' or
        any(not math.isfinite(value) or abs(value) > .01 for value in speeds) or
        cs.gasPressed or cs.cruiseState.enabled or ss.enabled or ss.active or str(ss.state) != 'disabled' or
        cc.enabled or cc.latActive or cc.longActive or not ds.started or
        str(ss.alertSize) != 'none' or str(ss.alertStatus) != 'normal' or str(ss.alertSound) != 'none' or
        not pandas or not any(p.ignitionLine or p.ignitionCan for p in pandas)):
      return None
    for panda in pandas:
      if (str(panda.pandaType) == 'unknown' or str(panda.harnessStatus) == 'notConnected' or panda.controlsAllowed or
          panda.heartbeatLost or panda.safetyRxChecksInvalid or str(panda.faultStatus) != 'none' or len(panda.faults)):
        return None
    return Witness(mono_ns, boot_ns, tuple(stamps), tuple((str(event.type), bool(event.pressed)) for event in cs.buttonEvents))
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError, OSError, RuntimeError):
    return None


class ParkGate:
  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self.since: int | None = None
    self.previous: Witness | None = None
    self.ready = False

  def update(self, witness: Witness | None) -> bool:
    previous = self.previous
    if witness is None:
      self.reset()
      return False
    if previous is not None and (witness.mode != previous.mode or not 0 <= witness.mono_ns - previous.mono_ns <= FAST_AGE_NS or
                                 abs((witness.boot_ns - witness.mono_ns) - (previous.boot_ns - previous.mono_ns)) > RESUME_SKEW_NS or
                                 any(new < old for new, old in zip(witness.stamps, previous.stamps, strict=True))):
      self.reset()
    self.previous = witness
    if self.since is None:
      self.since = witness.mono_ns
    self.ready = witness.mono_ns - self.since >= DWELL_NS
    return self.ready


DRIVING_PROCESSES = frozenset(('card', 'selfdrived', 'controlsd', 'modeld', 'dmonitoringmodeld'))


def read_offroad_witness(ui: Any, mono_ns: int, boot_ns: int, *, native_device: bool, vehicle=None, preparing: bool = False) -> Witness | None:
  """Require actual quiet processes and healthy physical noOutput, not a mode request.

  preparing=True may open only the verification screen, never gameplay. Its
  independent decoder must prove Park before the full witness can be admitted.
  Ignition-off uses the same physical parked premise as the existing native
  parked-action owner. With ignition on, a live independent CAN decoder must
  continue proving Park/zero speed; a remembered pre-transition value is invalid.
  """
  try:
    if (not native_device or ui.started or ui.projection_read_only or ui.replay_clock is not None or
        getattr(ui.sm, 'replay_sample', None) is not None or not ui.params.get_bool('IsOffroad')):
      return None
    sm = ui.sm
    stamps = []
    for name in ('deviceState', 'pandaStates', 'managerState'):
      ttl = PANDA_TTL_NS if name == 'pandaStates' else SLOW_AGE_NS
      stamp, received = int(sm.logMonoTime[name]), float(sm.recv_time[name])
      now = boot_ns if name == 'pandaStates' else mono_ns
      if (not sm.valid[name] or not sm.alive[name] or stamp <= 0 or not 0 <= now - stamp <= ttl or
          not math.isfinite(received) or not 0 <= mono_ns - int(received * 1e9) <= ttl):
        return None
      stamps.append(stamp)
    if sm['deviceState'].started or str(sm['deviceState'].thermalStatus) != 'ok':
      return None
    processes = tuple(sm['managerState'].processes)
    names = [p.name for p in processes]
    if len(names) != len(set(names)) or not DRIVING_PROCESSES.issubset(names):
      return None
    if any(p.running for p in processes if p.name in DRIVING_PROCESSES):
      return None
    pandas = tuple(sm['pandaStates'])
    if not pandas:
      return None
    for panda in pandas:
      if (str(panda.pandaType) == 'unknown' or str(panda.safetyModel) != 'noOutput' or panda.controlsAllowed or
          panda.heartbeatLost or panda.safetyRxChecksInvalid or str(panda.faultStatus) != 'none' or len(panda.faults)):
        return None
    ignition = any(p.ignitionLine or p.ignitionCan for p in pandas)
    if ignition and not preparing:
      if (any(str(p.harnessStatus) == 'notConnected' for p in pandas) or vehicle is None or
          not vehicle.ready(mono_ns, boot_ns)):
        return None
    return Witness(mono_ns, boot_ns, tuple(stamps), (), 'offroad-check' if ignition and preparing else 'offroad')
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError, OSError, RuntimeError):
    return None
