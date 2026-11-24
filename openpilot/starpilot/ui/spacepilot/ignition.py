"""Release only this UI session's game request after a continuous physical key-off."""

from dataclasses import dataclass
import math

from openpilot.starpilot.parked_evidence import PANDA_TTL_NS, RESUME_SKEW_NS
from openpilot.starpilot.ui.spacepilot.guard import DWELL_NS, FAST_AGE_NS, SLOW_AGE_NS, read_offroad_witness, sample_clocks


@dataclass(frozen=True)
class IgnitionSample:
  mono: int
  boot: int
  stamps: tuple[int, int]
  topology: tuple[tuple[str, str], ...]
  uptimes: tuple[int, ...]
  pandad_pid: int
  ignition: bool


def sample_ignition(ui, *, native_device: bool) -> IgnitionSample | None:
  try:
    mono, boot = sample_clocks()
    if (not native_device or ui.projection_read_only or
        ui.replay_clock is not None or getattr(ui.sm, 'replay_sample', None) is not None):
      return None
    sm = ui.sm
    stamps = []
    for name, now, ttl in (('pandaStates', boot, PANDA_TTL_NS), ('managerState', mono, SLOW_AGE_NS)):
      stamp, received = int(sm.logMonoTime[name]), float(sm.recv_time[name])
      if (not sm.valid[name] or not sm.alive[name] or sm.recv_frame[name] <= 0 or stamp <= 0 or
          not 0 <= now - stamp <= ttl or not math.isfinite(received) or
          not 0 <= mono - int(received * 1e9) <= ttl):
        return None
      stamps.append(stamp)
    processes = tuple(sm['managerState'].processes)
    names = [p.name for p in processes]
    pandad = [p for p in processes if p.name == 'pandad']
    if len(names) != len(set(names)) or len(pandad) != 1 or not pandad[0].running or pandad[0].pid <= 0:
      return None
    pandas = tuple(sm['pandaStates'])
    if not pandas:
      return None
    for p in pandas:
      if (str(p.pandaType) == 'unknown' or str(p.harnessStatus) == 'notConnected' or p.uptime < 0 or
          p.controlsAllowed or p.heartbeatLost or p.safetyRxChecksInvalid or
          str(p.faultStatus) != 'none' or len(p.faults)):
        return None
    return IgnitionSample(mono, boot, (stamps[0], stamps[1]),
                          tuple((str(p.pandaType), str(p.harnessStatus)) for p in pandas),
                          tuple(int(p.uptime) for p in pandas), int(pandad[0].pid),
                          any(p.ignitionLine or p.ignitionCan for p in pandas))
  except (AttributeError, KeyError, OSError, RuntimeError, TypeError, ValueError, OverflowError):
    return None


class IgnitionReturn:
  """No persisted ownership: restarting UI cannot adopt someone else's Offroad.

  A loss of continuity permanently disarms this return, even if data reconnects.
  Ordinary game exit keeps observing, but holds no reader, widget or wake token.
  """
  def __init__(self, ui, owner, revision: str, *, native_device: bool):
    self.ui, self.owner, self.revision, self.native_device = ui, owner, revision, native_device
    self.previous = sample_ignition(ui, native_device=native_device)
    self.finished = self.previous is None or not self.previous.ignition
    self.off_since: int | None = None
    self.off_stamp: int | None = None

  def _owned(self) -> bool:
    state = self.owner.snapshot()
    return bool(state.available and str(state.mode) == 'offroad' and state.revision == self.revision)

  def _continuous(self, current: IgnitionSample | None) -> bool:
    old = self.previous
    return bool(current is not None and old is not None and
                0 <= current.mono - old.mono <= FAST_AGE_NS and
                abs((current.boot - current.mono) - (old.boot - old.mono)) <= RESUME_SKEW_NS and
                current.topology == old.topology and current.pandad_pid == old.pandad_pid and
                all(new >= previous for new, previous in zip(current.stamps, old.stamps, strict=True)) and
                all(previous <= new <= previous + 1 for new, previous in zip(current.uptimes, old.uptimes, strict=True)))

  def observe(self) -> bool:
    """Return true only after fresh, advancing all-off reports and quiet noOutput."""
    if self.finished:
      return False
    try:
      current = sample_ignition(self.ui, native_device=self.native_device)
      if not self._owned() or not self._continuous(current):
        self.finished = True
        return False
      assert current is not None
      if self.previous is not None and current.ignition != self.previous.ignition and current.stamps[0] <= self.previous.stamps[0]:
        self.finished = True
        return False
      self.previous = current
      if current.ignition:
        self.off_since = self.off_stamp = None
        return False
      # Require actual service shutdown too, without using old Park/CAN data
      # after ignition off. An unknown/faulted offroad witness cannot release.
      if read_offroad_witness(self.ui, current.mono, current.boot, native_device=self.native_device) is None:
        self.finished = True
        return False
      if self.off_since is None:
        self.off_since, self.off_stamp = current.mono, current.stamps[0]
      return self.off_stamp is not None and current.mono - self.off_since >= DWELL_NS and current.stamps[0] > self.off_stamp
    except (AttributeError, KeyError, OSError, RuntimeError, TypeError, ValueError):
      self.finished = True
      return False

  def release(self, game_closed) -> None:
    """Normal owner transaction rechecks physical evidence and closed game."""
    if self.finished:
      return
    def authorized():
      return bool(game_closed() and self.observe())
    try:
      if authorized():
        self.owner.request('auto', expected_revision=self.revision, authorized=authorized, override_allowed=lambda: False)
    finally:
      # One bounded transaction; a busy/rejected owner is never retried blindly.
      self.finished = True
