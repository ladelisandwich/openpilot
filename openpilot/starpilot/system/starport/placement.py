"""Placement for projection rendering and transport.

The renderer uses SCHED_IDLE on background cores 0-2: ordinary driving work
always preempts it, and it cannot compete on the controls/model cores 3-7.
Transport retains its existing normal scheduling on core 6. Every renderer
thread inherits the idle policy; affinity is maintained for later driver threads
and for CPU power-state changes.
"""

from __future__ import annotations

import os
from collections.abc import Callable
from typing import Any

RENDER_CORE = 6  # existing transport/default placement
BACKGROUND_RENDER_CORES = (0, 1, 2)
PLACEMENT_CHECK_INTERVAL = 1.0  # s
FALLBACK_NICE = 10  # if SCHED_IDLE is unavailable


def _thread_ids() -> list[int]:
  try:
    return [int(tid) for tid in os.listdir("/proc/self/task")]
  except OSError:
    return [0]


def _core_online(core: int) -> bool:
  if core == 0:
    return True
  try:
    with open(f"/sys/devices/system/cpu/cpu{core}/online") as f:
      return f.read().strip() == "1"
  except OSError:
    return False


class RendererPlacement:
  """Keeps every thread on the requested online core or core pool:
  SCHED_IDLE for the renderer, or normal scheduling (idle=False) for
  starportd, whose stream thread otherwise sticks on whichever isolated
  core it first woke on (core 7, waiting ~32 % of the time behind the models)."""

  def __init__(self, core: int | tuple[int, ...] = RENDER_CORE, report: Callable[[dict[str, Any]], None] | None = None,
               thread_ids: Callable[[], list[int]] = _thread_ids, core_online: Callable[[int], bool] = _core_online,
               idle: bool = True):
    self.core = core
    self.cores = (core,) if isinstance(core, int) else core
    if not self.cores or any(cpu < 0 for cpu in self.cores):
      raise ValueError("Expected a nonempty pool of CPU indices")
    self.idle = idle
    self.report = report or (lambda _event: None)
    self.thread_ids = thread_ids
    self.core_online = core_online
    self.next_check = 0.0
    self.pinned = False

  def start(self) -> None:
    """Call first, before any thread exists, so later threads inherit the policy."""
    if not hasattr(os, "sched_setaffinity"):
      self.report({"event": "cpu_placement", "pinned": False, "reason": "no CPU affinity on this platform"})
      self.next_check = float("inf")
      return
    inherited = sorted(os.sched_getaffinity(0))
    policy = "idle" if self.idle else "normal"
    try:
      if self.idle:
        os.sched_setscheduler(0, os.SCHED_IDLE, os.sched_param(0))
    except (AttributeError, OSError):
      policy = f"nice{FALLBACK_NICE}"
      try:
        os.nice(FALLBACK_NICE)
      except OSError:
        policy = "default"
    target = {"core": self.core} if isinstance(self.core, int) else {"cores": list(self.cores)}
    self.report({"event": "cpu_placement", "policy": policy, "inherited_cores": inherited, **target})
    self.maintain(0.0)

  def maintain(self, now: float) -> None:
    if now < self.next_check:
      return
    self.next_check = now + PLACEMENT_CHECK_INTERVAL
    target = {core for core in self.cores if self.core_online(core)}
    if not target:
      if self.pinned:
        self.report({"event": "cpu_placement", "pinned": False, "reason": f"core {self.core} offline"})
      self.pinned = False
      return

    changed = False
    for tid in self.thread_ids():
      try:
        if os.sched_getaffinity(tid) != target:
          os.sched_setaffinity(tid, target)
          changed = True
      except OSError:
        pass  # the thread exited
    if changed or not self.pinned:
      self.report({"event": "cpu_placement", "pinned": True, "cores": sorted(os.sched_getaffinity(0))})
    self.pinned = True
