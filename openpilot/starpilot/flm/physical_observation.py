"""Optional observer wait state, never a physical sample or authority receipt."""
from dataclasses import dataclass


@dataclass(frozen=True)
class PendingPhysical:
  mono_ns: int
  received_ns: int
  context_changed: bool = False

  def fresh(self, now_ns):
    return bool(type(self.context_changed) is bool and all(type(value) is int and value > 0 for value in (self.mono_ns, self.received_ns, now_ns)) and
      self.mono_ns <= self.received_ns <= now_ns and now_ns - self.mono_ns <= 100_000_000)
