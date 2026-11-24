"""Clock an equipped Bolt's replacement brake frame from the factory ACC cycle."""

class BoltAccBrakeClock:
  MIN_INTERVAL_NS = 30_000_000
  MAX_AGE_NS = 80_000_000

  def __init__(self):
    self.command = None
    self.command_ns = 0
    self.source_ns = 0
    self.source_counter = None
    self.counter = -1
    self.emit_ns = 0
    self.active_last = False

  def prepare(self, now_ns, brake, enabled, near_stop, full_stop):
    self.command = (brake, enabled, near_stop, full_stop)
    self.command_ns = now_ns

  def release(self, now_ns):
    if not self.active_last or now_ns - self.emit_ns < self.MIN_INTERVAL_NS:
      return None
    # A lost clock cannot authorize braking. Retire the last active command
    # once with the canonical zero release; the native ownership gates still run.
    self.counter = (self.counter + 1) % 4
    self.emit_ns = now_ns
    self.active_last = False
    return (0, self.counter, False, False, False)

  def sample(self, now_ns, source_ns, counter, *, active, main_ready, source_valid):
    if not main_ready:
      self.command = None
      self.active_last = False
      return None
    if (not source_valid or self.command is None or
        not 0 < self.command_ns <= now_ns or now_ns - self.command_ns > self.MAX_AGE_NS or
        not 0 < source_ns <= now_ns or now_ns - source_ns > self.MAX_AGE_NS or not 0 <= counter < 4):
      return self.release(now_ns)
    if source_ns <= self.source_ns or counter == self.source_counter:
      return self.release(now_ns) if now_ns - self.emit_ns > self.MAX_AGE_NS else None
    if self.emit_ns and now_ns - self.emit_ns < self.MIN_INTERVAL_NS:
      return None
    self.source_ns, self.source_counter = source_ns, counter
    if counter == self.counter:
      return None  # The safe release may already have consumed this counter.
    self.counter, self.emit_ns = counter, now_ns
    brake, enabled, near_stop, full_stop = self.command
    self.active_last = bool(active and (brake > 0 or enabled))
    return (brake, counter, enabled, near_stop, full_stop) if active else (0, counter, False, False, False)
