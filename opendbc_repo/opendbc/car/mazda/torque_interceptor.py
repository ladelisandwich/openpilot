"""Mazda Torque Interceptor (TI1, GEN1) quirks the car controller works around.

TiCeilingGuard: held at the 800-count ceiling, the TI faults. After 1.84-1.99 s of an unbroken 800 it
reports STATE OFF with VIOL 0x11 (22 of 22 trips, CX-9 routes 00000022-00000035, at 9-77 mph) and stops
steering for ~31 s: the violation clears ~29.5 s later and RUN returns 2 s after that. Nothing else
trips it: a command between 780 and 800, or one held at the ceiling for under 1.84 s, never did. A
single 20 Hz sample dipped to 790 restarted its count (route 00000033 held 790-800 for 3.1 s with two
such dips). So a held ceiling is dipped a little, for a moment, well before 1.84 s.
"""


class TiCeilingGuard:
  def __init__(self, dip: int, hold_frames: int, dip_frames: int):
    self.dip = dip                    # magnitude of a dipped command, and the level above which the count runs
    self.hold_frames = hold_frames    # frames above `dip` before a dip
    self.dip_frames = dip_frames
    self.held = 0                     # consecutive frames above `dip`
    self.dipping = 0                  # frames left in the current dip

  def update(self, torque: int) -> int | None:
    """Fed every frame with the command about to go to the TI (0 while it isn't steering).
    Returns the magnitude the TI command may have this frame, or None for no limit."""
    if self.dipping == 0:
      self.held = self.held + 1 if abs(torque) > self.dip else 0
      if self.held >= self.hold_frames:
        self.dipping = self.dip_frames
    if self.dipping > 0:
      self.dipping -= 1
      self.held = 0
      return self.dip
    return None
