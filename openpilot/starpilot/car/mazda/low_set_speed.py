"""Mazda radar emulation: Lower Minimum Set Speed (the LowerMinSetSpeed toggle).

The car's own cruise (the PCM, and the stock MRCC radar) will not hold a set speed under 19 mph. While openpilot's
emulated radar is the ACC master, openpilot alone drives gas and brake, so it can own the set speed instead: card's
cruise helper then steps it on the wheel's SET+/SET- (accelCruise/decelCruise) from the PCM's speed, down to
V_CRUISE_MIN (8 km/h, about 5 mph), and the planner follows it (selfdrive/car/cruise.py owner_set_speed). The dash keeps
showing the PCM's speed; openpilot's screen shows the real target.

With hybrid longitudinal this holds only while emulation is the master (HybridRadarManager.emulating, which includes the
hand-back to MRCC while it is under way); MRCC keeps the car's 19 mph minimum, and the master returns to MRCC on any
disengage, so the low set speed ends there too. The StarPilot 6 fork had this for plain radar emulation only (it
ignored the toggle with hybrid long, when MRCC and emulation shared the dash set speed).
"""
from opendbc.car.mazda.values import MazdaSafetyFlags

PARAM = "LowerMinSetSpeed"


class LowSetSpeed:
  def __init__(self, CP, params):
    flags = int(CP.flags)
    self.enabled = bool(CP.brand == "mazda" and flags & MazdaSafetyFlags.RADAR_EMULATION and CP.openpilotLongitudinalControl and
                        params.get_bool(PARAM))
    self.hybrid = bool(flags & MazdaSafetyFlags.HYBRID_LONG)

  def owns_set_speed(self, CC) -> bool:
    """CC is the Mazda CarController (its hybrid_radar manager), or None."""
    if not self.enabled:
      return False
    if not self.hybrid:
      return True
    radar = getattr(CC, "hybrid_radar", None)
    return bool(radar is not None and radar.emulating)
