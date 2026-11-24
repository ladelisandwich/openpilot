"""The wipers as a rain report (Mazda "Wipers Mean Rain", the MazdaWiperWeatherMode toggle).

While the front wipers sweep no automated lane change starts (a nudged one still goes), and none starts for
WASHER_BLOCK_S after the washer: the model's lane lines are least trustworthy right then. The inputs come from
starpilotCarState (starpilot/car/mazda/dashboard.py publishes BLINK_INFO.WIPER_LO/HI and FRONT_WASHER).

The StarPilot 6 fork also applied the rain driving offsets (longer gap, gentler acceleration and cornering) while
wiping; StarPilot 7 has no weather driving offsets to attach that to yet.
"""
from opendbc.car.mazda.values import MazdaSafetyFlags

WASHER_BLOCK_S = 3.0
STALE_S = 1.0  # a companion message older than this says nothing about the wipers


def wiper_gate_active(CP, params) -> bool:
  return (getattr(CP, "brand", None) == "mazda" and bool(int(getattr(CP, "flags", 0)) & MazdaSafetyFlags.GEN1)
          and params.get_bool("MazdaWiperWeatherMode"))


class MazdaWiperGate:
  def __init__(self, active: bool):
    self.active = active
    self.washer_until_ns = 0
    self.wipers_active = False

  def update(self, wipers_active: bool, washer_active: bool, now_ns: int) -> None:
    self.wipers_active = bool(wipers_active)
    if washer_active:
      self.washer_until_ns = now_ns + int(WASHER_BLOCK_S * 1e9)

  def blocks_auto_lane_change(self, now_ns: int) -> bool:
    return self.active and (self.wipers_active or now_ns < self.washer_until_ns)

  def feed(self, sm, now_ns: int) -> bool:
    """Read the companion state from a SubMaster that subscribes starpilotCarState and return whether an automated
    lane change is blocked right now."""
    if not self.active:
      return False
    if sm.updated["starpilotCarState"] or sm.recv_frame["starpilotCarState"] > 0:
      fresh = (now_ns - int(sm.logMonoTime["starpilotCarState"])) < STALE_S * 1e9
      state = sm["starpilotCarState"]
      self.update(fresh and bool(state.wipersActive), fresh and bool(state.washerActive), now_ns)
    return self.blocks_auto_lane_change(now_ns)
