"""Mazda GEN1 wheel and body inputs the card acts on (StarPilot 7 port of the fork's starpilot_card hooks).

Hybrid longitudinal mode switch. With hybrid longitudinal the driver, not an automatic trigger, picks the ACC
master: Experimental Mode on drives with openpilot emulating the radar, off hands the car back to the stock radar
(MRCC). The distance-increase side of the wheel's distance rocker (CRZ_BTNS.DISTANCE_MORE) is that choice, acted on
at release:

  - long press (CRUISE_LONG_PRESS frames, 0.5 s, or more): toggles Experimental Mode (emulation <-> MRCC).
  - short tap while in Experimental Mode: back to MRCC; in MRCC it does nothing here.

The mode lives in the ExperimentalMode param, where the car controller reads it (opendbc carcontroller
_hybrid_experimental), and selfdrived publishes the same choice as the planner's mode (planner_experimental):
emulation always plans in Experimental Mode, as StarPilot 6's forced CEStatus did, and Conditional Mode's automatic
triggers never move either one. The distance-decrease side (openpilot's gapAdjustCruise) keeps its StarPilot functions.
While MRCC drives, the stock radar sees either side of the rocker itself and steps its own following distance.

Horn tap bookmarks (the MazdaHornBookmark toggle): one bookmark per press, however long the horn is held. selfdrived
counts them itself from starpilotCarState.hornPressed (HornBookmarks), next to the userBookmark messages: the UI
already publishes userBookmark, and msgq hands a service to its newest publisher, so a second publisher in card
would raise on its next send and take card (the TI and emulated-radar frames) down with it.
"""
from opendbc.car.mazda.values import MazdaSafetyFlags
from openpilot.selfdrive.car.cruise import CRUISE_LONG_PRESS


def mazda_hybrid_buttons_active(CP) -> bool:
  flags = int(getattr(CP, "flags", 0))
  return getattr(CP, "brand", None) == "mazda" and bool(flags & MazdaSafetyFlags.HYBRID_LONG) and \
    bool(flags & MazdaSafetyFlags.RADAR_EMULATION)


def planner_experimental(conditional_experimental: bool, hybrid_choice: bool) -> bool:
  """selfdrived's experimentalMode with hybrid longitudinal: the driver's choice, which is also the car controller's
  ACC master (emulation = True). Conditional Mode would otherwise decide on its own (its default, Automatic, does
  whenever ConditionalModeConfig is absent), and the planner ran chill under emulation."""
  del conditional_experimental  # the conditional result never overrides the master choice, in either direction
  return bool(hybrid_choice)


def display_mode(CP, configured, effective, stock_choice):
  """The onroad screen's (configured, effective) Conditional Mode pair. With hybrid longitudinal the driver's ACC-master
  choice is the mode: the screen reads it as the stock choice with no conditional decision, so the edge colour and the
  "ACC - Experimental/Chill" text follow selfdriveState.experimentalMode (planner_experimental) like the planner does."""
  if CP is not None and mazda_hybrid_buttons_active(CP):
    return stock_choice, None
  return configured, effective


class HornBookmarks:
  """Rising edges of the horn, for selfdrived's bookmark count (the MazdaHornBookmark toggle)."""
  def __init__(self, enabled: bool):
    self.enabled = enabled
    self.previously = False

  def update(self, horn: bool) -> int:
    horn = bool(horn) and self.enabled
    pressed = horn and not self.previously
    self.previously = horn
    return int(pressed)


class MazdaCardHooks:
  def __init__(self, CP, params, long_press_frames: int = CRUISE_LONG_PRESS):
    self.params = params
    self.hybrid = mazda_hybrid_buttons_active(CP)
    self.long_press_frames = long_press_frames
    self.distance_increase_counter = 0
    self.distance_increase_previously = False

  def update(self, parsed, can_valid: bool = True) -> None:
    """parsed is the opendbc Mazda CarState (its distance_increase_button), once per card cycle. While CAN is
    invalid the button value may be stale: the press is dropped, so a recovery never reads as a long-press release."""
    if not self.hybrid:
      return
    if not can_valid:
      self.distance_increase_counter = 0
      self.distance_increase_previously = False
      return
    self._update_hybrid_mode_button(bool(getattr(parsed, "distance_increase_button", 0)))

  def _update_hybrid_mode_button(self, pressed: bool) -> None:
    if pressed:
      self.distance_increase_counter += 1
    released = self.distance_increase_previously and not pressed
    self.distance_increase_previously = pressed
    if not released:
      return
    held, self.distance_increase_counter = self.distance_increase_counter, 0
    experimental = self.params.get_bool("ExperimentalMode")
    if held >= self.long_press_frames:
      self.params.put_bool("ExperimentalMode", not experimental)
    elif experimental:
      self.params.put_bool("ExperimentalMode", False)
