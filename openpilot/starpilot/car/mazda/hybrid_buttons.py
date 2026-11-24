"""Mazda GEN1 wheel and body inputs the card acts on (StarPilot 7 port of the fork's starpilot_card hooks).

Hybrid longitudinal mode switch. With hybrid longitudinal the driver, not an automatic trigger, picks the ACC
master: Experimental Mode on drives with openpilot emulating the radar, off hands the car back to the stock radar
(MRCC). The distance-increase side of the wheel's distance rocker (CRZ_BTNS.DISTANCE_MORE) is that choice, acted on
at release:

  - long press (CRUISE_LONG_PRESS frames, 0.5 s, or more): toggles Experimental Mode (emulation <-> MRCC).
  - short tap while in Experimental Mode: back to MRCC; in MRCC it does nothing here.

The mode lives in the ExperimentalMode param, where the car controller reads it (opendbc carcontroller
_hybrid_experimental). The distance-decrease side (openpilot's gapAdjustCruise) keeps its StarPilot functions.
While MRCC drives, the stock radar sees either side of the rocker itself and steps its own following distance.

Horn tap bookmarks (the MazdaHornBookmark toggle): one userBookmark per press, however long the horn is held;
selfdrived counts userBookmark messages as saved events.
"""
from opendbc.car.mazda.values import MazdaSafetyFlags
from openpilot.cereal import messaging
from openpilot.selfdrive.car.cruise import CRUISE_LONG_PRESS


def mazda_hybrid_buttons_active(CP) -> bool:
  flags = int(getattr(CP, "flags", 0))
  return getattr(CP, "brand", None) == "mazda" and bool(flags & MazdaSafetyFlags.HYBRID_LONG) and \
    bool(flags & MazdaSafetyFlags.RADAR_EMULATION)


class MazdaCardHooks:
  def __init__(self, CP, params, long_press_frames: int = CRUISE_LONG_PRESS):
    self.params = params
    self.hybrid = mazda_hybrid_buttons_active(CP)
    self.horn_bookmark = params.get_bool("MazdaHornBookmark")
    self.long_press_frames = long_press_frames
    self.distance_increase_counter = 0
    self.distance_increase_previously = False
    self.horn_pressed_previously = False
    self._bookmark_sock = None

  def update(self, parsed) -> None:
    """parsed is the opendbc Mazda CarState (its distance_increase_button and horn_pressed), once per card cycle."""
    if self.hybrid:
      self._update_hybrid_mode_button(bool(getattr(parsed, "distance_increase_button", 0)))
    if self.horn_bookmark:
      horn = bool(getattr(parsed, "horn_pressed", False))
      if horn and not self.horn_pressed_previously:
        self.bookmark()
      self.horn_pressed_previously = horn

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

  def bookmark(self) -> None:
    if self._bookmark_sock is None:
      self._bookmark_sock = messaging.pub_sock('userBookmark')
    self._bookmark_sock.send(messaging.new_message('userBookmark').to_bytes())
