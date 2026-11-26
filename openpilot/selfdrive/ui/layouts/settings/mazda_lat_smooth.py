"""The Mazda Torque Interceptor's steering smoothing setting (starpilot/car/mazda/model_smoothing.py): a - / + stepper.

It is an InitialCruiseSpeedControl, so the StarPilot shell's settings adapter (starpilot_settings_adapter_large.py) shows
and edits it the same way: display_options() for the choices, select_speed() / adjust_speed() for a change.
"""
from openpilot.starpilot.ui.raylib import rl

from openpilot.selfdrive.ui.layouts.settings.initial_cruise_speed import (InitialCruiseSpeedControl, STEP_BUTTON_WIDTH,
                                                                         VALUE_FONT_SIZE)
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.starpilot.car.mazda.model_smoothing import PARAM, options, saved_seconds
from openpilot.system.ui.lib.application import FontWeight, TextAlignment
from openpilot.system.ui.widgets.label import gui_label
from openpilot.system.ui.widgets.list_view import BUTTON_HEIGHT


def label(value: float) -> str:
  return f"{value:.2f} s"


class MazdaLatSmoothControl(InitialCruiseSpeedControl):
  def __init__(self):
    super().__init__()
    self.refresh()

  def refresh(self):
    self._options = options()
    self._selected_idx = self._options.index(saved_seconds(ui_state.params))

  def display_options(self) -> tuple[tuple[str, ...], int]:
    self.refresh()
    return tuple(label(v) for v in self._options), self._selected_idx

  def select_speed(self, index: int) -> bool:
    self.refresh()
    if not self.enabled or type(index) is not int or not 0 <= index < len(self._options):
      return False
    self._selected_idx = index
    ui_state.params.put(PARAM, self._options[index], block=True)
    return True

  def _render(self, rect):
    y = rect.y + (rect.height - BUTTON_HEIGHT) / 2
    self._decrease.render(rl.Rectangle(rect.x, y, STEP_BUTTON_WIDTH, BUTTON_HEIGHT))
    self._increase.render(rl.Rectangle(rect.x + rect.width - STEP_BUTTON_WIDTH, y, STEP_BUTTON_WIDTH, BUTTON_HEIGHT))
    gui_label(rl.Rectangle(rect.x + STEP_BUTTON_WIDTH, y, rect.width - 2 * STEP_BUTTON_WIDTH, BUTTON_HEIGHT),
              label(self._options[self._selected_idx]), VALUE_FONT_SIZE,
              color=rl.WHITE if self.enabled else rl.GRAY, font_weight=FontWeight.MEDIUM, alignment=TextAlignment.CENTER)
