"""The Steering Smoothing stepper: one of the 0.01-0.10 s options, saved on each step, the shell adapter's interface."""
from unittest.mock import patch

from openpilot.starpilot.ui.raylib import rl
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.settings import mazda_lat_smooth
from openpilot.selfdrive.ui.layouts.settings.initial_cruise_speed import InitialCruiseSpeedControl
from openpilot.starpilot.car.mazda.model_smoothing import PARAM


class FakeParams:
  def __init__(self):
    self.store = {}

  def get(self, key, return_default=False, **kw):
    return self.store.get(key, 0.05 if return_default else None)

  def put(self, key, value, block=False):
    self.store[key] = value


def test_steps_and_saves(monkeypatch):
  params = FakeParams()
  monkeypatch.setattr(mazda_lat_smooth.ui_state, "params", params, raising=False)
  with patch.object(gui_app, 'font', return_value=rl.Font()):   # as test_native_settings_pages builds its stepper
    control = mazda_lat_smooth.MazdaLatSmoothControl()
  assert isinstance(control, InitialCruiseSpeedControl)   # what starpilot_settings_adapter_large.py recognizes
  choices, index = control.display_options()
  assert choices[index] == "0.05 s" and choices[0] == "0.01 s" and choices[-1] == "0.10 s"
  assert control.adjust_speed(-1) and params.store[PARAM] == 0.04
  assert control.select_speed(0) and params.store[PARAM] == 0.01
  assert not control.adjust_speed(-1)                       # already at the bottom
  assert control.select_speed(len(choices) - 1) and params.store[PARAM] == 0.10
  assert not control.adjust_speed(1)
