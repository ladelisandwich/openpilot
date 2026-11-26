"""Mazda hybrid long: the onroad edge and mode text follow the driver's ACC-master choice, not Conditional Mode."""
from dataclasses import replace
from types import SimpleNamespace

from opendbc.car.mazda.values import MazdaSafetyFlags
from openpilot.starpilot.car.mazda.hybrid_buttons import display_mode
from openpilot.starpilot.conditional_mode.policy import ModeChoice
from openpilot.starpilot.ui.onroad import axis_status_color, driving_mode_description
from openpilot.starpilot.ui.preview_shell import reference_onroad

HYBRID = SimpleNamespace(brand="mazda", flags=int(MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR |
                                                  MazdaSafetyFlags.RADAR_EMULATION | MazdaSafetyFlags.HYBRID_LONG))
OTHER = SimpleNamespace(brand="toyota", flags=0)
ORANGE, GREEN = (218, 111, 37), (22, 127, 64)


def rgb(color):
  return color.r, color.g, color.b


def engaged(**kw):
  return replace(reference_onroad('onroad_engaged_no_camera'), visual_preview=None, lateral_active=True,
                 longitudinal_active=True, **kw)


def test_automatic_driving_mode_used_to_hide_emulation():
  # what the first sp7 drives showed: Driving Mode "Automatic" (CEM) with no conditional decision paints chill green
  # although the planner ran in experimental under emulation
  state = engaged(conditional_configured=ModeChoice.CEM, conditional_effective=None, experimental_enabled=True)
  assert rgb(axis_status_color(state)) == GREEN


def test_hybrid_follows_the_master_choice():
  for experimental, color, text in ((True, ORANGE, "ACC - Experimental"), (False, GREEN, "ACC - Chill")):
    configured, effective = display_mode(HYBRID, ModeChoice.CEM, object(), ModeChoice.STOCK)
    assert (configured, effective) == (ModeChoice.STOCK, None)
    state = engaged(conditional_configured=configured, conditional_effective=effective, experimental_enabled=experimental)
    assert rgb(axis_status_color(state)) == color
    assert driving_mode_description(state) == text


def test_other_cars_keep_conditional_mode():
  sentinel = object()
  assert display_mode(OTHER, ModeChoice.CEM, sentinel, ModeChoice.STOCK) == (ModeChoice.CEM, sentinel)
  assert display_mode(None, ModeChoice.CEM, sentinel, ModeChoice.STOCK) == (ModeChoice.CEM, sentinel)
