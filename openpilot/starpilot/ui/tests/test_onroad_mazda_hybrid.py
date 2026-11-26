"""The Mazda hybrid master icon in the StarPilot shell: where it sits and what it shows."""
import copy
from types import SimpleNamespace

from opendbc.car.mazda.hybrid import HYBRID_MASTER_ACTIVE, HYBRID_MASTER_EMULATING, HYBRID_MASTER_PENDING
from opendbc.car.mazda.values import MazdaSafetyFlags
from openpilot.starpilot.ui import onroad_mazda_hybrid as icon_module
from openpilot.starpilot.ui.onroad_customization import default_document, placement
from openpilot.starpilot.ui.onroad_state import AlertSize
from openpilot.starpilot.ui.raylib import rl

HYBRID_CP = SimpleNamespace(brand="mazda", flags=int(MazdaSafetyFlags.RADAR_EMULATION | MazdaSafetyFlags.HYBRID_LONG))
CONTENT = rl.Rectangle(0, 0, 2160, 1080)


def wheel_rect(document):
  # where SteeringWheelWidget.render draws the wheel at its default position and size
  return CONTENT.x + CONTENT.width - 146 - 96, CONTENT.y + 45


def test_decode_and_activity():
  assert icon_module.decode_hybrid_master(0) == (False, False, False)
  assert icon_module.decode_hybrid_master(HYBRID_MASTER_EMULATING | HYBRID_MASTER_ACTIVE) == (True, False, True)
  assert icon_module.decode_hybrid_master(HYBRID_MASTER_PENDING) == (False, True, False)
  assert icon_module.mazda_hybrid_active(HYBRID_CP)
  assert not icon_module.mazda_hybrid_active(SimpleNamespace(brand="mazda", flags=int(MazdaSafetyFlags.RADAR_EMULATION)))
  assert not icon_module.mazda_hybrid_active(SimpleNamespace(brand="toyota", flags=HYBRID_CP.flags))
  assert not icon_module.mazda_hybrid_active(None)


def test_eye_and_waves():
  assert icon_module.eye_frame(0.0) == 12 and icon_module.eye_frame(-1.0) == 0 and icon_module.eye_frame(1.0) == 24
  alphas = icon_module.wave_alphas(0.22 * icon_module.WAVE_PERIOD_S)
  assert alphas[0] == 1.0 and alphas[2] == icon_module.WAVE_REST_ALPHA


def test_sits_left_of_the_wheel():
  document = default_document()
  anchor = icon_module.shell_anchor(CONTENT, document)
  assert (anchor.x, anchor.y) == wheel_rect(document)


def test_takes_the_wheels_place_when_the_wheel_is_hidden():
  document = copy.deepcopy(default_document())
  placement(document, "large", "steering_wheel")["enabled"] = False
  anchor = icon_module.shell_anchor(CONTENT, document)
  x, y = wheel_rect(document)
  # render_left_of puts the icon SPACING + SIZE left of its anchor: right where the wheel would be
  assert anchor.x - icon_module.SPACING - icon_module.SIZE == x and anchor.y == y


def test_render_shell_draws_only_without_an_alert(monkeypatch):
  drawn = []
  icon = icon_module.MazdaHybridIcon()
  monkeypatch.setattr(icon, "render_left_of", lambda anchor, CP, started: drawn.append((anchor.x, anchor.y, started)))
  document = default_document()
  for size in (AlertSize.SMALL, AlertSize.MID, AlertSize.FULL):
    icon.render_shell(CONTENT, SimpleNamespace(alert=SimpleNamespace(size=size), customization=document), HYBRID_CP, True)
  assert drawn == []
  icon.render_shell(CONTENT, SimpleNamespace(alert=SimpleNamespace(size=AlertSize.NONE), customization=document), HYBRID_CP, True)
  assert drawn == [(*wheel_rect(document), True)]
