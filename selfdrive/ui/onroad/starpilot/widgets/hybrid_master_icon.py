import math
import time

import pyray as rl
from opendbc.car.mazda.hybrid import HYBRID_MASTER_EMULATING, HYBRID_MASTER_PARAM, HYBRID_MASTER_PENDING
from opendbc.car.mazda.values import MazdaSafetyFlags
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget

MAZDA_HYBRID_FLAGS = MazdaSafetyFlags.RADAR_EMULATION | MazdaSafetyFlags.HYBRID_LONG
READ_INTERVAL_S = 0.1
PULSE_HZ = 1.25


def mazda_hybrid_active(CP) -> bool:
  if CP is None or getattr(CP, "brand", "") != "mazda":
    return False
  flags = int(getattr(CP, "flags", 0))
  return flags & MAZDA_HYBRID_FLAGS == MAZDA_HYBRID_FLAGS


def decode_hybrid_master(status: int) -> tuple[bool, bool]:
  """(openpilot's emulated radar is the ACC master, the driver's choice is still to come)."""
  return bool(status & HYBRID_MASTER_EMULATING), bool(status & HYBRID_MASTER_PENDING)


class HybridMasterIcon(Widget):
  """Mazda hybrid longitudinal: who has gas and brake right now. A car sending radar waves while the stock
  radar (MRCC) drives, an eye with a "c" while openpilot drives on comma's vision with the emulated radar.
  Pulses while a switch the driver asked for is still to come (deferred below 19 mph, at a stop, while
  braking, or waiting for the radar)."""
  SIZE = 144
  ICON_SIZE = 104

  def __init__(self):
    super().__init__()
    self._emulating = False
    self._pending = False
    self._last_read = 0.0
    self._bg = rl.Color(0, 0, 0, 166)
    self._txt_mrcc = gui_app.texture("icons/mazda_mrcc_radar.png", self.ICON_SIZE, self.ICON_SIZE)
    self._txt_vision = gui_app.texture("icons/mazda_comma_vision.png", self.ICON_SIZE, self.ICON_SIZE)
    self.set_visible(lambda: ui_state.started and mazda_hybrid_active(ui_state.CP))

  def place_left_of(self, anchor: rl.Rectangle, spacing: float = 15) -> None:
    """Sit just left of `anchor` (the steering wheel button), centred on it vertically."""
    self.set_rect(rl.Rectangle(anchor.x - spacing - self.SIZE, anchor.y + (anchor.height - self.SIZE) / 2,
                               self.SIZE, self.SIZE))

  def _update_state(self) -> None:
    if not self.is_visible:
      return
    now = time.monotonic()
    if now - self._last_read >= READ_INTERVAL_S:
      self._last_read = now
      self._emulating, self._pending = decode_hybrid_master(ui_state.params_memory.get_int(HYBRID_MASTER_PARAM, default=0))

  def _render(self, rect: rl.Rectangle) -> None:
    center = rl.Vector2(rect.x + rect.width / 2, rect.y + rect.height / 2)
    rl.draw_circle_v(center, rect.width / 2, self._bg)

    alpha = 255
    if self._pending:
      alpha = int(110 + 145 * (0.5 + 0.5 * math.cos(2 * math.pi * PULSE_HZ * time.monotonic())))
    texture = self._txt_vision if self._emulating else self._txt_mrcc
    rl.draw_texture_ex(texture, rl.Vector2(center.x - texture.width / 2, center.y - texture.height / 2), 0.0, 1.0,
                       rl.Color(255, 255, 255, alpha))
