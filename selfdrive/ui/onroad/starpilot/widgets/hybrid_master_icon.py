import math
import time

import pyray as rl
from opendbc.car.mazda.hybrid import HYBRID_MASTER_ACTIVE, HYBRID_MASTER_EMULATING, HYBRID_MASTER_PARAM, HYBRID_MASTER_PENDING
from opendbc.car.mazda.values import MazdaSafetyFlags
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget

MAZDA_HYBRID_FLAGS = MazdaSafetyFlags.RADAR_EMULATION | MazdaSafetyFlags.HYBRID_LONG
READ_INTERVAL_S = 0.1
PULSE_HZ = 1.25

# The eye's sprite sheet (icons/mazda_comma_vision_frames.png): the iris from looking full left (frame 0) to
# full right (frame 24), 5 frames to a row; frame 12 looks straight ahead.
EYE_FRAMES = 25
EYE_COLS = 5
# A glance left, a glance right, back to the middle: (seconds, where the iris looks, -1 left .. 1 right).
EYE_KEYS = ((0.0, 0.0), (0.6, 0.0), (1.0, -1.0), (1.8, -1.0), (2.6, 1.0), (3.4, 1.0), (3.8, 0.0), (4.2, 0.0))
EYE_PERIOD_S = EYE_KEYS[-1][0]

# The radar waves go out one after another, smallest first, then a short rest; a resting wave stays faint.
WAVE_PERIOD_S = 1.6
WAVE_PEAKS = (0.22, 0.42, 0.62)  # where in the cycle each wave is brightest
WAVE_WIDTH = 0.22                # cycle fraction each side of a peak over which a wave brightens and fades
WAVE_REST_ALPHA = 0.25


def mazda_hybrid_active(CP) -> bool:
  if CP is None or getattr(CP, "brand", "") != "mazda":
    return False
  flags = int(getattr(CP, "flags", 0))
  return flags & MAZDA_HYBRID_FLAGS == MAZDA_HYBRID_FLAGS


def decode_hybrid_master(status: int) -> tuple[bool, bool, bool]:
  """(openpilot's emulated radar is the ACC master, the driver's choice is still to come, the master is driving)."""
  return bool(status & HYBRID_MASTER_EMULATING), bool(status & HYBRID_MASTER_PENDING), bool(status & HYBRID_MASTER_ACTIVE)


def eye_look(t: float) -> float:
  """Where the iris looks at time t: -1 full left .. 1 full right, eased between the EYE_KEYS."""
  p = t % EYE_PERIOD_S
  for (t0, x0), (t1, x1) in zip(EYE_KEYS, EYE_KEYS[1:], strict=False):
    if p <= t1:
      u = (p - t0) / (t1 - t0)
      u = u * u * (3.0 - 2.0 * u)  # smoothstep: start and stop gently
      return x0 + (x1 - x0) * u
  return 0.0


def eye_frame(look: float) -> int:
  return min(EYE_FRAMES - 1, max(0, round((look + 1.0) / 2.0 * (EYE_FRAMES - 1))))


def wave_alphas(t: float) -> tuple[float, ...]:
  """Brightness of the small, middle and big wave at time t, 0..1."""
  p = (t % WAVE_PERIOD_S) / WAVE_PERIOD_S
  return tuple(WAVE_REST_ALPHA + (1.0 - WAVE_REST_ALPHA) * max(0.0, 1.0 - abs(p - peak) / WAVE_WIDTH) for peak in WAVE_PEAKS)


class HybridMasterIcon(Widget):
  """Mazda hybrid longitudinal: who has gas and brake right now. A car sending radar waves while the stock
  radar (MRCC) drives, an eye with a "c" while openpilot drives on comma's vision with the emulated radar.
  Animated only while that master is actually driving (the waves go out one after another while MRCC holds
  the cruise; the eye looks left and right while openpilot commands gas and brake), still otherwise.
  Pulses while a switch the driver asked for is still to come (deferred below 19 mph, at a stop, while
  braking, or waiting for the radar), and after a radar restart until it accepts SET/RES (about 12 s): a solid
  car means RES will work."""
  SIZE = 144
  ICON_SIZE = 104

  def __init__(self):
    super().__init__()
    self._emulating = False
    self._pending = False
    self._active = False
    self._last_read = 0.0
    self._bg = rl.Color(0, 0, 0, 166)
    icon = self.ICON_SIZE
    self._txt_car = gui_app.texture("icons/mazda_mrcc_car.png", icon, icon)
    self._txt_waves = [gui_app.texture(f"icons/mazda_mrcc_wave{i}.png", icon, icon) for i in (1, 2, 3)]
    rows = math.ceil(EYE_FRAMES / EYE_COLS)
    self._txt_eye = gui_app.texture("icons/mazda_comma_vision_frames.png", EYE_COLS * icon, rows * icon)
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
      self._emulating, self._pending, self._active = decode_hybrid_master(
        ui_state.params_memory.get_int(HYBRID_MASTER_PARAM, default=0))

  def _render(self, rect: rl.Rectangle) -> None:
    now = time.monotonic()
    center = rl.Vector2(rect.x + rect.width / 2, rect.y + rect.height / 2)
    rl.draw_circle_v(center, rect.width / 2, self._bg)

    alpha = 1.0
    if self._pending:
      alpha = (110 + 145 * (0.5 + 0.5 * math.cos(2 * math.pi * PULSE_HZ * now))) / 255
    icon = self.ICON_SIZE
    dest = rl.Rectangle(center.x - icon / 2, center.y - icon / 2, icon, icon)

    if self._emulating:
      frame = eye_frame(eye_look(now)) if self._active else eye_frame(0.0)
      src = rl.Rectangle((frame % EYE_COLS) * icon, (frame // EYE_COLS) * icon, icon, icon)
      rl.draw_texture_pro(self._txt_eye, src, dest, rl.Vector2(0, 0), 0.0, self._white(alpha))
    else:
      self._draw_layer(self._txt_car, dest, alpha)
      waves = wave_alphas(now) if self._active else (1.0, 1.0, 1.0)
      for texture, wave_alpha in zip(self._txt_waves, waves, strict=True):
        self._draw_layer(texture, dest, alpha * wave_alpha)

  @staticmethod
  def _white(alpha: float) -> rl.Color:
    return rl.Color(255, 255, 255, int(round(255 * min(1.0, max(0.0, alpha)))))

  def _draw_layer(self, texture: rl.Texture, dest: rl.Rectangle, alpha: float) -> None:
    rl.draw_texture_pro(texture, rl.Rectangle(0, 0, texture.width, texture.height), dest, rl.Vector2(0, 0), 0.0,
                        self._white(alpha))
