"""Display-only wheel colors from caller-qualified vehicle observations."""

import math
from typing import Any


BRAKE_RGB = (255, 0, 0)
ACCEL_RGB = (22, 127, 64)


def observe_wheel_acceleration(car_state: Any) -> float | None:
  """Inputs must already satisfy transport freshness and current-drive checks."""
  if (car_state is None or getattr(car_state, "canValid", None) is not True or
      getattr(car_state, "canTimeout", None) is not False):
    return None
  acceleration = getattr(car_state, "aEgo", None)
  return float(acceleration) if (type(acceleration) is int or type(acceleration) is float) and math.isfinite(acceleration) else None


class WheelFeedbackFade:
  """Measured acceleration tint: full color at ±2 m/s², with hysteresis and fading."""

  def __init__(self) -> None:
    self._target = 0.0
    self._amount = 0.0
    self._last_ns = 0
    self._drive_frame: int | None = None

  def update(self, acceleration: float | None, enabled: bool, now_ns: int, drive_frame: int | None) -> tuple[int, int, int]:
    if now_ns and (not self._last_ns or not enabled or acceleration is None or
                   drive_frame != self._drive_frame or now_ns < self._last_ns):
      self._target = 0.0
      self._amount = 0.0
      self._last_ns = now_ns
    target = 0.0
    if enabled and acceleration is not None:
      threshold = 0.15 if now_ns and self._target * acceleration > 0 else 0.25
      if abs(acceleration) > threshold:
        target = max(-1.0, min(1.0, acceleration / 2.0))
    amount = target
    # Untimed snapshots are static layout samples.
    if now_ns:
      dt = (now_ns - self._last_ns) / 1e9
      self._last_ns, self._drive_frame = now_ns, drive_frame
      self._target = target
      self._amount += (target - self._amount) * (1 - math.exp(-dt / 0.08))
      amount = self._amount
    endpoint = BRAKE_RGB if amount < 0 else ACCEL_RGB
    red, green, blue = [round(255 + (channel - 255) * abs(amount)) for channel in endpoint]
    return red, green, blue
