"""Mazda with a Torque Interceptor: lateral smoothing and its matching lookahead, set by the driver.

StarPilot 6 (openpilot 0.10) smoothed the model's desired curvature with a 0.1 s time constant in modeld and added
the same 0.1 s to the lateral delay that modeld feeds the model and controlsd feeds the torque controller. openpilot
0.11 dropped both (LAT_SMOOTH_SECONDS = 0) for every car. On the CX-9 with the interceptor, with the delay, gains and
tune otherwise back at StarPilot 6's, the first drive on 4c026b03 (route 0000000e) still rocked 2-3x more than the
old branch at 45-67 mph in the 0.8-3 Hz band: the model's request (0.039 vs 0.017 m/s^2 rms), the controller output
(0.071 vs 0.022) and the car's response (0.069 vs 0.022) alike. modeld and controlsd add these seconds through the
same hook as the Kona's Stable filter (starpilot/car/hyundai/model_smoothing.py), read once when they start.

The amount is the MazdaLatSmoothSeconds setting (Toggles, "Steering Smoothing"): 0.01 to 0.10 s in 0.01 s steps,
0.05 s by default (half of StarPilot 6's; the owner's choice once lagd had learned the car's delay). More smooths the
request and the lookahead grows with it; less answers sooner.
"""
from opendbc.car.mazda.values import MazdaSafetyFlags

PARAM = "MazdaLatSmoothSeconds"
MIN_SECONDS = 0.01
MAX_SECONDS = 0.10
STEP_SECONDS = 0.01
DEFAULT_SECONDS = 0.05
SP6_LAT_SMOOTH_SECONDS = 0.1  # what StarPilot 6 ran for every car


def options() -> list[float]:
  count = round((MAX_SECONDS - MIN_SECONDS) / STEP_SECONDS) + 1
  return [round(MIN_SECONDS + i * STEP_SECONDS, 2) for i in range(count)]


def clamp(value) -> float:
  """A saved value as one of the options; anything unreadable is the default."""
  try:
    value = float(value)
  except (TypeError, ValueError):
    return DEFAULT_SECONDS
  if value != value:  # NaN
    return DEFAULT_SECONDS
  return round(min(MAX_SECONDS, max(MIN_SECONDS, round(value / STEP_SECONDS) * STEP_SECONDS)), 2)


def saved_seconds(params) -> float:
  try:
    return clamp(params.get(PARAM, return_default=True))
  except Exception:
    return DEFAULT_SECONDS


def applies(cp) -> bool:
  return (getattr(cp, 'brand', '') == 'mazda' and bool(int(getattr(cp, 'flags', 0)) & MazdaSafetyFlags.TORQUE_INTERCEPTOR) and
          not cp.passive and not cp.dashcamOnly and not cp.notCar)


def seconds(cp, params=None) -> float:
  if not applies(cp):
    return 0.
  if params is None:
    from openpilot.common.params import Params
    params = Params()
  return saved_seconds(params)
