"""Original Dom preAP dynamics constants and per-drive torquezero owner."""
from opendbc.car.common.conversions import Conversions as CV

PEDAL_DI_MIN = -5.
PEDAL_DI_ZERO = 0.
ACCEL_MAX = 2.5
REGEN_MAX = -1.5
PEDAL_BP = [0., 5., 12., 20., 30., 40.]
PEDAL_MAX_VALUES = [50., 58., 66., 74., 82., 90.]
PEDAL_RAMP_RATE_UP = 5.
PEDAL_RAMP_RATE_DOWN = 2.5
TORQUE_LEVEL_ACC = 0.
TORQUE_LEVEL_DECEL = -30.
ZERO_TORQUE_MIN_SPEED = 10. * CV.MPH_TO_MS


class PedalZeroTorque:
  def __init__(self):
    self.value = PEDAL_DI_ZERO
    self._best_torque = TORQUE_LEVEL_DECEL

  def update(self, torque_level: float, current_pedal_di: float, v_ego: float) -> None:
    if v_ego < ZERO_TORQUE_MIN_SPEED:
      return

    if TORQUE_LEVEL_DECEL < torque_level < TORQUE_LEVEL_ACC and abs(torque_level) < abs(self._best_torque):
      self.value = current_pedal_di
      self._best_torque = torque_level

  def get(self, v_ego: float) -> float:
    if v_ego < 5.0 * CV.MPH_TO_MS:
      return PEDAL_DI_ZERO
    return self.value
