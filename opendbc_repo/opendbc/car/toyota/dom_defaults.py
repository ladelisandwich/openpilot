"""Vehicle numerical defaults and target shaping retained from Dom4bfdfd0074f.

These functions shape only an already admitted controller; they grant no axes.
"""
import numpy as np
from opendbc.car.toyota.values import CAR, ToyotaFlags

TOYOTA_INTERCEPTOR_COMFORT_TARGET_ACCEL = 2.


def camry_hybrid(cp):
  return cp.carFingerprint == CAR.TOYOTA_CAMRY and bool(cp.flags & ToyotaFlags.HYBRID)


def rav4h_sdsu(cp):
  return cp.carFingerprint == CAR.TOYOTA_RAV4H and cp.openpilotLongitudinalControl and bool(cp.safetyConfigs[0].safetyParam & 0x1000)


def bypass_pid(cp):
  return bool(cp.flags & ToyotaFlags.GAS_INTERCEPTOR or
              cp.carFingerprint == CAR.TOYOTA_CAMRY and not camry_hybrid(cp) or
              cp.carFingerprint == CAR.TOYOTA_HIGHLANDER and cp.flags & (ToyotaFlags.LONG_FILTER | ToyotaFlags.RADAR_CAN_FILTER))


def apply_defaults(cp):
  if not cp.openpilotLongitudinalControl:
    return
  prius = cp.carFingerprint in (CAR.TOYOTA_PRIUS, CAR.TOYOTA_PRIUS_RETROFIT)
  pedal = bool(cp.flags & ToyotaFlags.GAS_INTERCEPTOR)
  if cp.flags & ToyotaFlags.HYBRID and (prius or pedal or camry_hybrid(cp) or rav4h_sdsu(cp)):
    cp.longitudinalActuatorDelay = .05
  if cp.carFingerprint == CAR.TOYOTA_HIGHLANDER and not cp.flags & ToyotaFlags.HYBRID:
    cp.longitudinalActuatorDelay = .4
  if cp.carFingerprint == CAR.TOYOTA_SIENNA:
    cp.longitudinalActuatorDelay = .5
  if pedal:
    cp.longitudinalActuatorDelay = max(cp.longitudinalActuatorDelay, .2)
    cp.stopAccel = -1.5


def limit_interceptor_pcm_accel(pcm_accel_cmd: float, target_accel: float, stopping: bool, v_ego: float) -> float:
  if stopping:
    return pcm_accel_cmd

  # The Toyota long path should not cross the accel sign against a strong planner
  # request on pedal/SDSU cars during ordinary driving.
  if target_accel > 0.15 and pcm_accel_cmd < 0.0:
    positive_floor = float(np.interp(v_ego, [0.0, 5.0, 15.0], [0.12, 0.08, 0.0]))
    pcm_accel_cmd = max(pcm_accel_cmd, positive_floor)
  elif target_accel < -0.15 and pcm_accel_cmd > 0.0:
    negative_floor = float(np.interp(v_ego, [0.0, 5.0, 15.0], [-0.12, -0.08, 0.0]))
    pcm_accel_cmd = min(pcm_accel_cmd, negative_floor)

  if target_accel <= -1.6:
    return pcm_accel_cmd

  if abs(target_accel) > TOYOTA_INTERCEPTOR_COMFORT_TARGET_ACCEL:
    return pcm_accel_cmd

  lower_delta = float(np.interp(v_ego, [0.0, 5.0, 15.0], [0.20, 0.25, 0.35]))
  upper_delta = float(np.interp(v_ego, [0.0, 5.0, 15.0], [0.25, 0.30, 0.40]))
  limited = float(np.clip(pcm_accel_cmd, target_accel - lower_delta, target_accel + upper_delta))

  # Pedal/SDSU cars can feel especially surgey if the Toyota longitudinal controller
  # swings far away from the planner target. Bias comfort-zone requests back toward
  # the planner while still allowing some controller correction.
  target_weight = float(np.interp(v_ego, [0.0, 5.0, 15.0], [0.8, 0.65, 0.5]))
  limited = (target_weight * target_accel) + ((1.0 - target_weight) * limited)

  if target_accel > 0.15:
    limited = max(limited, float(np.interp(v_ego, [0.0, 5.0, 15.0], [0.10, 0.05, 0.0])))
  elif target_accel < -0.15:
    limited = min(limited, float(np.interp(v_ego, [0.0, 5.0, 15.0], [-0.10, -0.05, 0.0])))

  return limited


def limit_interceptor_stopping_accel(pcm_accel_cmd: float, target_accel: float, stopping: bool, v_ego: float, lead_visible: bool) -> float:
  if not stopping or lead_visible or pcm_accel_cmd >= 0.0 or v_ego >= 2.5:
    return pcm_accel_cmd

  # Pedal/SDSU Toyotas can feel abrupt in the last few feet of a no-lead stop
  # because stopping state can hold onto a stale strong negative command even
  # after the planner target has already softened. Keep real lead stops
  # untouched, but let the last 2-3 mph of a no-lead stop unwind toward the
  # current planner target instead of shoving through zero.
  stop_floor = float(np.interp(v_ego, [0.0, 0.2, 0.5, 0.9, 1.5, 2.5], [-0.72, -0.76, -0.82, -0.92, -1.05, -1.20]))
  target_buffer = float(np.interp(v_ego, [0.0, 0.5, 1.5, 2.5], [0.08, 0.10, 0.15, 0.20]))
  planner_floor = float(target_accel) - target_buffer
  return max(pcm_accel_cmd, max(stop_floor, planner_floor))


def stopping_decel_rate(cp):
  from opendbc.car.toyota import pedal_composite, pedal
  if (cp.brand != 'toyota' or not cp.openpilotLongitudinalControl or cp.passive or cp.dashcamOnly or cp.notCar):
    return None
  if pedal.configured(cp):
    return .3
  if (camry_hybrid(cp) or rav4h_sdsu(cp)) and any(module.configured(cp) for module in pedal_composite.parent_modules()):
    return .3
  return None
