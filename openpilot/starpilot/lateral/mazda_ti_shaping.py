"""Speed shaping for a Mazda GEN1 with a Torque Interceptor (CarParams flag MazdaSafetyFlags.TORQUE_INTERCEPTOR).

The TI adds its request to the EPS torque-sensor signal, so the full power-steering assist answers it, and the plant
is not the EPS-limited one the torque controller was written for. Two things follow, both fitted on the CX-9 2023
routes of the StarPilot 6 fork (tools/mazda/HANDOFF.md):

1. The plant itself changes with speed. Lateral accel per unit of torque is about 2.6 at 5-10 m/s, 2.05 at
   10-13.5, 1.5 at 13.5-16.5 and 1.75-1.9 from 16.5 to 32 m/s, where torqued learns its one global value; friction
   is about 0.30 of full torque below 13.5 m/s against 0.18 on the highway. zoompilot's learned per-speed table for
   the 2022 CX-5 EPS has the same shape (a peak near 9 m/s, a dip near 16 m/s). So the live or tune values are taken
   as the highway base and shaped by speed, which converts the P and I corrections and the feedforward alike. The
   shapes sit about a third of the way back from the fit toward 1.0: the fit is noise-limited below 10 m/s.

2. The proportional response at low speed. StarPilot 6's controller multiplied the error by its low-speed factor
   (1 + LSF(v)^2 / v^2, LSF from SP6_LOW_SPEED_*), 4.5x at 7.5 m/s and 25x at 3 m/s, and a small error near the
   apex of a low-speed turn swung the request from full into the turn toward full out of it. The fork took most of
   that back out below 30 mph (SP6_MAZDA_TI_ERROR_SCALE_*), and a week of drives settled on an effective gain that
   is a nearly flat 1.2-1.5x of the highway gain from 5 to 13.5 m/s and exactly the highway gain from 30 mph up.
   openpilot 0.11's controller raises the proportional gain itself instead (latcontrol_torque KP_INTERP: 14x at
   5 m/s, 7x at 7.5, 4x at 10, 2x at 20). mazda_ti_error_scale() converts one schedule into the other, so the TI
   gets the effective gain the fork tested on either controller. It has not been driven on this controller yet:
   the first drives on StarPilot 7 should compare against the fork's routes (tools/mazda/ti_steer_report.py).
"""
import numpy as np

# the plant (1.)
MAZDA_TI_LAT_ACCEL_FACTOR_SCALE_BP = [8.75, 11.75, 15.0, 18.75]  # m/s
MAZDA_TI_LAT_ACCEL_FACTOR_SCALE_V = [1.3, 1.1, 0.9, 1.0]
MAZDA_TI_FRICTION_SCALE_BP = [5.0, 13.5, 21.0]  # m/s
MAZDA_TI_FRICTION_SCALE_V = [1.6, 1.5, 1.0]

# the tested effective proportional gain (2.), expressed against StarPilot 6's controller
SP6_LOW_SPEED_X = [0.0, 10.0, 20.0, 30.0]
SP6_LOW_SPEED_Y = [12.0, 10.5, 8.0, 5.0]
SP6_MAZDA_TI_ERROR_SCALE_BP = [5.0, 7.5, 10.0, 13.5]  # m/s
SP6_MAZDA_TI_ERROR_SCALE_V = [0.2, 0.4, 0.65, 1.0]
SP6_MIN_SPEED = 1.0  # the low-speed factor is on the curvature; below this the conversion is not defined


def get_mazda_ti_lat_accel_factor_scale(v_ego: float) -> float:
  return float(np.interp(v_ego, MAZDA_TI_LAT_ACCEL_FACTOR_SCALE_BP, MAZDA_TI_LAT_ACCEL_FACTOR_SCALE_V))


def get_mazda_ti_friction_scale(v_ego: float) -> float:
  return float(np.interp(v_ego, MAZDA_TI_FRICTION_SCALE_BP, MAZDA_TI_FRICTION_SCALE_V))


def sp6_effective_gain(v_ego: float) -> float:
  """The fork's tested proportional gain per unit of lateral-accel error, relative to the highway gain."""
  v = max(float(v_ego), SP6_MIN_SPEED)
  low_speed_factor = float(np.interp(v, SP6_LOW_SPEED_X, SP6_LOW_SPEED_Y)) ** 2
  return (1.0 + low_speed_factor / v ** 2) * float(np.interp(v, SP6_MAZDA_TI_ERROR_SCALE_BP, SP6_MAZDA_TI_ERROR_SCALE_V))


def mazda_ti_error_scale(v_ego: float, kp_speeds, kp_values) -> float:
  """What to multiply the lateral-accel error by so a controller with the given proportional-gain schedule
  (kp_values over kp_speeds, the last entry the highway gain) applies the fork's tested effective gain."""
  v = max(float(v_ego), SP6_MIN_SPEED)
  controller_gain = float(np.interp(v, kp_speeds, kp_values)) / float(kp_values[-1])
  return min(sp6_effective_gain(v) / controller_gain, 1.0)
