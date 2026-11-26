"""The Mazda Torque Interceptor's shaping of the torque controller (starpilot/lateral/mazda_ti_shaping.py)."""
import math

import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.common.parameterized import parameterized
from openpilot.cereal import log
from opendbc.car.structs import car
from opendbc.car.car_helpers import interfaces
from opendbc.car.mazda.values import CAR as MAZDA, MazdaSafetyFlags
from opendbc.car.toyota.values import CAR as TOYOTA
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.latcontrol_torque import INTERP_SPEEDS, KP_INTERP, LatControlTorque
from openpilot.starpilot.lateral.controller_selection import ControllerMode, default_selection
from openpilot.starpilot.lateral.mazda_ti_shaping import (SP6_KP, SP6_LOW_SPEED_X, SP6_LOW_SPEED_Y, get_mazda_ti_friction_scale,
                                                         get_mazda_ti_lat_accel_factor_scale, get_sp6_mazda_ti_error_scale,
                                                         mazda_ti_error_scale, sp6_effective_gain)


def cx9_params(torque_interceptor=True, friction=None):
  """A CX-9 2021 CarParams the way the interface sets up a TI car (steering down to a stop), with or without the flag."""
  CarInterface = interfaces[MAZDA.MAZDA_CX9_2021]
  CP = CarInterface.get_non_essential_params(MAZDA.MAZDA_CX9_2021)
  CP.flags = int(CP.flags) & ~int(MazdaSafetyFlags.TORQUE_INTERCEPTOR) & 0xFFFFFFFF
  if torque_interceptor:
    CP.flags = int(CP.flags) | int(MazdaSafetyFlags.TORQUE_INTERCEPTOR)
  CP.minSteerSpeed = 0.0
  CP.steerAtStandstill = True
  if friction is not None:
    CP.lateralTuning.torque.friction = friction
  return CP


def run_torque(v_ego, torque_interceptor, curvature, curvature_error, frames=150, friction=None, mode=ControllerMode.STANDARD):
  """Settle on the requested curvature, then hold curvature_error short of it with the integrator from zero."""
  CP = cx9_params(torque_interceptor, friction)
  CI = interfaces[MAZDA.MAZDA_CX9_2021](CP)
  controller = LatControlTorque(CP.as_reader(), CI, DT_CTRL, controller_mode=mode)
  VM = VehicleModel(CP)
  CS = car.CarState.new_message()
  CS.vEgo = v_ego
  CS.steeringPressed = False
  params = log.VehicleParameters.new_message()
  params.steerRatio = CP.steerRatio
  params.stiffnessFactor = 1.0
  output = 0.0
  for settled, error in ((False, 0.0), (True, curvature_error)):
    CS.steeringAngleDeg = math.degrees(VM.get_steer_from_curvature(-(curvature - error), v_ego, 0.0))
    if settled:
      controller.pid.i = 0.0
    for _ in range(frames):
      output, _, _ = controller.update(True, CS, VM, params, False, curvature, False, 0.2)
  return controller, output


class TestMazdaTiShaping(OpenpilotTestCase):
  def test_plant_scales(self):
    # more lateral accel per unit of torque and more friction at low speed, a dip near 15 m/s, the learned highway
    # value from 18.75 m/s (gain) and 21 m/s (friction) up
    self.assertAlmostEqual(get_mazda_ti_lat_accel_factor_scale(0.0), 1.3)
    self.assertAlmostEqual(get_mazda_ti_lat_accel_factor_scale(8.75), 1.3)
    self.assertAlmostEqual(get_mazda_ti_lat_accel_factor_scale(15.0), 0.9)
    self.assertAlmostEqual(get_mazda_ti_friction_scale(0.0), 1.6)
    self.assertAlmostEqual(get_mazda_ti_friction_scale(13.5), 1.5)
    for v_ego in (21.0, 25.0, 35.0):
      self.assertEqual(get_mazda_ti_lat_accel_factor_scale(v_ego), 1.0)
      self.assertEqual(get_mazda_ti_friction_scale(v_ego), 1.0)

  def test_error_scale_reproduces_the_tested_effective_gain(self):
    # the fork's effective gain: about 1.1 per m/s^2 of error from 5 to 13.5 m/s, 0.76 at 20 m/s, the 0.6 highway gain
    # from 30 m/s (StarPilot 6's KP of 0.6 plus its low-speed factor, times its error scale)
    for v_ego in (5.0, 7.5, 10.0, 13.5):
      self.assertTrue(1.05 <= sp6_effective_gain(v_ego) * SP6_KP <= 1.15, (v_ego, sp6_effective_gain(v_ego)))
    self.assertAlmostEqual(sp6_effective_gain(20.0) * SP6_KP, 0.76, places=6)
    self.assertAlmostEqual(sp6_effective_gain(30.0) * SP6_KP, SP6_KP + (5.0 / 30.0) ** 2, places=6)
    for v_ego in (30.0, 35.0):
      self.assertEqual(mazda_ti_error_scale(v_ego, INTERP_SPEEDS, KP_INTERP), 1.0)
    for v_ego in (3.0, 5.0, 7.5, 10.0, 13.5, 20.0, 25.0):
      scale = mazda_ti_error_scale(v_ego, INTERP_SPEEDS, KP_INTERP)
      controller_gain = float(np.interp(v_ego, INTERP_SPEEDS, KP_INTERP)) / KP_INTERP[-1]
      self.assertLess(scale, 1.0)
      self.assertAlmostEqual(scale * controller_gain, sp6_effective_gain(v_ego), places=6)
    # no hole: from 5 m/s up the converted gain never drops below the highway gain, and it falls toward it with speed
    # from 13.5 m/s (the scale itself is the ratio of two piecewise-linear schedules and need not be monotonic)
    speeds = np.linspace(5.0, 30.0, 101)
    gains = [mazda_ti_error_scale(v, INTERP_SPEEDS, KP_INTERP) * float(np.interp(v, INTERP_SPEEDS, KP_INTERP)) / KP_INTERP[-1]
             for v in speeds]
    self.assertTrue(min(gains) >= 1.0)
    self.assertTrue(np.all(np.diff([g for v, g in zip(speeds, gains, strict=True) if v >= 13.5]) <= 1e-9))

  @parameterized.expand([(3.0,), (5.0,), (8.5,), (10.0,), (12.0,)])
  def test_softens_low_speed_error_response(self, v_ego):
    # 0.005 1/m short of the request: the proportional response is scaled for the TI only
    ti, _ = run_torque(v_ego, True, 0.02, 0.005)
    stock, _ = run_torque(v_ego, False, 0.02, 0.005)
    self.assertTrue(ti.is_mazda_ti and not stock.is_mazda_ti)
    self.assertGreater(abs(stock.pid.p), 0.0)
    self.assertAlmostEqual(ti.pid.p, stock.pid.p * ti.mazda_ti_error_scale(v_ego), delta=abs(stock.pid.p) * 1e-6)

  @parameterized.expand([(5.5,), (8.5,), (12.0,)])
  def test_softens_low_speed_integral(self, v_ego):
    # a small, unsaturated error held for 1.5 s: the integral builds at the scaled rate (the controller freezes it below 5 m/s)
    ti, ti_out = run_torque(v_ego, True, 0.01, 0.0005, friction=0.0)
    stock, stock_out = run_torque(v_ego, False, 0.01, 0.0005, friction=0.0)
    self.assertTrue(abs(ti_out) < 1.0 and abs(stock_out) < 1.0)
    self.assertGreater(abs(stock.pid.i), 1e-4)
    self.assertAlmostEqual(ti.pid.i, stock.pid.i * ti.mazda_ti_error_scale(v_ego), delta=abs(stock.pid.i) * 1e-6)

  @parameterized.expand([(3.0,), (5.0,), (8.5,), (10.0,), (12.0,), (15.0,)])
  def test_shapes_the_torque_by_speed(self, v_ego):
    # with friction off, pid.f is the feedforward alone: the same lateral accel (0.5 m/s^2, inside the TI car's limit
    # at every speed) is asked for, converted to torque by the speed-shaped latAccelFactor
    ti, ti_out = run_torque(v_ego, True, 0.5 / v_ego ** 2, 0.0, friction=0.0)
    stock, stock_out = run_torque(v_ego, False, 0.5 / v_ego ** 2, 0.0, friction=0.0)
    self.assertGreater(abs(stock.pid.f), 0.0)
    self.assertAlmostEqual(ti.pid.f, stock.pid.f, delta=abs(stock.pid.f) * 1e-6)
    self.assertAlmostEqual(ti_out, stock_out / get_mazda_ti_lat_accel_factor_scale(v_ego), delta=abs(stock_out) * 1e-4)
    self.assertEqual(float(ti.torque_params.friction), 0.0)

  @parameterized.expand([(5.0,), (12.0,), (25.0,)])
  def test_speed_shape_rides_on_the_live_params(self, v_ego):
    ti, _ = run_torque(v_ego, True, 0.01, 0.0)
    ti.update_torque_parameters(2.0, 0.0, 0.2)
    ti.update(True, car.CarState.new_message(vEgo=v_ego), VehicleModel(cx9_params()), log.VehicleParameters.new_message(),
              False, 0.0, False, 0.2)
    self.assertAlmostEqual(float(ti.torque_params.latAccelFactor), 2.0 * get_mazda_ti_lat_accel_factor_scale(v_ego), places=5)
    self.assertAlmostEqual(float(ti.torque_params.friction), 0.2 * get_mazda_ti_friction_scale(v_ego), places=5)
    # the PID limits follow: the TI ceiling (700 of 800 counts) at the shaped gain
    self.assertAlmostEqual(ti.steer_max, 700 / 800)
    self.assertAlmostEqual(ti.pid.pos_limit, ti.steer_max * float(ti.torque_params.latAccelFactor), places=5)

  def test_saturates_at_the_ti_ceiling(self):
    # a turn no torque can make: the TI car's output stops at 700/800 and reports saturation, the stock EPS car's at 1.0
    outputs = {}
    for torque_interceptor in (True, False):
      CP = cx9_params(torque_interceptor)
      CI = interfaces[MAZDA.MAZDA_CX9_2021](CP)
      controller = LatControlTorque(CP.as_reader(), CI, DT_CTRL, controller_mode=ControllerMode.STANDARD)
      CS = car.CarState.new_message(vEgo=30.0)
      params = log.VehicleParameters.new_message(steerRatio=CP.steerRatio, stiffnessFactor=1.0)
      lac_log = None
      for _ in range(1000):
        out, _, lac_log = controller.update(True, CS, VehicleModel(CP), params, False, 0.01, False, 0.2)
      outputs[torque_interceptor] = (round(abs(out), 6), bool(lac_log.saturated))
    self.assertEqual(outputs[True], (round(700 / 800, 6), True))
    self.assertEqual(outputs[False], (1.0, True))

  @parameterized.expand([(30.0,), (31.0,), (33.0,)])
  def test_unchanged_at_highway_speed(self, v_ego):
    # a gentle curve, inside the TI car's limit (700 of 800 counts): from 30 m/s up the only difference left is that limit
    for curvature_error in (0.0, 0.0001, -0.00015):
      ti, ti_out = run_torque(v_ego, True, 0.0005, curvature_error)
      stock, stock_out = run_torque(v_ego, False, 0.0005, curvature_error)
      self.assertTrue(abs(ti_out) < 0.7 and abs(stock_out) < 0.7, (ti_out, stock_out))
      self.assertEqual(ti_out, stock_out)
      self.assertEqual(ti.pid.i, stock.pid.i)

  def test_flag_is_ignored_on_other_brands(self):
    CarInterface = interfaces[TOYOTA.TOYOTA_RAV4]
    CP = CarInterface.get_non_essential_params(TOYOTA.TOYOTA_RAV4)
    CP.flags = int(CP.flags) | int(MazdaSafetyFlags.TORQUE_INTERCEPTOR)
    self.assertFalse(LatControlTorque(CP.as_reader(), CarInterface(CP), DT_CTRL).is_mazda_ti)


class TestMazdaTiStarPilotController(OpenpilotTestCase):
  """The StarPilot controller (StarPilot 6's) with the fork's TI shaping, this car's default."""

  def test_default_controller(self):
    self.assertEqual(default_selection(cx9_params(True)).mode, ControllerMode.STARPILOT)
    self.assertEqual(default_selection(cx9_params(False)).mode, ControllerMode.STANDARD)
    CP = cx9_params(True)
    controller = LatControlTorque(CP.as_reader(), interfaces[MAZDA.MAZDA_CX9_2021](CP), DT_CTRL)
    self.assertEqual(controller.controller_mode, ControllerMode.STARPILOT)
    self.assertTrue(controller.is_mazda_ti and controller.starpilot_extension.policy.mazda_ti)

  @parameterized.expand([(3.0,), (5.0,), (7.5,), (10.0,), (12.0,), (13.5,), (20.0,), (30.0,)])
  def test_effective_gain_is_starpilot_6s(self, v_ego):
    # the proportional term per m/s^2 of lateral-accel error is StarPilot 6's (KP + LSF) * error scale; the stock
    # (non-TI) car on the same controller has no error scale
    curvature_error = 0.05 / v_ego ** 2
    ti, _ = run_torque(v_ego, True, 0.02, curvature_error, mode=ControllerMode.STARPILOT)
    stock, _ = run_torque(v_ego, False, 0.02, curvature_error, mode=ControllerMode.STARPILOT)
    lsf = (float(np.interp(v_ego, SP6_LOW_SPEED_X, SP6_LOW_SPEED_Y)) / v_ego) ** 2
    stock_gain = stock.pid.p / 0.05
    self.assertAlmostEqual(stock_gain, SP6_KP + lsf, delta=(SP6_KP + lsf) * 0.02)
    self.assertAlmostEqual(ti.pid.p, stock.pid.p * get_sp6_mazda_ti_error_scale(v_ego), delta=abs(stock.pid.p) * 1e-6)
    self.assertAlmostEqual(ti.pid.p / 0.05, sp6_effective_gain(v_ego) * SP6_KP, delta=sp6_effective_gain(v_ego) * SP6_KP * 0.02)

  @parameterized.expand([(3.0,), (8.5,), (15.0,), (25.0,)])
  def test_shapes_the_plant_by_speed(self, v_ego):
    ti, _ = run_torque(v_ego, True, 0.01, 0.0, mode=ControllerMode.STARPILOT)
    base_lat_accel_factor, base_friction = ti.mazda_ti_base_params
    self.assertAlmostEqual(float(ti.torque_params.latAccelFactor), base_lat_accel_factor * get_mazda_ti_lat_accel_factor_scale(v_ego), places=5)
    self.assertAlmostEqual(float(ti.torque_params.friction), base_friction * get_mazda_ti_friction_scale(v_ego), places=5)
    self.assertAlmostEqual(ti.pid.pos_limit, ti.steer_max * float(ti.torque_params.latAccelFactor), places=5)

  def test_saturates_at_the_ti_ceiling(self):
    CP = cx9_params(True)
    controller = LatControlTorque(CP.as_reader(), interfaces[MAZDA.MAZDA_CX9_2021](CP), DT_CTRL, controller_mode=ControllerMode.STARPILOT)
    CS = car.CarState.new_message(vEgo=30.0)
    params = log.VehicleParameters.new_message(steerRatio=CP.steerRatio, stiffnessFactor=1.0)
    for _ in range(1000):
      out, _, lac_log = controller.update(True, CS, VehicleModel(CP), params, False, 0.01, False, 0.2)
    self.assertEqual((round(abs(out), 6), bool(lac_log.saturated)), (round(700 / 800, 6), True))
