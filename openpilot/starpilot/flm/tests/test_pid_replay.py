"""Actual registered PID constructors/checkpoints/chronological selected updates."""
from dataclasses import replace
import math
from types import SimpleNamespace

import pytest

from opendbc.car import structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.honda.values import CAR
from openpilot.starpilot.flm.controller_replay import (
  ControllerReplay, ReplayFrame, _run, capture_snapshot, frame_input, restore_snapshot,
)
from openpilot.starpilot.flm.pid_controller_replay import PidControllerFactory, PID_SNAPSHOT_REVISION
from openpilot.starpilot.flm.runtime_adapter_registry import selected_descriptor, baseline_values, baseline_revision
from openpilot.starpilot.flm.torque_adapter import Objective


def factory():
  cp = interfaces[CAR.HONDA_CIVIC].get_non_essential_params(CAR.HONDA_CIVIC)
  assert cp.lateralTuning.which() == 'pid'
  return PidControllerFactory(cp, 'native')


def test_actual_pid_factory_descriptor_and_checkpoint_preserve_gains():
  f = factory()
  controller, vm = f.create()
  descriptor = selected_descriptor(f.cp, f.interface_class(f.cp), controller)
  assert descriptor.domain == 'torque_pid'
  snapshot = capture_snapshot(controller, vm)
  assert snapshot.revision == PID_SNAPSHOT_REVISION
  original = baseline_values(controller)
  controller.pid.i = .031
  controller.automatic_pid_previous_setpoint = .8
  restore_snapshot(controller, vm, snapshot)
  assert controller.pid.i == 0.
  assert controller.automatic_pid_previous_setpoint is None
  assert baseline_values(controller) == original
  assert all(not name.startswith('tune.') for name, _ in snapshot.values)


def test_pid_checkpoint_cannot_modify_stock_gains_or_drop_state():
  f = factory()
  controller, vm = f.create()
  snapshot = capture_snapshot(controller, vm)
  altered = tuple((name, value + .01 if name == 'controller.ff_factor' else value) for name, value in snapshot.values)
  with pytest.raises(ValueError, match='immutable actual gains'):
    restore_snapshot(controller, vm, replace(snapshot, values=altered))
  controller, vm = f.create()
  with pytest.raises(ValueError, match='incomplete or foreign'):
    restore_snapshot(controller, vm, replace(snapshot, values=snapshot.values[1:]))


def test_exact_actual_pid_replay_does_not_convert_to_torque_controller():
  f = factory()
  actual, vm = f.create()
  descriptor = selected_descriptor(f.cp, f.interface_class(f.cp), actual)
  engine = ControllerReplay(f, Objective('feedforward', 'high', 'steady'),
                            identity='a' * 64, baseline_revision=baseline_revision(descriptor, baseline_values(actual)))
  cp_bytes = f.cp.as_builder().to_bytes()
  original = baseline_values(actual)
  cs = structs.CarState.new_message(vEgo=20., steeringAngleDeg=0., gearShifter='drive')
  params = SimpleNamespace(angleOffsetDeg=.1, roll=.01)
  for tick in range(220):
    now = 400_000_000_000 + tick * 10_000_000
    if tick % 100 == 0:
      engine.checkpoint(now, capture_snapshot(actual, vm))
    curvature = .0001 if tick < 110 else -.0001
    values = frame_input(cs, vm, params, active=True, safety_limited=False,
                         desired_curvature=curvature, curvature_limited=False, lat_delay=.2)
    values += (('control_frame', float(tick + 1)), ('raw_model_curvature', curvature))
    output, _, _ = actual.update(True, cs, vm, params, False, curvature, False, .2)
    row = ReplayFrame(now, values, float(output), (0., 0.), frame_sequence=tick + 1,
      actual_lac_command=float(output), actual_stage_residual=(0., 0.), desired_physical_accel=curvature * cs.vEgo ** 2,
      controller_domain='torque_pid')
    assert engine.normalize(row, _verify_only=True) is not None
  assert not engine.failed
  assert baseline_values(actual) == original
  assert f.cp.as_builder().to_bytes() == cp_bytes
  with pytest.raises(ValueError, match='cannot consume torque tuning'):
    _run(actual, vm, values, engine.objective, (.01, 0.))


@pytest.mark.parametrize('kwargs', [{'mode': 'standard'}, {'mode': 'native', 'turn_assist': True},
                                   {'mode': 'native', 'manual_baseline': (1., 0., .1)}])
def test_pid_factory_rejects_torque_specific_configuration(kwargs):
  f = factory()
  with pytest.raises(ValueError, match='stock torque PID'):
    PidControllerFactory(f.cp, **kwargs)


def test_registered_pid_physical_sign_matches_actual_vm_and_native_request():
  f = factory()
  controller, vm = f.create()
  cs = structs.CarState.new_message(vEgo=20., steeringAngleDeg=0., gearShifter='drive')
  params = SimpleNamespace(angleOffsetDeg=0., roll=0.)
  curvature = .0001
  output, desired_angle, _ = controller.update(True, cs, vm, params, False, curvature, False, .2)
  assert output < 0.  # Exact normalized request, no invented actuator sign change.
  physical = -vm.calc_curvature(math.radians(desired_angle), cs.vEgo, params.roll) * cs.vEgo ** 2
  assert physical == pytest.approx(curvature * cs.vEgo ** 2)
