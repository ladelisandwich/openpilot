"""Root-only actual Controls -> selected LaC -> typed evidence -> real CI/CAN.

The driver accelerator alone does not replace active lateral control. Pedal,
steering, fault, native, source and pose controls remain separate qualifications.
No controller response, fit, native outcome or emitted proof is supplied.
"""
import math
from types import SimpleNamespace
from unittest import mock

import pytest

from opendbc.car.car_helpers import interfaces
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.hyundai.values import CAR as HYUNDAI
from opendbc.car.toyota.values import CAR as TOYOTA
from opendbc.car.steering_projection import install_observer, inspect_projection
from openpilot.cereal import messaging
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.starpilot.flm.evidence_hooks import control_message, begin_applied_frame, applied_snapshot
from openpilot.starpilot.flm.tests.test_registered_controls_global import controls_for, frame
from openpilot.starpilot.flm.tests.test_physical_pose_queue import fixture, add_state, observe, BASE as POSE_BASE

BASE = 600_000_000_000
CONFIGURATIONS = (
  (TOYOTA.TOYOTA_RAV4_TSS2, 'standard', LatControlTorque),
  (HYUNDAI.HYUNDAI_IONIQ_6, 'starpilot', LatControlTorque),
  (HONDA.HONDA_CIVIC, None, LatControlPID),
)


def controls_tick(controls, tick, changes):
  original = controls.sm.update_msgs
  def deliver(received, packets):
    updated = []
    for packet in packets:
      if packet.which() == 'carState':
        packet = packet.as_builder()
        if type(controls.LaC) is LatControlPID:
          # Declared kinematic tracking input for the fixture's unchanged
          # .0002 model curvature and30m/s speed. Holding measured angle0
          # indefinitely accumulates PID integral/saturation, unrelated to gas.
          packet.carState.steeringAngleDeg = math.degrees(
            controls.VM.get_steer_from_curvature(-.0002, 30., 0.))
        for name, value in changes.items():
          setattr(packet.carState, name, value)
        packet = packet.as_reader()
      updated.append(packet)
    return original(received, updated)
  with mock.patch.object(controls.sm, 'update_msgs', side_effect=deliver):
    command, log = frame(controls, BASE + tick * 10_000_000, tick)
  return command, log


@pytest.mark.parametrize('platform,mode,lac_type', CONFIGURATIONS)
@pytest.mark.parametrize('gas', (False, True))
def test_actual_active_lateral_driver_gas_retains_control_and_consumed_can_evidence(platform, mode, lac_type, gas):
  cp = interfaces[platform].get_non_essential_params(platform)
  with controls_for(cp, mode, True) as controls:
    assert type(controls.LaC) is lac_type
    original_cp = controls.CP.as_builder().to_bytes()
    controls.CI.CS.update(controls.CI.can_parsers)
    assert install_observer(controls.CI)
    for tick in range(180):
      command, _ = controls_tick(controls, tick, {'gasPressed': gas})
    now = BASE + 179 * 10_000_000
    assert command.latActive
    assert controls.sm['carState'].gasPressed is gas
    automatic = controls.global_learning
    assert automatic is not None
    assert automatic.frame is not None, {
      'reason': automatic.reason, 'eligible': automatic.eligible,
      'domain': None if automatic.descriptor is None else automatic.descriptor.domain,
      'satTime': controls.LaC.sat_time, 'stage': automatic.observation.stage}
    assert automatic.source is not None
    if lac_type is LatControlPID:
      assert automatic.descriptor.domain == 'torque_pid'
      assert automatic.observation.stage is not None
      assert abs(automatic.observation.stage.error_deg) < .01
      assert controls.LaC.sat_time == 0.
    else:
      assert automatic.descriptor.domain == 'torque'
    assert automatic.source.enabled and not automatic.source.manual and automatic.eligible
    evidence = messaging.new_message('starpilotLateralState', valid=True, logMonoTime=now)
    assert control_message(evidence, controls.LaC, command, control_ns=now,
      state_ns=controls.sm.logMonoTime['carState'], limited=controls.steer_limited_by_safety) is True
    wire = messaging.log_from_bytes(evidence.to_bytes()).starpilotLateralState.learningEvidence
    assert wire.version == 1 and wire.active and wire.valid and not wire.override
    assert not wire.manual and wire.enabled
    assert wire.stageValues and wire.replayInputs
    if lac_type is LatControlPID:
      assert wire.stageSchemaRevision == 'pid-selected-stage-snapshot-v2'
      values = {str(row.name): float(row.value) for row in wire.stageValues}
      assert abs(values['angle_error_deg']) < .01
      assert values['baseline_ff'] != 0. and values['ff_factor'] == pytest.approx(controls.LaC.ff_factor)
      assert values['pid_f'] == pytest.approx(float(controls.LaC.pid.f))
    # Native Card feedback comes from this selected controller's actual command.
    controls.CI.CS.out = controls.sm['carState']
    owner_sm = messaging.SubMaster(['carControl', 'pandaStates'])
    packet = messaging.new_message('carControl', valid=True, logMonoTime=now)
    packet.carControl = command
    consumed = messaging.log_from_bytes(packet.to_bytes()).carControl
    pandas = messaging.new_message('pandaStates', len(controls.CP.safetyConfigs), valid=True, logMonoTime=now)
    for panda, config in zip(pandas.pandaStates, controls.CP.safetyConfigs, strict=True):
      panda.safetyModel, panda.safetyParam = config.safetyModel, config.safetyParam
      panda.alternativeExperience = controls.CP.alternativeExperience
      panda.controlsAllowed = str(config.safetyModel) not in ('silent', 'noOutput')
    owner_sm.update_msgs(now / 1e9, [packet.as_reader(), pandas.as_reader()])
    owner = SimpleNamespace(CP=controls.CP, CI=controls.CI, sm=owner_sm)
    begin_applied_frame(owner)
    actuators, outgoing = controls.CI.apply(consumed, now + 1_000_000)
    witness = controls.CI.CC.learning_projection
    assert witness is not None and inspect_projection(controls.CI, consumed, actuators, witness) is not None
    assert (witness.can_address, witness.can_data, witness.can_bus) in outgoing
    applied = applied_snapshot(owner, consumed, controls.sm['carState'], actuators,
      apply_ns=now + 1_000_000, state_ns=controls.sm.logMonoTime['carState'])
    assert applied is not None and applied.active and applied.valid and not applied.override
    assert applied.native_allowed and applied.projection is not None
    for flag in ('steeringPressed', 'brakePressed', 'regenBraking'):
      changed = controls.sm['carState'].as_builder()
      setattr(changed, flag, True)
      rejected = applied_snapshot(owner, consumed, changed.as_reader(), actuators,
        apply_ns=now + 1_000_000, state_ns=controls.sm.logMonoTime['carState'])
      assert rejected is not None and rejected.override
    changed = controls.sm['carState'].as_builder()
    changed.buttonEvents = [{'type': 'cancel', 'pressed': True}]
    rejected = applied_snapshot(owner, consumed, changed.as_reader(), actuators,
      apply_ns=now + 1_000_000, state_ns=controls.sm.logMonoTime['carState'])
    assert rejected is not None and rejected.override
    assert controls.CP.as_builder().to_bytes() == original_cp


@pytest.mark.parametrize('platform,mode,lac_type', CONFIGURATIONS)
@pytest.mark.parametrize('flag', ('steeringPressed', 'brakePressed', 'regenBraking', 'steerFaultTemporary', 'steerFaultPermanent', 'accFaulted'))
def test_actual_gas_does_not_override_other_lateral_rejections(platform, mode, lac_type, flag):
  cp = interfaces[platform].get_non_essential_params(platform)
  with controls_for(cp, mode, True) as controls:
    for tick in range(180):
      command, _ = controls_tick(controls, tick, {'gasPressed': True, flag: True})
    now = BASE + 179 * 10_000_000
    evidence = messaging.new_message('starpilotLateralState', valid=True, logMonoTime=now)
    assert control_message(evidence, controls.LaC, command, control_ns=now,
      state_ns=controls.sm.logMonoTime['carState'], limited=controls.steer_limited_by_safety) is None
    assert evidence.starpilotLateralState.learningEvidence.version == 0


@pytest.mark.parametrize('gas', (False, True))
def test_real_pose_observer_gas_preserves_original_source_clock_and_interpolation(gas):
  collector, cp, sm = fixture()
  sm['carState'].gasPressed = gas
  add_state(collector, sm, POSE_BASE - 10_000_000, speed=20., angle=0.)
  assert observe(collector, cp, sm, POSE_BASE) is None
  add_state(collector, sm, POSE_BASE + 10_000_000, speed=22., angle=2.)
  result = observe(collector, cp, sm, POSE_BASE + 10_000_000)
  assert result is not None
  assert result.mono_ns == result.source_boot_ns == result.received_ns == POSE_BASE
  assert result.clock_offset_ns == result.clock_uncertainty_ns == 0
  assert result.speed_mps == 21. and result.steering_angle_deg == 1.
  assert result.yaw_rate == pytest.approx(.02)


@pytest.mark.parametrize('flag', ('steeringPressed', 'brakePressed', 'regenBraking', 'steerFaultTemporary', 'steerFaultPermanent', 'canTimeout'))
def test_real_pose_observer_gas_does_not_bypass_other_flags(flag):
  collector, cp, sm = fixture()
  sm['carState'].gasPressed = True
  setattr(sm['carState'], flag, True)
  add_state(collector, sm, POSE_BASE - 10_000_000)
  assert observe(collector, cp, sm, POSE_BASE) is None
  add_state(collector, sm, POSE_BASE + 10_000_000)
  assert observe(collector, cp, sm, POSE_BASE + 10_000_000) is None
