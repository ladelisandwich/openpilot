"""Real GM/Toyota/Hyundai encoder cadence with separate fullrate geometry.

This invokes actual CarInterface/CarController/CAN encoding and decodes every
observed native packet. Source pose/model/current-native are EXPLICIT numerical
unit seams; this does not claim Panda permission or an actual road experiment.
"""
from dataclasses import replace
import copy
from types import SimpleNamespace

import pytest

from opendbc.car import Bus, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.gm.values import CAR as GM
from opendbc.car.toyota.values import CAR as TOYOTA
from opendbc.car.hyundai.values import CAR as HYUNDAI
from opendbc.car.steering_projection import TorqueProjectionObserver
from openpilot.starpilot.flm.torque_emission_replay import Emission, EmissionDecoder
from openpilot.starpilot.flm.tests.test_live_tracking_episodes import (
  BASE, DT, owner, pair, numeric_pose, numerical_join_validation_only,
)


@pytest.mark.parametrize('fingerprint,step', [
  (GM.CHEVROLET_BOLT_EUV, 3), (TOYOTA.TOYOTA_RAV4_TSS2, 1), (HYUNDAI.HYUNDAI_IONIQ_6, 1),
])
def test_actual_native_cadence_never_thins_100hz_geometry_or_invents_20hz_poses(monkeypatch, fingerprint, step):
  interface_type = interfaces[fingerprint]
  cp = interface_type.get_non_essential_params(fingerprint)
  interface = interface_type(cp)
  if fingerprint == GM.CHEVROLET_BOLT_EUV:
    # Same parser-owned forwarding metadata assignment as GM CarState.update.
    # This initializes the unrelated camera status frame; it asserts no CAN
    # health, native permission or received physical vehicle traffic.
    interface.CS.pscm_status = copy.copy(interface.can_parsers[Bus.pt].vl['PSCMStatus'])
    # Retain current GM EPS tune: STEER_STEP=3, approximately33Hz. The
    # controller's old50Hz comment is not an observed transmission contract.
    assert interface.CC.params.STEER_STEP == 3
  state = structs.CarState.new_message(canValid=True, canTimeout=False, vEgo=20., vEgoRaw=20.,
    steeringAngleDeg=0., steeringRateDeg=0., steeringTorque=0., standstill=False)
  state.cruiseState.enabled = state.cruiseState.available = True
  state.cruiseState.speed = 20.
  interface.CS.out = state.as_reader()
  observer = TorqueProjectionObserver(cp)
  interface.CC.learning_projection_observer = observer
  command = structs.CarControl.new_message(enabled=True, latActive=True, longActive=False)
  command.actuators.torque = 0.
  decoder = EmissionDecoder(cp)
  builder = owner()
  numerical_join_validation_only(monkeypatch, builder)
  actual_emissions = []
  emitted_frames = []
  pose_count = 0
  for index in range(150):
    stamp = BASE+index*DT
    geometry = .1 if index < 25 or 75 <= index < 100 else -.1
    control, _, profile = pair(builder, index, geometry)
    assert builder.observe_source_control(control, profile_sha256=profile, now_ns=stamp, current_native=True)
    actuators, packets = interface.apply(command.as_reader(), stamp)
    projection = observer.current
    if projection is not None:
      assert projection.steer_requested
      assert any(packet[0] == projection.can_address and packet[1] == projection.can_data and packet[2] == projection.can_bus
                 for packet in packets)
      from openpilot.starpilot.flm.fullrate_collector import AppliedEvidence
      applied = AppliedEvidence(stamp, stamp, stamp, 'normalized_torque', actuators.torque,
        True, True, False, False, projection.unlimited_raw != projection.final_raw, True, projection)
      emission = Emission(stamp, stamp, stamp, projection, control.identity, control.source_epoch,
        control.baseline_revision, True, True, False, False, applied.limited, True, False, 0, 0, '')
      decoder.check(emission)  # Real packet/parser equality, not a patched native verdict.
      assert builder.observe_control(control, applied, profile_sha256=profile, now_ns=stamp, current_native=True)
      actual_emissions.append((control, applied))
      emitted_frames.append(projection.frame)
    if index % 5 == 0:
      target = stamp-builder.certificate.lag_ns
      eligible = [(c, o) for c, o in actual_emissions if o.apply_ns <= target]
      if eligible:
        c, o = eligible[-1]
        pose = numeric_pose(builder, c, yaw=0.)
        pose = replace(pose, mono_ns=stamp, received_ns=stamp, observed_ns=stamp, completed_ns=stamp,
          source_timestamp_ns=stamp, source_lower_ns=stamp, source_upper_ns=stamp)
        witness = SimpleNamespace(certificate=builder.certificate, last_control_ns=c.control_ns,
          last_apply_ns=o.apply_ns, last_pose_ns=stamp)
        assert builder.observe_pose(pose, c, o, witness=witness, now_ns=stamp, current_native=True)
        pose_count += 1
  assert len(builder.ticks) == 150
  expected_frames = tuple(range(step, 150, step)) if step > 1 else tuple(range(150))
  assert tuple(emitted_frames) == expected_frames
  assert len(actual_emissions) == len(expected_frames)
  assert sum(tick.output is not None for tick in builder.ticks.values()) == len(actual_emissions)
  assert pose_count == 26 and len(builder.endpoints) == pose_count
  completed = builder.drain()
  assert len(completed) == 1 and completed[0].kind == 'subtle'
  assert completed[0].start_ns == BASE+25*DT and completed[0].end_ns == BASE+100*DT-1
  assert completed[0].control_max_gap_ns == DT
  assert completed[0].endpoints < len(actual_emissions)
  if step > 1:
    assert len(actual_emissions) < len(builder.ticks)
