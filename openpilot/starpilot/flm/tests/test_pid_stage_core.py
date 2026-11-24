"""Registered torque-PID controller stage tests; no fitted profile acceptance claim."""

from types import SimpleNamespace
import math

import pytest

from opendbc.car import structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.gm.values import CAR as GM
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.starpilot.flm.canonical_surface_adapter import CanonicalSurfaceEffects
from openpilot.starpilot.flm.pid_stages import PidObservation, SUPPORTED, UNSUPPORTED, canonical_feedforward, canonical_error, PidFrame


class PidTrialSource:
  """Pure candidate injection only; production admission tested separately."""

  def __init__(self, coordinates):
    self.source = CanonicalSurfaceEffects(coordinates, 'a' * 64)
    self.original_effect_anchor = self.source.original_effect_anchor
    self.frame = 0

  def selected_pid_stage(self, frame, *, original_anchor):
    return canonical_feedforward(self.source, frame, original_anchor=original_anchor)

  def selected_pid_error(self, frame, *, original_anchor):
    return canonical_error(self.source, frame, original_anchor=original_anchor)

  def selected_output(self, *args, **kwargs):
    return self.source.selected_output(*args, **kwargs)

  def begin_frame(self):
    self.frame += 1
    self.source.begin_frame(self.frame)


def controllers(platform):
  cls = interfaces[platform]
  cp = cls.get_non_essential_params(platform)
  assert cp.steerControlType == structs.CarParams.SteerControlType.torque
  assert cp.lateralTuning.which() == 'pid'
  ci = cls(cp)
  baseline, candidate = LatControlPID(cp, ci, 0.01), LatControlPID(cp, ci, 0.01)
  candidate.automatic_flm_observation = PidObservation()
  return cp, baseline, candidate


def update(lac, cp, tick, *, active=True, override=False, limited=False):
  cs = structs.CarState(vEgo=18.0, canValid=True, steeringAngleDeg=0.15 * math.sin(tick * 0.01), steeringPressed=override)
  params = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)
  vm = VehicleModel(cp)
  return lac.update(active, cs, vm, params, limited, -0.0002 * math.sin(tick * 0.015), False, 0.1)


@pytest.mark.parametrize('platform', (HONDA.HONDA_CIVIC, HONDA.HONDA_CIVIC_BOSCH, HONDA.HONDA_PILOT_4G, HONDA.ACURA_TLX_2G_MMR, GM.CADILLAC_ESCALADE_ESV))
def test_actual_pid_zero_effect_preserves_original_controller_state_and_tunes(platform):
  cp, baseline, candidate = controllers(platform)
  original = cp.to_bytes()
  source = PidTrialSource({'ff_gain_left': 0.0})
  candidate.automatic_flm_effects = source
  for tick in range(300):
    source.begin_frame()
    candidate.automatic_flm_observation.begin_frame()
    expected = update(baseline, cp, tick)
    actual = update(candidate, cp, tick)
    assert actual[0] == expected[0] and actual[1] == expected[1]
    assert candidate.pid.i == baseline.pid.i
    assert tuple(map(tuple, candidate.pid._k_p)) == tuple(map(tuple, baseline.pid._k_p)) and tuple(map(tuple, candidate.pid._k_i)) == tuple(
      map(tuple, baseline.pid._k_i)
    )
    assert cp.to_bytes() == original
  assert len(SUPPORTED) == 26 and len(UNSUPPORTED) == 4


@pytest.mark.parametrize('platform', (HONDA.HONDA_CIVIC, HONDA.HONDA_CIVIC_BOSCH, GM.CADILLAC_ESCALADE_ESV))
def test_actual_pid_own_feedforward_has_bounded_nonzero_canonical_effect(platform):
  cp, baseline, candidate = controllers(platform)
  source = PidTrialSource({'ff_gain_left': 0.02, 'ff_gain_right': 0.02})
  candidate.automatic_flm_effects = source
  effect = False
  original = cp.to_bytes()
  for tick in range(300):
    source.begin_frame()
    observation = candidate.automatic_flm_observation
    observation.begin_frame()
    ordinary = update(baseline, cp, tick)
    actual = update(candidate, cp, tick)
    assert abs(actual[0]) <= candidate.steer_max
    assert candidate.ff_factor == baseline.ff_factor
    assert cp.to_bytes() == original
    effect |= actual[0] != ordinary[0]
    trace = observation.finish_frame(candidate, actual[2], selected_command=actual[0])
    assert trace is not None and trace[0] == 'pid-selected-stage-snapshot-v2'
    assert 'friction' not in dict(trace[1]) and 'effective_factor' not in dict(trace[1])
  assert effect


@pytest.mark.parametrize('coordinate', tuple(sorted(UNSUPPORTED)))
def test_pid_refuses_unmeasured_friction_threshold_coordinates(coordinate):
  source = CanonicalSurfaceEffects({coordinate: 0.001}, 'a' * 64)
  source.begin_frame(1)
  frame = PidFrame(18.0, 0.4, 0.1, 0.02, 0.5, 1.0, 0.5, True, False, False)
  with pytest.raises(ValueError, match='unsupported'):
    canonical_feedforward(source, frame, original_anchor=source.original_effect_anchor)


def test_unsupported_stage_does_not_fall_through_to_partial_output_assist():
  cp, baseline, candidate = controllers(HONDA.HONDA_CIVIC)
  source = PidTrialSource({'turn_in_threshold_reduction_left': 0.01, 'low_speed_angle_assist_max_torque': 0.01})
  candidate.automatic_flm_effects = source
  for tick in range(100):
    source.begin_frame()
    candidate.automatic_flm_observation.begin_frame()
    ordinary = update(baseline, cp, tick)
    actual = update(candidate, cp, tick)
    assert actual[0] == ordinary[0]
    assert not candidate.automatic_pid_stage_admitted


@pytest.mark.parametrize('speed,index,cap', ((0.0, 0, 0.30), (5.0, 1, 0.30), (10.0, 2, 0.20), (15.0, 3, 0.12), (25.0, 4, 0.08)))
@pytest.mark.parametrize('error', (-20.0, -1.0, -0.01, 0.0, 0.01, 1.0, 20.0))
def test_actual_pid_degree_error_normalizer_respects_original_error_and_raw_caps(speed, index, cap, error):
  names = ('center_deadband_crawl_deg', 'center_deadband_low_deg', 'center_deadband_mid_deg', 'center_deadband_fast_deg', 'center_deadband_highway_deg')
  source = CanonicalSurfaceEffects({names[index]: 0.25}, 'a' * 64)
  source.begin_frame(1)
  frame = PidFrame(speed, 0.4, 0.1, 0.02, error, error, 0.0, True, False, False)
  actual = canonical_error(source, frame, original_anchor=source.original_effect_anchor)
  assert actual == math.copysign(abs(error) - min(0.25 * abs(error), cap), error)
  assert 0.75 * abs(error) <= abs(actual) <= abs(error)
  assert actual * error >= 0.0


def test_actual_registered_pid_deadband_changes_p_error_without_gain_or_ff_retune():
  cp, baseline, candidate = controllers(HONDA.HONDA_CIVIC)
  source = PidTrialSource({'center_deadband_fast_deg': 0.10, 'center_deadband_highway_deg': 0.10})
  candidate.automatic_flm_effects = source
  effect = False
  original = cp.to_bytes()
  for tick in range(300):
    source.begin_frame()
    observation = candidate.automatic_flm_observation
    observation.begin_frame()
    ordinary = update(baseline, cp, tick)
    actual = update(candidate, cp, tick)
    assert tuple(map(tuple, candidate.pid._k_p)) == tuple(map(tuple, baseline.pid._k_p)) and tuple(map(tuple, candidate.pid._k_i)) == tuple(
      map(tuple, baseline.pid._k_i)
    )
    assert candidate.ff_factor == baseline.ff_factor and cp.to_bytes() == original
    assert 0.75 * abs(observation.stage.error_deg) <= abs(observation.learned_error) <= abs(observation.stage.error_deg)
    assert actual[2].f == ordinary[2].f
    effect |= actual[0] != ordinary[0]
  assert effect


@pytest.mark.parametrize('platform', (HONDA.HONDA_CIVIC, HONDA.HONDA_CIVIC_BOSCH, GM.CADILLAC_ESCALADE_ESV))
@pytest.mark.parametrize('curvature', (-0.0002, 0.0002))
def test_actual_pid_stage_physical_sign_matches_original_vehicle_model(platform, curvature):
  cp, baseline, candidate = controllers(platform)
  cs = structs.CarState(vEgo=18.0, canValid=True, steeringAngleDeg=0.0)
  params = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)
  vm = VehicleModel(cp)
  source = PidTrialSource({'ff_gain_left': 0.0})
  candidate.automatic_flm_effects = source
  source.begin_frame()
  candidate.automatic_flm_observation.begin_frame()
  ordinary = baseline.update(True, cs, vm, params, False, curvature, False, 0.1)
  actual = candidate.update(True, cs, vm, params, False, curvature, False, 0.1)
  frame = candidate.automatic_flm_observation.stage
  assert actual[0] == ordinary[0] and actual[1] == ordinary[1]
  assert frame.setpoint == curvature * cs.vEgo**2
  assert frame.desired_angle_deg * curvature < 0.0
  assert frame.baseline_ff * curvature < 0.0
  assert actual[0] * curvature < 0.0
