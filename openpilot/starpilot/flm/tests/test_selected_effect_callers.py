"""Actual registered controller plumbing, not physical fit/admission evidence."""

from functools import lru_cache
import math
from types import SimpleNamespace
import pytest
from openpilot.starpilot.flm.controller_replay import SelectedControllerFactory
from openpilot.starpilot.flm.tests.test_registered_controls_global import configurations
from openpilot.starpilot.lateral.controller_selection import policy_for


@lru_cache(maxsize=1)
def actual_paths():
  chosen = {}
  for cp, _ in configurations():
    if cp.lateralTuning.which() != 'torque' or cp.notCar or cp.passive or cp.dashcamOnly:
      continue
    for mode in ('standard', 'starpilot') if policy_for(cp) else ('standard',):
      key = (cp.brand, mode, policy_for(cp) if mode == 'starpilot' else None)
      if key not in chosen:
        chosen[key] = cp.as_reader().as_builder().to_bytes()
  assert len(chosen) >= 8
  return tuple(chosen.items())


class Probe:
  original_effect_anchor = 'a' * 64
  applied_coordinates = (0.0, 0.0)

  def __init__(self, stage=None):
    self.stage = stage
    self.calls = {'stage': 0, 'friction': 0, 'deadzone': 0, 'output': 0}

  def selected_stage(self, f, *, original_anchor):
    assert original_anchor == self.original_effect_anchor
    self.calls['stage'] += 1
    return (f.baseline_ff * 1.1 if self.stage == 'ff' else f.baseline_ff, f.baseline_threshold * 0.9 if self.stage == 'threshold' else f.baseline_threshold)

  def selected_friction(self, baseline, *, original_anchor):
    self.calls['friction'] += 1
    return baseline * 1.1 if self.stage == 'friction' else baseline

  def selected_deadzone(self, speed, baseline_accel, derivative, friction_accel, requested_accel, *, original_anchor):
    assert math.isfinite(derivative) and derivative > 0
    self.calls['deadzone'] += 1
    return baseline_accel + friction_accel * 0.1 if self.stage == 'deadzone' else baseline_accel

  def selected_output(self, speed, setpoint, desired, actual, baseline, friction_torque, *, original_anchor):
    self.calls['output'] += 1
    return baseline + 0.1 * friction_torque if self.stage == 'output' else baseline


@pytest.mark.parametrize('effect', [None, 'ff', 'threshold', 'friction', 'deadzone', 'output'])
def test_all_actual_registered_path_families_have_same_real_effect_consumers(effect):
  from opendbc.car import structs
  from openpilot.starpilot.flm.torque_observation import TorqueObservation
  from openpilot.starpilot.flm.controller_replay import CandidateSource
  from openpilot.starpilot.flm.torque_adapter import Objective

  for key, raw in actual_paths():
    with structs.CarParams.from_bytes(raw) as cp:
      factory = SelectedControllerFactory(cp, key[1], turn_assist=False)
      baseline, vm = factory.create()
      candidate, cvm = factory.create()
      saved = cp.to_dict()
      initial_tune = candidate.torque_params.to_dict()
      cs = structs.CarState.new_message(vEgo=30.0, steeringAngleDeg=0.0, steeringRateDeg=0.0, gearShifter='drive')
      params = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)
      source = Probe(effect)
      candidate.automatic_flm_effects = source
      # Existing source still observes exact stage once; this test does not
      # forge a learned profile or claim independent physical qualification.
      candidate.automatic_flm_stages = CandidateSource(Objective('feedforward', 'high', 'steady'), (0.0, 0.0))
      baseline.automatic_flm_stages = CandidateSource(Objective('feedforward', 'high', 'steady'), (0.0, 0.0))
      changes = 0
      # Saturated friction and the HKG highway threshold floor intentionally
      # mask small perturbations. Excite the SAME actual consumers across a
      # bounded deterministic near-centre error/speed sweep, preserving tunes.
      ticks = 640 if effect in ('threshold', 'deadzone') else 160
      for tick in range(ticks):
        if effect in ('threshold', 'deadzone'):
          cs.vEgo = 10.0 if tick < ticks // 2 else 20.0
          target_accel = (1e-5, 1e-4, 0.001, 0.003, 0.01, 0.03, 0.09, 0.2)[(tick // 40) % 8]
          desired = target_accel / cs.vEgo**2
        else:
          desired = 0.0001
        candidate.automatic_flm_observation = TorqueObservation()
        # CandidateSource is once-per-frame.
        candidate.automatic_flm_stages.calls = baseline.automatic_flm_stages.calls = 0
        reference = baseline.update(True, cs, vm, params, False, desired, False, 0.15)
        trial = candidate.update(True, cs, cvm, params, False, desired, False, 0.15)
        assert math.isfinite(trial[0]) and abs(trial[0]) <= candidate.steer_max
        if effect is None:
          assert reference[0] == trial[0], key
          assert reference[2].to_dict() == trial[2].to_dict(), key
        changes += reference[0] != trial[0]
      assert all(count == ticks for count in source.calls.values()), (key, source.calls)
      if effect is not None:
        assert changes, (key, effect, 'wired hook made no actual-controller difference')
      assert saved == cp.to_dict() and candidate.torque_params.to_dict() == initial_tune


def test_full30_owner_transition_replays_exact_actual_selected_controller_paths():
  from dataclasses import replace
  from opendbc.car import structs
  from openpilot.starpilot.flm.controller_replay import CandidateSource
  from openpilot.starpilot.flm.selected_effects import EffectReplayTransition, EffectTrial
  from openpilot.starpilot.flm.tests.test_selected_effect_owner import fixture, NOW, BASE
  from openpilot.starpilot.flm.torque_adapter import Admission, Objective

  for key, raw in actual_paths():
    with structs.CarParams.from_bytes(raw) as cp:
      factory = SelectedControllerFactory(cp, key[1], turn_assist=False)
      actual, vm = factory.create()
      replayed, rvm = factory.create()
      owner, p, v, context = fixture()
      envelope = replace(p.effect_envelope, prior_coordinates=p.effect_envelope.coordinates, prior_full_profile_sha256=p.effect_envelope.full_profile_sha256)
      trial = EffectTrial(owner.objective, p.coefficients, BASE, envelope.replay_anchor())
      replay = EffectReplayTransition(trial)
      cs = structs.CarState.new_message(vEgo=30.0, steeringAngleDeg=0.0, steeringRateDeg=0.0, gearShifter='drive')
      params = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)
      for tick in range(40):
        now = NOW + tick * 10_000_000
        healthy = tick not in (17, 18)
        owner.prepare(Admission(p, now, True, False, healthy, v), active=True, current_context=context)
        actual.automatic_flm_effects = owner
        replayed.automatic_flm_effects = replay.source_for_frame(now, owner.trace)
        for controller in (actual, replayed):
          controller.automatic_flm_stages = CandidateSource(Objective('feedforward', 'high', 'steady'), (0.0, 0.0))
        a = actual.update(True, cs, vm, params, False, 0.0001, False, 0.15)
        b = replayed.update(True, cs, rvm, params, False, 0.0001, False, 0.15)
        assert a[0] == b[0], (key, tick)
        assert a[2].to_dict() == b[2].to_dict(), (key, tick)
        assert abs(a[0]) <= actual.steer_max
