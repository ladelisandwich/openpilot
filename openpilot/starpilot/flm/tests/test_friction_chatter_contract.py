"""Model noise remains ineligible even for explicitly retained controller chatter."""
from dataclasses import replace

import pytest

from openpilot.starpilot.flm.canonical_surface_adapter import GROUPS
from openpilot.starpilot.flm.controller_replay import pack_primitive_snapshot
from openpilot.starpilot.flm.event_objective import EffectObjective
from openpilot.starpilot.flm.evidence_noise import reject_noisy_job


def noisy_payload(*, model=False):
  frames = []
  points = []
  for i in range(101):
    goal = (.2 if i % 10 < 5 else -.2) if model else .6
    command = .2 if i % 10 < 5 else -.2
    inputs = (('raw_model_curvature', goal / 900.), ('vEgo', 30.))
    frames.append({'control_ns': 1_000_000_000 + i * 10_000_000, 'inputs': inputs, 'actual_command': command})
    points.append({'control_ns': frames[-1]['control_ns'],
                   'control': {'replay_inputs': inputs, 'requested': command}})
  return {'replay': pack_primitive_snapshot({'frames': frames}), 'raw_blocks': [{'samples': points}]}


@pytest.mark.parametrize('group', tuple(GROUPS))
def test_exact_canonical_objective_only_retains_expected_chatter(group):
  objective = EffectObjective(group, GROUPS[group], 'a' * 64, 'b' * 64)
  permitted = group in ('center', 'deadband.low', 'deadband.high', 'friction.left', 'friction.right')
  assert objective.retains_controller_chatter is permitted
  assert reject_noisy_job(noisy_payload(), retain_centre_chatter=objective.retains_controller_chatter) == (
    None if permitted else 'controller_chatter')
  assert reject_noisy_job(noisy_payload(model=True), retain_centre_chatter=objective.retains_controller_chatter) == 'model_goal_wobble'


def test_nonfriction_coordinates_cannot_claim_friction_contract():
  objective = EffectObjective('friction.left', GROUPS['friction.left'], 'a' * 64, 'b' * 64)
  with pytest.raises(ValueError, match='canonical event group'):
    replace(objective, names=GROUPS['ff.left'])
