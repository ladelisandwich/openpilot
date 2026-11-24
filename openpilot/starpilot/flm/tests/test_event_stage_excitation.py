"""Independent opposite turns cannot be relabeled as a directional objective."""
from types import SimpleNamespace

import pytest

from openpilot.starpilot.flm.event_raw_fit import objective_excited


def event(features):
  points = tuple({'apply_ns': index + 1} for index in range(40))
  engine = SimpleNamespace(projected={point['apply_ns']: (0., features) for point in points})
  objective = SimpleNamespace(parameters=tuple(SimpleNamespace(excitation_floor=.01) for _ in features))
  return {'samples': points}, engine, objective


def test_zero_right_effect_cannot_masquerade_as_left_estimation_cohort():
  actual, engine, objective = event((0.,))
  assert not objective_excited(actual, engine, objective)
  assert len(actual['samples']) == 40 and len(engine.projected) == 40


def test_same_original_excitation_floor_and_whole_point_set():
  actual, engine, objective = event((.02,))
  original = tuple(actual['samples'])
  assert objective_excited(actual, engine, objective)
  assert tuple(actual['samples']) == original
  actual, engine, objective = event((.009,))
  assert not objective_excited(actual, engine, objective)


def test_every_identified_coordinate_needs_whole_episode_support():
  actual, engine, objective = event((.1, 0.))
  assert not objective_excited(actual, engine, objective)
  actual, engine, objective = event((.1, .1))
  assert objective_excited(actual, engine, objective)
  engine.projected[1] = (0., (.1,))
  with pytest.raises(ValueError):
    objective_excited(actual, engine, objective)
