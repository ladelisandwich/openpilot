"""Exact original-frame join; no synthetic replay qualification."""
from types import SimpleNamespace
from typing import TypedDict
from unittest import mock

import pytest

from openpilot.starpilot.flm.event_raw_fit import _control_frame_index, fit_event_torque
from openpilot.starpilot.flm.tests.test_event_activation import fitted_model


class StageSample(TypedDict):
  control_ns: int
  apply_ns: int
  mono_ns: int
  control: dict[str, int]
  output: dict[str, int]
  physical: dict[str, float]


class StageEvent(TypedDict):
  block_id: str
  samples: tuple[StageSample, ...]


def test_index_preserves_original_frame_identity_and_missing_timestamp():
  first = SimpleNamespace(control_ns=10, desired_physical_accel=.02)
  second = SimpleNamespace(control_ns=20, desired_physical_accel=.03)
  frames = _control_frame_index((first, second))
  assert frames[10] is first and frames[20] is second
  assert frames.get(30) is None


@pytest.mark.parametrize('stamp', (10, 0, True, 10.))
def test_actual_fit_rejects_duplicate_or_invalid_frames_before_fitting(stamp):
  engine = SimpleNamespace(records=(SimpleNamespace(control_ns=10), SimpleNamespace(control_ns=stamp)))
  with mock.patch('openpilot.starpilot.flm.event_raw_fit.original_event_evidence') as evidence, \
       pytest.raises(ValueError, match='duplicate/invalid original control frame'):
    fit_event_torque({}, engine, None)
  evidence.assert_not_called()


def test_actual_fit_missing_stage_frame_is_fail_closed():
  # Isolate the stage join after its independently checked upstream operations.
  # The model comes from the real measured-row fitter, never a qualification flag.
  model = fitted_model()
  events: tuple[StageEvent, ...] = tuple({'block_id': str(index), 'samples': ({'control_ns': index * 1_000_000_000,
    'apply_ns': index * 1_000_000_000 + 1, 'mono_ns': index * 1_000_000_000 + 2,
    'control': {'stamp': index * 1_000_000_000}, 'output': {'stamp': index * 1_000_000_000 + 1},
    'physical': {'lateral_accel': .02}},)} for index in range(1, 17))
  first_stage = events[8]['samples'][0]
  engine = SimpleNamespace(records=(), identity=model.identity, projected={}, ensure_causal_features=lambda: None)
  objective = SimpleNamespace(names=('center',), required_cohorts=('center',))
  envelope = SimpleNamespace(prior_coordinates=(('center', 0.),))
  with mock.patch('openpilot.starpilot.flm.event_raw_fit.original_event_evidence',
                  return_value=(envelope, model.context, model.lag_ns, (), events)), \
       mock.patch('openpilot.starpilot.flm.event_raw_fit.fit_event_physical', return_value=model), \
       mock.patch('openpilot.starpilot.flm.event_raw_fit.objective_excited', return_value=True), \
       mock.patch('openpilot.starpilot.flm.event_raw_fit.control_from_dict',
                  side_effect=lambda values, *, trace_archive: SimpleNamespace(control_ns=values['stamp'])) as control, \
       mock.patch('openpilot.starpilot.flm.event_raw_fit.output_from_dict',
                  side_effect=lambda values: SimpleNamespace(apply_ns=values['stamp'])), \
       mock.patch('openpilot.starpilot.flm.event_raw_fit.delivered', return_value=.01), \
       pytest.raises(ValueError, match='supported actual physical response'):
    engine.projected[first_stage['apply_ns']] = (.01, (1.,))
    fit_event_torque({}, engine, objective)
  # This isolated legacy fixture has no normalized trace archive; the stage
  # decoder must still receive the explicit archive argument before the join.
  control.assert_called_once_with(first_stage['control'], trace_archive=None)
