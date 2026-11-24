"""Primitive proof binding only; no fabricated controller/native qualification."""
from dataclasses import asdict, replace
import math

from openpilot.starpilot.flm.event_activation import (
  ACTIVATION_VERSION, EventActivationPlan, activation_from_dict, activation_matches_model,
  activation_model_from_dict, bind_activation_model,
)
from openpilot.starpilot.flm.event_physical import fit_event_physical
from openpilot.starpilot.flm.tests.test_event_physical import episodes


def fitted_model():
  rows = episodes()
  model = fit_event_physical('a' * 64, 'b' * 64,
    tuple(replace(row, context='b' * 64) for row in rows), lag_ns=50_000_000, now_ns=rows[-1].response_ns)
  assert model is not None
  return model


def primitive_plan(model):
  # This is a primitive lineage fixture, not an actual replay validation.
  return EventActivationPlan(ACTIVATION_VERSION, model.identity, 'c' * 64, 'd' * 64, model.context,
    model.train_blocks, model.heldout_blocks, 40_000_000_000,
    tuple('stage-' + str(index) for index in range(9)),
    ((1_000_000_000, 45_000_000_000, 41_000_000_000, 41_000_000_000),)).validate()


def must_reject(call):
  try:
    call()
  except (ValueError, TypeError):
    return
  raise AssertionError('malformed proof was accepted')


def test_actual_fitted_model_values_and_inverse_unchanged_when_binding_plan():
  model = fitted_model()
  plan = primitive_plan(model)
  bound = bind_activation_model(model, plan)
  for name in model.__dataclass_fields__:
    assert getattr(model, name) == getattr(bound, name)
  assert bound.inverse(.02) == model.inverse(.02)
  assert bound.inverse(100.) is None
  assert activation_matches_model(plan, bound)
  assert activation_model_from_dict(asdict(bound)) == bound
  assert activation_model_from_dict(asdict(model)) == model
  assert activation_from_dict(asdict(plan)) == plan


def test_altered_activation_cannot_reuse_old_model_revision_binding():
  model = fitted_model()
  plan = primitive_plan(model)
  bound = bind_activation_model(model, plan)
  moved = replace(plan, windows=((1_000_000_000, 45_000_000_000, 42_000_000_000, 42_000_000_000),))
  assert moved.digest() != plan.digest()
  assert not activation_matches_model(moved, bound)
  assert not activation_matches_model(replace(plan, stage_blocks=tuple(reversed(plan.stage_blocks))), bound)


def test_plan_rejects_overlapping_lineage_bad_clock_and_nonchronological_windows():
  plan = primitive_plan(fitted_model())
  changes = (
    {'version': 'unknown'}, {'cutoff_ns': True}, {'cutoff_ns': 0},
    {'stage_blocks': ('0',) + plan.stage_blocks[1:]}, {'stage_blocks': plan.stage_blocks[:7]},
    {'windows': ((1, 45_000_000_000, 40_000_000_000, 40_000_000_000),)},
    {'windows': ((1, 45_000_000_000, 41_000_000_000, 0),)},
    {'windows': ((1, 45_000_000_000, 41_000_000_000, 42_000_000_000),)},
    {'windows': ((1, 45_000_000_000, 0, 0),)},
    {'windows': plan.windows + ((44_000_000_000, 46_000_000_000, 0, 0),)},
  )
  for change in changes:
    must_reject(lambda change=change: replace(plan, **change).validate())


def test_report_missing_or_extra_fields_and_foreign_fitted_model_reject():
  model = fitted_model()
  plan = primitive_plan(model)
  values = asdict(plan)
  must_reject(lambda: activation_from_dict({**values, 'candidate_gain': 1.}))
  del values['cutoff_ns']
  must_reject(lambda: activation_from_dict(values))
  must_reject(lambda: bind_activation_model(model, replace(plan, identity='e' * 64)))
  must_reject(lambda: bind_activation_model(model, replace(plan, physical_context='e' * 64)))


def test_activated_model_decode_rejects_nonfinite_physics_and_changed_version():
  model = bind_activation_model(fitted_model(), primitive_plan(fitted_model()))
  values = asdict(model)
  for field, value in (('slope', 0.), ('intercept', math.inf), ('heldout_rmse', math.nan),
                       ('activation_version', 'unknown'), ('activation_plan_sha256', ''),
                       ('calibration_cutoff_ns', True)):
    must_reject(lambda field=field, value=value: activation_model_from_dict({**values, field: value}))
