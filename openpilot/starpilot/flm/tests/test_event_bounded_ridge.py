"""Bounded train proposal; no native permission or heldout-selected direction."""
from dataclasses import asdict, replace
from unittest import mock

import pytest

from openpilot.starpilot.flm.adaptive_profile import Parameter, evaluate, loss
from openpilot.starpilot.flm.event_bounded_ridge import bounded_ridge
from openpilot.starpilot.flm.event_evaluate import evaluate_event
from openpilot.starpilot.flm.tests.test_event_native_backtrack import problem


def parameter(low=-1., high=1., max_step=1.):
  return Parameter('test', 'test', low, high, max_step, .005)


def test_big_error_legacy_rejects_but_event_computes_actual_bounded_improvement():
  args, kwargs = problem()
  blocks = tuple(replace(block, samples=tuple(replace(row, residual_target=10.) for row in block.samples)) for block in args[2])
  before = tuple(asdict(p) for p in args[1]), blocks, asdict(kwargs['effect_envelope'])
  legacy = evaluate(args[0], args[1], blocks, **{k: kwargs[k] for k in ('now_ns', 'required_cohorts', 'response_revision', 'prior')})
  assert legacy.reason == 'coordinate_envelope' and legacy.profile is None
  native = mock.Mock(return_value=True)
  result = evaluate_event(args[0], args[1], blocks, **kwargs, native_feasible=native)
  assert result.profile is not None and result.profile.coefficients == (.005,)
  native.assert_called_once_with((.005,))
  held = tuple(row for block in blocks[4:] for row in block.samples)
  assert result.profile.candidate_loss == loss(held, (.005,)) < result.profile.baseline_loss == loss(held, (0.,))
  assert (tuple(asdict(p) for p in args[1]), blocks, asdict(kwargs['effect_envelope'])) == before
  assert result.reason == 'event_tiny_trial_pending_actual_replay'  # Not installed/admitted/current authority.


def test_box_solver_reoptimizes_coupled_free_coordinate_and_satisfies_kkt():
  matrix, rhs = ((2., 1.), (1., 2.)), (2., 0.)
  result = bounded_ridge(matrix, rhs, (parameter(), parameter()), (0., 0.), .5)
  assert result == (.5, -.25)
  gradient = tuple(sum(a*b for a, b in zip(row, result, strict=True))-r for row, r in zip(matrix, rhs, strict=True))
  assert gradient[0] < 0 and gradient[1] == 0  # Upper active / free KKT.
  assert result != (.5, -.5)  # Componentwise clamp of unconstrained (4/3,-2/3).


@pytest.mark.parametrize('sign', (-1., 1.))
def test_actual_domain_max_step_and_nonzero_prior_intersection(sign):
  p = parameter(-.02, .02, .012)
  result = bounded_ridge(((1.,),), (100.*sign,), (p,), (.01*sign,), .005)
  assert result == (.012*sign,)
  assert abs(result[0]-.01*sign) <= .005 and abs(result[0]) <= p.max_step
  p = parameter(0., .003, .25)
  assert bounded_ridge(((1.,),), (100.,), (p,), (0.,), .005) == (.003,)


def test_train_direction_is_identical_on_adversarial_heldout_and_no_native_call():
  args, kwargs = problem()
  train_big = tuple(replace(b, samples=tuple(replace(r, residual_target=10.) for r in b.samples)) for b in args[2])
  bad = tuple(replace(b, samples=tuple(replace(r, residual_target=-10.) for r in b.samples)) if i >= 4 else b
              for i, b in enumerate(train_big))
  from openpilot.starpilot.flm import event_bounded_ridge
  original = event_bounded_ridge.bounded_ridge
  selected = []
  def observe(*a):
    result = original(*a)
    selected.append(result)
    return result
  native = mock.Mock(return_value=True)
  with mock.patch.object(event_bounded_ridge, 'bounded_ridge', side_effect=observe):
    good = evaluate_event(args[0], args[1], train_big, **kwargs)
    rejected = evaluate_event(args[0], args[1], bad, **kwargs, native_feasible=native)
  assert selected == [(.005,), (.005,)] and good.profile is not None
  assert rejected.profile is None and rejected.reason == 'heldout_cohort_regression_or_noise'
  native.assert_not_called()


@pytest.mark.parametrize('defect', ('limited', 'duplicate', 'unexcited', 'confounded', 'wrong_prior', 'nan'))
def test_original_evidence_excitation_and_contract_gates_remain(defect):
  args, kwargs = problem()
  parameters, blocks = args[1], args[2]
  if defect in ('limited', 'nan', 'unexcited'):
    rows = tuple(replace(row, **({'native_allowed': False} if defect == 'limited' else
                                {'features': (float('nan'),)} if defect == 'nan' else {'features': (0.,)}))
                 for row in blocks[0].samples)
    blocks = (replace(blocks[0], samples=rows), *blocks[1:])
    if defect == 'unexcited':
      blocks = tuple(replace(b, samples=tuple(replace(r, features=(0.,)) for r in b.samples)) for b in blocks)
  elif defect == 'duplicate':
    blocks = (blocks[0], replace(blocks[1], block_id=blocks[0].block_id), *blocks[2:])
  elif defect == 'wrong_prior':
    kwargs = {**kwargs, 'prior': (True,)}
  else:
    # Excitation floor remains mandatory even when a constrained optimum exists.
    parameters = tuple(replace(p, excitation_floor=2.) for p in parameters)
  native = mock.Mock(return_value=True)
  result = evaluate_event(args[0], parameters, blocks, **kwargs, native_feasible=native)
  assert result.profile is None
  native.assert_not_called()


@pytest.mark.parametrize('matrix,rhs,centre,step', (
  (((float('nan'),),), (1.,), (0.,), .005),
  (((1.,),), (float('inf'),), (0.,), .005),
  (((1.,),), (1.,), (True,), .005),
  (((1.,),), (1.,), (0.,), True),
  (((1.,),), (1.,), (0.,), -.005),
))
def test_nonfinite_and_boolean_solver_inputs_fail_closed(matrix, rhs, centre, step):
  with pytest.raises(ValueError):
    bounded_ridge(matrix, rhs, (parameter(),), centre, step)


def test_all_four_dimensions_and_fixed_training_only_bounded_domain():
  matrix = tuple(tuple(2. if i == j else 0. for j in range(4)) for i in range(4))
  assert bounded_ridge(matrix, (10., -10., .002, -.002), (parameter(),)*4, (0.,)*4, .005) == (.005, -.005, .001, -.001)
