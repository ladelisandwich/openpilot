"""Exact leaf speedup equivalence, including all original generic fallbacks."""
from collections import deque
import math
import pickle
from unittest import mock

import numpy as np
import pytest

from openpilot.starpilot.flm import controller_replay as replay
from openpilot.starpilot.flm.selected_effects import EffectReplayRecordedSource
from openpilot.starpilot.flm.tests.test_event_controller_prefix import fixture


def original_tree(values, path, value):
  # Original complete recursive traversal, without calling the proposed tree.
  if type(value).__module__.startswith('capnp') and 'List' in type(value).__name__:
    value = list(value)
  if isinstance(value, (tuple, list, deque)):
    values.append((path + '.kind', float(3 if isinstance(value, deque) else 2 if isinstance(value, tuple) else 1)))
    values.append((path + '.length', float(len(value))))
    if isinstance(value, deque):
      values.append((path + '.maxlen', float(value.maxlen or 0)))
    for i, item in enumerate(value):
      original_tree(values, path + '.' + str(i), item)
  else:
    values.append((path, replay._numeric(value)))
  if len(values) > replay.MAX_SNAPSHOT_VALUES:
    raise ValueError('snapshot exceeds bounded primitive budget')


def outcome(function, value, initial=()):
  values = list(initial)
  error = None
  try:
    function(values, 'state', value)
  except (TypeError, ValueError, OverflowError, AttributeError) as exc:
    error = (type(exc), str(exc))
  return pickle.dumps(values, protocol=5), error


@pytest.mark.parametrize('value', (0., -0., .12345678901234567, 1, True, False, -(2 ** 100), 2 ** 10000,
                                   float('nan'), float('inf'), -float('inf'), 'invalid', None,
                                   (1., [False, -0., .3]), deque([1., 2., (3.,)], maxlen=6),
                                   np.float64(.123), np.float32(-.5), np.int64(3), np.bool_(True), np.float64('nan')))
@pytest.mark.parametrize('size', (0, 1023, 1024))
def test_every_leaf_fallback_order_floatbits_error_and_max_bound_match_original(value, size):
  initial = tuple((str(i), 1.) for i in range(size))
  assert outcome(replay._tree, value, initial) == outcome(original_tree, value, initial)


def test_subclass_coercion_and_native_capnp_pid_list_stay_original_fallback():
  class Float(float):
    calls = 0
    def __float__(self):
      type(self).calls += 1
      return .25
  value = Float(1.)
  expected = outcome(original_tree, value)
  assert Float.calls == 1
  Float.calls = 0
  assert outcome(replay._tree, value) == expected and Float.calls == 1
  class Tuple(tuple):
    pass
  assert outcome(replay._tree, Tuple((.1, -0.))) == outcome(original_tree, Tuple((.1, -0.)))
  from opendbc.car import structs
  cp = structs.CarParams.new_message()
  pid = cp.lateralTuning.init('pid')
  pid.kpBP = [0., 5., 20.]
  assert outcome(replay._tree, pid.kpBP) == outcome(original_tree, pid.kpBP)


@pytest.mark.parametrize('domain', ('torque', 'starpilot', 'pid'))
def test_actual_warm_selected_snapshot_every_value_and_hidden_invalid_state_match_original(domain):
  engine = fixture(domain, nonzero=True)
  controller, vm = engine.factory.create()
  replay.restore_snapshot(controller, vm, engine.checkpoints[0][1])
  for frame in engine.records:
    value, _ = replay._run(controller, vm, frame.inputs, engine.engine.objective, (0., 0.), frame.torque_values,
      actual_stage_residual=frame.actual_stage_residual, effect_trial=EffectReplayRecordedSource(frame.effect_trace))
    assert replay.same_float(value, frame.actual_lac_command)
  proposed = replay.capture_snapshot(controller, vm)
  with mock.patch.object(replay, '_tree', original_tree):
    before = replay.capture_snapshot(controller, vm)
  assert pickle.dumps(proposed, protocol=5) == pickle.dumps(before, protocol=5)
  controller.pid.i = math.nan
  for function in (replay._tree, original_tree):
    with mock.patch.object(replay, '_tree', function), pytest.raises(ValueError, match='finite numeric'):
      replay.capture_snapshot(controller, vm)


@pytest.mark.parametrize('domain', ('torque', 'starpilot'))
def test_actual_manual_surface_and_duplicate_paths_guards_are_unchanged(domain):
  engine = fixture(domain)
  controller, vm = engine.factory.create()
  controller.flm_surface = object()
  for function in (replay._tree, original_tree):
    with mock.patch.object(replay, '_tree', function), pytest.raises(ValueError, match='manual surface'):
      replay.capture_snapshot(controller, vm)
  controller.flm_surface = None
  controller.pid.i = None
  setattr(controller.pid, 'i.none', 1.)
  for function in (replay._tree, original_tree):
    with mock.patch.object(replay, '_tree', function), pytest.raises(ValueError, match='cardinality'):
      replay.capture_snapshot(controller, vm)


def test_custom_metaclass_equality_cannot_impersonate_exact_builtin_leaf():
  class EqualFloat(type):
    def __eq__(cls, other):
      return other is float
    __hash__ = type.__hash__
  class NotNumeric(metaclass=EqualFloat):
    def __float__(self):
      return .25
  value = NotNumeric()
  assert type(value) == float  # noqa: E721 - exercise adversarial class equality, not type identity.
  assert type(value) is not float
  assert outcome(replay._tree, value) == outcome(original_tree, value)
  assert outcome(replay._tree, value)[1] == (ValueError, 'snapshot requires finite numeric state')
