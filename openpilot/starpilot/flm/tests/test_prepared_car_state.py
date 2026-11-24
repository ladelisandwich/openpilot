"""Actual selected-controller reader equivalence; no fit/application authority."""
import math
from types import SimpleNamespace
from unittest import mock

import pytest

from opendbc.car import structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.values import CAR as HYUNDAI
from openpilot.starpilot.flm.controller_replay import (
  SelectedControllerFactory, _PreparedCarStates, _car_state, _replay_car_state, _run, capture_snapshot, frame_input,
)
from openpilot.starpilot.flm.tests.test_event_replay_cost import engine_for
from openpilot.starpilot.flm.torque_adapter import Objective


def inputs_for(vm, tick=0):
  cs = structs.CarState.new_message(vEgo=23.123456789, steeringAngleDeg=.1 * math.sin(tick / 20),
    steeringRateDeg=.00003, gearShifter='drive')
  values = frame_input(cs, vm, SimpleNamespace(angleOffsetDeg=.02, roll=.001), active=True,
    safety_limited=False, desired_curvature=.0001 * math.cos(tick / 60), curvature_limited=False, lat_delay=.2)
  return values + (('control_frame', float(tick + 1)), ('raw_model_curvature', .0001))


@pytest.mark.parametrize('domain', ('torque', 'starpilot', 'pid'))
def test_full_selected_updates_and_every_captured_state_match_fresh_builder(domain):
  if domain == 'starpilot':
    fingerprint = HYUNDAI.HYUNDAI_IONIQ_6
    cp = interfaces[fingerprint].get_non_essential_params(fingerprint)
    factory = SelectedControllerFactory(cp, 'starpilot')
  else:
    factory = engine_for(domain).factory
  actual, vm = factory.create()
  cached, cached_vm = factory.create()
  cache = _PreparedCarStates()
  objective = Objective('feedforward', 'high', 'steady')
  original_cp = factory.cp.as_builder().to_bytes()
  for tick in range(220):
    inputs = inputs_for(vm, tick)
    prepared = cache.get(inputs)
    assert prepared is not None and prepared.inputs is inputs
    expected, original_log = _run(actual, vm, inputs, objective, (0., 0.))
    observed, cached_log = _run(cached, cached_vm, inputs, objective, (0., 0.), _prepared_car_state=prepared)
    assert observed == expected and cached_log.to_dict() == original_log.to_dict()
    assert capture_snapshot(cached, cached_vm) == capture_snapshot(actual, vm)
    assert cache.get(inputs) is prepared
    assert prepared.reader.as_builder().to_bytes() == _car_state(dict(inputs)).to_bytes()
  assert factory.cp.as_builder().to_bytes() == original_cp


def test_exact_capnp_fields_enum_float32_reader_immutability_and_replacement():
  _, vm = engine_for('torque').factory.create()
  inputs = inputs_for(vm)
  cache = _PreparedCarStates()
  prepared = cache.get(inputs)
  fresh = _car_state(dict(inputs))
  assert prepared.reader.to_dict() == fresh.to_dict()
  assert prepared.reader.as_builder().to_bytes() == fresh.to_bytes()
  assert prepared.reader.total_size.word_count <= 32
  assert prepared.reader.vEgo == fresh.vEgo
  assert prepared.reader.gearShifter == fresh.gearShifter
  assert prepared.reader.total_size.word_count == fresh.total_size.word_count
  with pytest.raises(AttributeError):
    prepared.reader.vEgo = 0.
  replaced = (*inputs[:-1], ('raw_model_curvature', -.0001))
  assert cache.get(replaced) is not prepared
  assert _replay_car_state(replaced, dict(replaced), prepared).to_dict() == _car_state(dict(replaced)).to_dict()


@pytest.mark.parametrize('value', ([], [('vEgo', 20.)], (['vEgo', 20.],), (('vEgo', []),)))
def test_mutable_or_unknown_input_shapes_take_original_path(value):
  assert _PreparedCarStates().get(value) is None


def test_subclasses_fall_back_and_duplicate_key_semantics_are_unchanged():
  class Scalar(float):
    pass
  _, vm = engine_for('torque').factory.create()
  inputs = inputs_for(vm)
  cache = _PreparedCarStates()
  assert cache.get((*inputs, ('extra', Scalar(1.)))) is None
  duplicate = (*inputs, ('vEgo', 21.125))
  assert cache.get(duplicate).reader.vEgo == _car_state(dict(duplicate)).vEgo == 21.125


def test_cache_is_strongly_retained_and_bounded_to_twelve_thousand_inputs():
  cache = _PreparedCarStates()
  with mock.patch('openpilot.starpilot.flm.controller_replay._car_state') as build:
    for tick in range(12000):
      inputs = (('tick', tick),)
      assert cache.get(inputs).inputs is inputs
    assert cache.get((('tick', 12000),)) is None
    assert build.call_count == 12000
  assert len(cache._PreparedCarStates__entries) == 12000


def test_default_call_still_constructs_fresh_car_state_every_time():
  _, vm = engine_for('torque').factory.create()
  inputs = inputs_for(vm)
  with mock.patch('openpilot.starpilot.flm.controller_replay._car_state', wraps=_car_state) as build:
    a = _replay_car_state(inputs, dict(inputs), None)
    b = _replay_car_state(inputs, dict(inputs), None)
  assert build.call_count == 2 and a is not b and a.to_dict() == b.to_dict()
