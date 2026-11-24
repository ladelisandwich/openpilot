from dataclasses import replace
from copy import deepcopy
from types import SimpleNamespace
from unittest import mock
import pytest

from openpilot.cereal import messaging
from openpilot.starpilot.flm.effect_admission_cache import validated_envelope, admit_context
from openpilot.starpilot.flm.global_runtime import GlobalRuntime, SERVICE
from openpilot.starpilot.flm.adaptive_profile import Result
from openpilot.starpilot.flm.learning_wire import result_message
from openpilot.starpilot.flm.tests.test_selected_effect_owner import fixture, NOW


def test_verified_envelope_cache_never_accepts_changed_profile_or_mutated_content():
  _, p, _, context = fixture()
  envelope = deepcopy(p.effect_envelope)
  validated_envelope(envelope, p)
  with pytest.raises(ValueError):
    validated_envelope(envelope, replace(p, coefficients=(0.005,)))
  object.__setattr__(envelope, 'full_profile_sha256', '0' * 64)
  with pytest.raises(ValueError):
    validated_envelope(envelope, p)
  # The original independently frozen proof/key was never modified.
  assert admit_context(p.effect_envelope, profile=p, **context).full_profile_sha256 == p.effect_envelope.full_profile_sha256


@pytest.mark.parametrize(
  'field,value',
  [
    ('source_epoch', '7' * 64),
    ('reset_generation', '1'),
    ('cp_sha256', '7' * 64),
    ('controller_revision', 'changed'),
    ('selected_baseline_revision', '7' * 64),
    ('context_key', '7' * 64),
    ('current_full_profile_sha256', '7' * 64),
    ('persistent_source_epoch', '7' * 64),
  ],
)
def test_every_current_context_field_is_compared_even_after_cache_hit(field, value):
  _, p, _, context = fixture()
  admit_context(p.effect_envelope, profile=p, **context)
  with pytest.raises(ValueError):
    admit_context(p.effect_envelope, profile=p, **(context | {field: value}))


def test_mutable_nested_map_or_boolean_coordinate_cannot_alias_valid_cache_entry():
  _, p, _, _ = fixture()
  env = deepcopy(p.effect_envelope)
  validated_envelope(env, p)
  rows = list(env.prior_coordinates)
  object.__setattr__(env, 'prior_coordinates', rows)
  with pytest.raises(ValueError):
    validated_envelope(env, p)
  env = deepcopy(p.effect_envelope)
  rows = list(env.prior_coordinates)
  rows[0] = (rows[0][0], False)
  object.__setattr__(env, 'prior_coordinates', tuple(rows))
  with pytest.raises(ValueError):
    validated_envelope(env, p)


def test_actual_submaster_parse_cache_invalidates_on_each_new_delivery():
  _, p, v, _ = fixture()
  runtime = GlobalRuntime.__new__(GlobalRuntime)
  runtime._runtime_admission_cache = None
  sm = messaging.SubMaster([SERVICE])
  packet = result_message(Result('qualified', 'test', p), 'session', 1, NOW, replay_validation=v)
  sm.update_msgs(NOW / 1e9, [packet.as_reader()])
  from openpilot.starpilot.flm import global_runtime

  original = global_runtime.read_runtime_admission
  with mock.patch.object(global_runtime, 'read_runtime_admission', wraps=original) as read:
    event = SimpleNamespace(valid=True, autoSteeringLearning=sm[SERVICE])
    first = runtime._current_runtime_admission(sm, event)
    assert first is not None
    assert runtime._current_runtime_admission(sm, event) == first
    assert read.call_count == 1
    # Deliberately changed coefficient without matching envelope is invalid.
    packet2 = packet.as_reader().as_builder()
    packet2.autoSteeringLearning.learning.runtimeProfile.coefficients = [0.005]
    packet2.logMonoTime = NOW + 1
    sm.update_msgs((NOW + 1) / 1e9, [packet2.as_reader()])
    changed = SimpleNamespace(valid=True, autoSteeringLearning=sm[SERVICE])
    assert runtime._current_runtime_admission(sm, changed) is None
    assert read.call_count == 2
