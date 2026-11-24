"""Replay-local canonical CP reuse rereads native bytes; no live/global authority."""
from dataclasses import replace
from types import SimpleNamespace
from unittest import mock

import pytest

from opendbc.car import structs
from openpilot.starpilot.flm import runtime_adapter_registry as registry
from openpilot.starpilot.flm.tests.test_event_controller_prefix import fixture


def changed(engine, update):
  cp = engine.factory.cp.as_builder()
  update(cp)
  engine.factory.cp = cp.as_reader()


@pytest.mark.parametrize('domain', ('torque', 'starpilot', 'pid'))
def test_actual_owned_cp_hits_only_one_entry_and_every_lookup_reads_bytes(domain):
  engine = fixture(domain).engine
  original = registry.car_params_digest
  with mock.patch.object(registry, 'car_params_digest', wraps=original) as canonical:
    assert engine._recorded_cp_digest() == original(engine.factory.cp)
    saved = engine.__dict__['_ControllerReplay__recorded_cp_digest_cache']
    for _ in range(5):
      assert engine._recorded_cp_digest() == saved[1]
    assert canonical.call_count == 1
    changed(engine, lambda cp: setattr(cp, 'carVin', '1HGBH41JXMN109186'))
    assert engine._recorded_cp_digest() == original(engine.factory.cp)
    assert canonical.call_count == 2
    assert engine.__dict__['_ControllerReplay__recorded_cp_digest_cache'][0] != saved[0]


@pytest.mark.parametrize('field,value', (('mass', 0.), ('mass', -0.), ('steerLimitTimer', float('inf')),
                                        ('steerLimitTimer', -float('inf')), ('carVin', 'new-source')))
def test_actual_scalar_bits_infinity_and_source_changes_match_original(field, value):
  engine = fixture().engine
  before = engine._recorded_cp_digest()
  changed(engine, lambda cp: setattr(cp, field, value))
  assert engine._recorded_cp_digest() == registry.car_params_digest(engine.factory.cp)
  assert engine._recorded_cp_digest() != before
  if field == 'mass':
    plus = engine._recorded_cp_digest()
    changed(engine, lambda cp: setattr(cp, 'mass', -value))
    # Test both directions of native Float32 signed zero identity.
    assert engine._recorded_cp_digest() != plus


def test_actual_safety_config_native_bits_are_not_hidden_by_cache():
  engine = fixture().engine
  before = engine._recorded_cp_digest()
  cp = engine.factory.cp.as_builder()
  assert len(cp.safetyConfigs)
  cp.safetyConfigs[0].safetyParam ^= 1
  engine.factory.cp = cp.as_reader()
  assert engine._recorded_cp_digest() == registry.car_params_digest(engine.factory.cp)
  assert engine._recorded_cp_digest() != before


@pytest.mark.parametrize('wrong', ('schema', 'object', 'factory'))
def test_unknown_or_wrong_schema_keeps_original_digest_each_time(wrong):
  engine = fixture().engine
  if wrong == 'schema':
    engine.factory.cp = structs.CarState.new_message().as_reader()
  elif wrong == 'object':
    engine.factory.cp = SimpleNamespace(to_dict=lambda: {'negativeZero': -0., 'sentinel': float('inf')})
  else:
    engine.factory = SimpleNamespace(cp=engine.factory.cp)
  original = registry.car_params_digest
  with mock.patch.object(registry, 'car_params_digest', wraps=original) as canonical:
    expected = original(engine.factory.cp)
    assert engine._recorded_cp_digest() == engine._recorded_cp_digest() == expected
    assert canonical.call_count == 2
  assert '_ControllerReplay__recorded_cp_digest_cache' not in engine.__dict__


@pytest.mark.parametrize('failure', ('serialization', 'changed_during_digest', 'canonical_error'))
def test_serialization_failure_and_unstable_pair_never_cache_errors_or_retry_original(failure):
  engine = fixture().engine
  original = registry.car_params_digest
  real = engine.factory.cp
  class Reader:
    schema = structs.CarParams.schema
    def __init__(self):
      self.calls = 0
    def to_dict(self):
      return real.to_dict()
    def as_builder(self):
      self.calls += 1
      if failure == 'serialization':
        raise ValueError('serializer unavailable')
      raw = b'before' if self.calls == 1 else b'after'
      return SimpleNamespace(to_bytes=lambda: raw)
  engine.factory.cp = Reader()
  with mock.patch('capnp.lib.capnp._DynamicStructReader', Reader):
    if failure == 'canonical_error':
      with mock.patch.object(registry, 'car_params_digest', side_effect=ValueError('original digest error')) as canonical:
        with pytest.raises(ValueError, match='original digest error'):
          engine._recorded_cp_digest()
        assert canonical.call_count == 1
    else:
      with mock.patch.object(registry, 'car_params_digest', wraps=original) as canonical:
        assert engine._recorded_cp_digest() == original(real)
        assert canonical.call_count == 1
  assert '_ControllerReplay__recorded_cp_digest_cache' not in engine.__dict__


@pytest.mark.parametrize('domain', ('torque', 'starpilot', 'pid'))
def test_actual_normalize_trace_cp_binding_uses_cache_without_losing_any_recent_states(domain):
  engine = fixture(domain).engine
  cp_digest = registry.car_params_digest(engine.factory.cp)
  frames = tuple(engine.records)
  engine.records.clear()
  original = registry.car_params_digest
  with mock.patch.object(registry, 'car_params_digest', wraps=original) as canonical:
    for frame in frames:
      trace = frame.effect_trace
      trace = replace(trace, envelope=replace(trace.envelope, cp_sha256=cp_digest).validate()).validate()
      assert engine.normalize(replace(frame, effect_trace=trace), _verify_only=True) is not None
    assert canonical.call_count == 1
  assert len(engine.recent_states) == len(frames) == len(engine.records) == 60
  assert not engine.failed


def test_mutation_through_builder_changes_same_reader_identity_and_recomputes():
  engine = fixture().engine
  builder = engine.factory.cp.as_builder()
  reader = builder.as_reader()
  engine.factory.cp = reader
  before = engine._recorded_cp_digest()
  builder.carVin = 'same-reader-new-native-bytes'
  assert engine.factory.cp is reader
  assert engine._recorded_cp_digest() == registry.car_params_digest(reader)
  assert engine._recorded_cp_digest() != before


def test_same_schema_node_id_without_full_schema_equality_does_not_cache():
  engine = fixture().engine
  real = engine.factory.cp
  class Schema:
    node = SimpleNamespace(id=structs.CarParams.schema.node.id)
    def __eq__(self, other):
      return False
  class Reader:
    schema = Schema()
    def to_dict(self):
      return real.to_dict()
    def as_builder(self):
      raise AssertionError('wrong schema must not be serialized for a cache hit')
  engine.factory.cp = Reader()
  original = registry.car_params_digest
  with mock.patch('capnp.lib.capnp._DynamicStructReader', Reader), \
       mock.patch.object(registry, 'car_params_digest', wraps=original) as canonical:
    assert engine._recorded_cp_digest() == engine._recorded_cp_digest() == original(real)
    assert canonical.call_count == 2
  assert '_ControllerReplay__recorded_cp_digest_cache' not in engine.__dict__


def test_malformed_original_result_is_not_cached_as_a_success():
  engine = fixture().engine
  with mock.patch.object(registry, 'car_params_digest', return_value=None) as canonical:
    assert engine._recorded_cp_digest() is None and engine._recorded_cp_digest() is None
    assert canonical.call_count == 2
  assert '_ControllerReplay__recorded_cp_digest_cache' not in engine.__dict__


def test_replacing_canonical_function_invalidates_cache_without_any_data_identity_shortcut():
  engine = fixture().engine
  original = registry.car_params_digest
  assert engine._recorded_cp_digest() == original(engine.factory.cp)
  with mock.patch.object(registry, 'car_params_digest', wraps=original) as replacement:
    assert engine._recorded_cp_digest() == original(engine.factory.cp)
    assert engine._recorded_cp_digest() == original(engine.factory.cp)
    assert replacement.call_count == 1
  saved = engine.__dict__['_ControllerReplay__recorded_cp_digest_cache']
  assert saved[2] is replacement
  engine._recorded_cp_digest()
  assert engine.__dict__['_ControllerReplay__recorded_cp_digest_cache'][2] is original
