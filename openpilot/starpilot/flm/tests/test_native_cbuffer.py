"""ROOT ONLY: actual rebuilt source-bound extension; author did not import/run.
No fixture substitutes the native serializer or grants learning authority.
"""
import hashlib
import importlib
import json
import math
import sys
from typing import cast

import pytest


def typed(value):
  kind = type(value)
  if value is None or kind in (bool, int, str):
    return [kind.__name__, value]
  if kind is float:
    return ['float64', value.hex()]
  if kind is bytes:
    return ['bytes', value.hex()]
  if kind in (tuple, list):
    return [kind.__name__, [typed(child) for child in value]]
  return ['dict', [[key, typed(value[key])] for key in sorted(value)]]


def oracle(value):
  return hashlib.sha256(json.dumps(typed(value), separators=(',', ':'), allow_nan=False).encode()).hexdigest()


@pytest.fixture
def native():
  return importlib.import_module('openpilot.starpilot.flm._primitive_walk')


def large(child):
  return {'rows': [child for _ in range(257)], 'version': 'test-only-byte-oracle'}


def test_complete_typed_alphabet_and_exact_ordering(native):
  value = large({'z': [None, True, False, 0, 1, -1, 2**511, -(2**511), 0.0, -0.0,
                       math.ldexp(1.0, -1074), sys.float_info.max, b'\x00\xff', (), [], {}],
                 'a': ('quote"', 'slash\\', '\n\r\t\b\f', '\x00\x1f', 'é', '\u2028',
                       '\U0001f680', '\ud800', '\udc00', '\ud800\udc00')})
  assert native.whole_body_digest(value) == oracle(value)
  assert native.whole_body_digest(large([True, 1])) != native.whole_body_digest(large([1, True]))
  assert native.whole_body_digest(large([0.0])) != native.whole_body_digest(large([-0.0]))
  assert native.whole_body_digest(large((1, 2))) != native.whole_body_digest(large([1, 2]))
  assert native.whole_body_digest(large({'b': 2, 'a': 1})) == oracle(large({'a': 1, 'b': 2}))


def test_fresh_call_mutable_alias_and_changed_body(native):
  shared = {'cp': bytes(range(256)) * 8, 'revision': ['a', 0.0], 'config': {'enabled': True}}
  value = large(shared)
  first = native.whole_body_digest(value)
  assert first == oracle(value)
  shared['revision'][1] = -0.0
  shared['config']['enabled'] = False
  shared['cp'] += b'changed'
  second = native.whole_body_digest(value)
  assert second == oracle(value) and second != first
  value['rows'][128] = {'new': ('distinct', 4)}
  assert native.whole_body_digest(value) == oracle(value)


@pytest.mark.parametrize('length', [5460, 5461, 5462, 32761, 32762, 32763])
def test_scalar_entry_boundary_unicode_bytes(native, length):
  # Crosses conservative Unicode and exact byte-token64KiB entry thresholds.
  child = {'s': '\U0001f680' * length, 'b': b'\xff' * length}
  value = {'rows': [child, child] + [None] * 255}
  assert native.whole_body_digest(value) == oracle(value)


def test_split_unicode_escape_boundaries(native):
  value = large({'s': 'x' * 2047 + '\U0001f680' + '\ud800' + '\udc00' + '\\"\n' + 'z' * 8200})
  assert native.whole_body_digest(value) == oracle(value)


def test_large_int_encoder_and_digit_limit(native):
  value = large({'n': 10**600, 'm': -(10**600)})
  assert native.whole_body_digest(value) == oracle(value)
  before = sys.get_int_max_str_digits()
  try:
    sys.set_int_max_str_digits(640)
    value = large({'n': 10**700})
    with pytest.raises(ValueError):
      oracle(value)
    with pytest.raises(ValueError):
      native.whole_body_digest(value)
  finally:
    sys.set_int_max_str_digits(before)


def test_whole_graph_prevalidation_and_no_callbacks(native):
  class Trap:
    def __repr__(self):
      raise AssertionError('callback ran')
  class DictSubclass(dict):
    def items(self):
      raise AssertionError('subclass callback ran')
  for bad in [Trap(), DictSubclass(), {1: 'nonstring key'}, float('nan'), float('inf')]:
    assert native.whole_body_digest(large(bad)) is native.FALLBACK
  cycle = []
  cycle.append(cycle)
  assert native.whole_body_digest(large(cycle)) is native.FALLBACK
  deep = 0
  for _ in range(34):
    deep = [deep]
  assert native.whole_body_digest(large(deep)) is native.FALLBACK


def test_no_cache_state_between_calls_after_failed_prevalidation(native):
  shared = {'x': b'original' * 128}
  value = large(shared)
  expected = native.whole_body_digest(value)
  shared['bad'] = object()
  assert native.whole_body_digest(value) is native.FALLBACK
  del shared['bad']
  assert native.whole_body_digest(value) == expected == oracle(value)



def test_every_ascii_and_unicode_escape_class(native):
  value = large(''.join(chr(i) for i in range(256)) + '\U00010000\U0010ffff\ud800\udfff')
  assert native.whole_body_digest(value) == oracle(value)


def test_native_buffer_boundaries_and_maximum_update(native, monkeypatch):
  value = {'rows': [b'\x00\x7f\xff' * 21900, 'x' * 65535, 'q' * 65536,
                    '\u007f' * 10923, '\U0001f680' * 5462] + [None] * 257}
  expected = oracle(value)
  real_sha = hashlib.sha256
  updates = []
  class RecordingDigest:
    def __init__(self):
      self.inner = real_sha()
    def update(self, chunk):
      updates.append(chunk)
      self.inner.update(chunk)
    def hexdigest(self):
      return self.inner.hexdigest()
  monkeypatch.setattr(hashlib, 'sha256', RecordingDigest)
  assert native.whole_body_digest(value) == expected
  assert max(map(len, updates)) <= 65536
  assert b''.join(updates) == json.dumps(typed(value), separators=(',', ':'), allow_nan=False).encode()


def test_small_path_still_original_encoder(native, monkeypatch):
  real_dumps = json.dumps
  calls = []
  def recording(value, *args, **kwargs):
    calls.append(value)
    return real_dumps(value, *args, **kwargs)
  monkeypatch.setattr(json, 'dumps', recording)
  assert native.whole_body_digest({'x': -0.0}) == oracle({'x': -0.0})
  assert len(calls) == 2


@pytest.mark.parametrize('key', [1, None, True, 1.5, ('tuple', 1)])
def test_post_prepass_nonstring_dict_key_matches_safe_original_json(native, monkeypatch, key):
  # Authoritative old algorithm also accepts this post-prepass mutation: the
  # dict key occupies a raw JSON pair slot, rather than a typed leaf slot.
  # Root must run against original and successor SOs, proving equal digests.
  value = [b'x' * 40000] + [None] * 255 + [{'marker': 'unchanged'}]
  real_sha = hashlib.sha256
  changed = False
  class ReentrantDigest:
    def __init__(self):
      self.inner = real_sha()
    def update(self, chunk):
      nonlocal changed
      if not changed:
        cast(dict[object, str], value[-1]).clear()
        cast(dict[object, str], value[-1])[key] = 'injected'
        changed = True
      self.inner.update(chunk)
    def hexdigest(self):
      return self.inner.hexdigest()
  monkeypatch.setattr(hashlib, 'sha256', ReentrantDigest)
  result = native.whole_body_digest(value)
  assert changed
  # Use the real digest because expected bytes are formed after mutation.
  assert result == real_sha(json.dumps(typed(value), separators=(',', ':'), allow_nan=False).encode()).hexdigest()
