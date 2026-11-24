"""Bound primitive validation work without changing the worker wire."""
import math
import pickle
import zlib
from types import SimpleNamespace

import pytest

from openpilot.starpilot.flm import controller_replay as wire


def test_shared_primitives_keep_exact_wire_types_and_signed_zero():
  row = {'negative_zero': -0., 'integer': 1, 'boolean': True, 'bytes': b'value'}
  value = {'rows': [row] * 8192}
  encoded = wire.pack_primitive_snapshot(value)
  assert encoded == zlib.compress(pickle.dumps(value, protocol=5), 1)
  restored = wire.unpack_primitive_snapshot(encoded)['rows']
  assert len(restored) == 8192 and restored[0] is restored[-1]
  assert math.copysign(1., restored[0]['negative_zero']) == -1.
  assert type(restored[0]['integer']) is int and type(restored[0]['boolean']) is bool
  assert restored[0]['bytes'] == b'value'


def test_shared_container_validation_is_bounded(monkeypatch):
  checked = []

  def finite(value):
    checked.append(value)
    return math.isfinite(value)

  monkeypatch.setattr(wire, 'math', SimpleNamespace(isfinite=finite))
  row = {'value': .25}
  wire.pack_primitive_snapshot({'rows': [row] * 8192})
  assert checked == [.25]


@pytest.mark.parametrize('bad', (object(), {'value': float('nan')}, {'value': float('inf')}, {1: True}))
def test_invalid_shared_data_still_rejects(bad):
  with pytest.raises(ValueError):
    wire.pack_primitive_snapshot([bad, bad])


def test_cycle_rejects_without_recursing_to_the_interpreter_limit():
  value = []
  value.append(value)
  with pytest.raises(ValueError, match='cyclic fit job primitive'):
    wire.pack_primitive_snapshot(value)


@pytest.mark.parametrize('limit,message', (('MAX_FIT_RAW_BYTES', 'raw byte bound'), ('MAX_FIT_SNAPSHOT_BYTES', 'IPC byte bound')))
def test_original_wire_size_limits_remain_enforced(monkeypatch, limit, message):
  monkeypatch.setattr(wire, limit, 1)
  with pytest.raises(ValueError, match=message):
    wire.pack_primitive_snapshot({'value': b'payload'})
