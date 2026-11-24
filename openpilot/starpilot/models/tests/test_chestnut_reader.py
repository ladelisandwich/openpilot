"""Actual reader ASTs and real pickle envelopes; fake host storage, no devices."""
import ast
import io
from pathlib import Path
import pickle
import struct
import sys
import tempfile
from types import ModuleType, SimpleNamespace
from typing import cast
import unittest
from unittest.mock import patch


class DType:
  def __init__(self, name, itemsize):
    self.name, self.itemsize = name, itemsize


UCHAR, UINT32 = DType('uchar', 1), DType('uint32', 4)


class Storage:
  def __init__(self):
    self.arenas, self.copies, self.source_views = [], [], []

  def buffer(self, device, size, dtype, *, preallocate=False):
    value = View(bytearray(size * dtype.itemsize), size, dtype, 0, self, 'arena')
    self.arenas.append((device, size, dtype, preallocate, value))
    return value

  def tensor(self, path):
    owner = self

    class FileTensor:
      def __getitem__(self, key):
        if key.stop is not None or key.step is not None:
          raise AssertionError('Unexpected file slice')
        raw = bytearray(path.read_bytes()[key.start:])
        buffer = View(raw, len(raw), UCHAR, 0, owner, 'source')
        return SimpleNamespace(uop=SimpleNamespace(buffer=buffer))

    return FileTensor()


class View:
  def __init__(self, storage, size, dtype, offset, owner, kind):
    self.storage, self.size, self.dtype, self.offset = storage, size, dtype, offset
    self.owner, self.kind = owner, kind
    self.nbytes = size * dtype.itemsize

  def ensure_allocated(self):
    return self

  def view(self, size, dtype, offset):
    if offset < 0 or offset + size * dtype.itemsize > self.nbytes:
      raise AssertionError('Out of bounds fixture view')
    result = View(self.storage, size, dtype, self.offset + offset, self.owner, self.kind)
    if self.kind == 'source':
      self.owner.source_views.append((size, dtype.name, result.offset))
    return result

  def copy_from(self, source):
    if self.nbytes != source.nbytes:
      raise AssertionError('Copy size mismatch')
    self.owner.copies.append((self.offset, source.offset, self.nbytes, self.dtype.name, source.dtype.name))
    self.storage[self.offset:self.offset + self.nbytes] = source.storage[source.offset:source.offset + source.nbytes]
    return self


def reader(path, name):
  node = next(item for item in ast.parse(path.read_text()).body if isinstance(item, ast.FunctionDef) and item.name == name)
  before = ast.dump(node, include_attributes=False)
  namespace = {'Path': Path, 'pathlib': SimpleNamespace(Path=Path), 'io': io, 'pickle': pickle, 'struct': struct, 'cast': cast}
  exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), namespace)
  assert ast.dump(node, include_attributes=False) == before
  return namespace[name]


def envelope(path, payload):
  markers = (object(), object(), object())
  records = ((32, UINT32, (8 << 20) - 48), (19, UCHAR, len(payload) - 19), (32, UINT32, (8 << 20) - 48))

  class OOBPickler(pickle.Pickler):
    def persistent_id(self, value):
      for marker, record in zip(markers, records, strict=True):
        if value is marker:
          return record
      return None

  stream = io.BytesIO()
  OOBPickler(stream).dump({'views': markers, 'metadata': {'abi': 'fixture', 'shape': (3, 7)}})
  opcodes = stream.getvalue()
  path.write_bytes(struct.pack('<q', len(opcodes)) + opcodes + payload)
  return opcodes, records


def invoke(function, path, legacy=False):
  owner = Storage()
  tinygrad, device, dtype = (ModuleType(name) for name in ('tinygrad', 'tinygrad.device', 'tinygrad.dtype'))
  tinygrad.Device, tinygrad.Tensor = SimpleNamespace(DEFAULT='CPU:LLVM'), owner.tensor
  device.Buffer, dtype.dtypes = owner.buffer, SimpleNamespace(uchar=UCHAR)
  saved = dict(sys.modules)
  with patch.dict(sys.modules, {'tinygrad': tinygrad, 'tinygrad.device': device, 'tinygrad.dtype': dtype}):
    # Legacy loader imports these names at module level; supply only the same
    # fake metadata globals to its unchanged extracted function.
    globals_ = function.__globals__
    with patch.dict(globals_, {'Device': tinygrad.Device, 'Tensor': tinygrad.Tensor,
                              'Buffer': owner.buffer, 'dtypes': dtype.dtypes}):
      result = function(path, out_of_band=True) if legacy else function(path)
  if saved != dict(sys.modules):
    raise AssertionError('Fake reader module custody changed')
  return result, owner


class TestChestnutReader(unittest.TestCase):
  def sources(self):
    root = Path(__file__).resolve().parents[4]
    current = root / 'openpilot/starpilot/models/runtime_bridge/runtime.py'
    legacy = root / 'tinygrad_model_runtime/legacy9d/tinygrad_repo/examples/openpilot/helpers.py'
    return reader(current, '_load_chestnut'), reader(legacy, 'load_pickle')

  def test_chunked_bytes_offsets_dtype_alias_and_tail_match_legacy(self):
    current, legacy = self.sources()
    size = 2 * (8 << 20) + 173
    pattern = bytes((index * 37 + 11) % 256 for index in range(251))
    payload = (pattern * (size // len(pattern) + 1))[:size]
    with tempfile.TemporaryDirectory() as temporary:
      path = Path(temporary) / 'captured.pkl'
      opcodes, records = envelope(path, payload)
      old, old_store = invoke(legacy, path, legacy=True)
      new, store = invoke(current, path)
      self.assertEqual(path.read_bytes()[8:8 + len(opcodes)], opcodes)
      self.assertEqual(new['metadata'], old['metadata'])
      self.assertEqual(new['metadata'], {'abi': 'fixture', 'shape': (3, 7)})
      self.assertEqual(store.copies, [(0, 0, 8 << 20, 'uchar', 'uchar'),
                                    (8 << 20, 8 << 20, 8 << 20, 'uchar', 'uchar'),
                                    (16 << 20, 16 << 20, 173, 'uchar', 'uchar')])
      self.assertEqual(old_store.copies, [(0, 0, size, 'uchar', 'uchar')])
      self.assertEqual(sum(row[2] for row in store.copies), size)
      self.assertEqual(store.source_views, [(row[2], 'uchar', row[1]) for row in store.copies])
      self.assertEqual(len(store.arenas), 1)
      device, count, dtype, preallocate, arena = store.arenas[0]
      self.assertEqual((device, count, dtype.name, preallocate), ('CPU:LLVM', size, 'uchar', True))
      self.assertEqual(bytes(arena.storage), payload)
      self.assertEqual(bytes(old_store.arenas[0][4].storage), payload)
      for expected, old_view, view in zip(records, old['views'], new['views'], strict=True):
        count, dtype, offset = expected
        self.assertEqual((view.size, view.dtype.name, view.offset), (count, dtype.name, offset))
        self.assertEqual((old_view.size, old_view.dtype.name, old_view.offset), (count, dtype.name, offset))
        self.assertIs(view.storage, arena.storage)
        self.assertEqual(bytes(view.storage[offset:offset + view.nbytes]), payload[offset:offset + view.nbytes])
      # Two independently unpickled persistent records alias the same arena.
      first, _, alias = new['views']
      first.storage[first.offset] ^= 0xff
      self.assertEqual(alias.storage[alias.offset], payload[first.offset] ^ 0xff)
      self.assertEqual(bytes(new['views'][1].storage[-19:]), payload[-19:])

  def test_opcode_only_pickle_does_not_allocate_an_arena(self):
    current, legacy = self.sources()
    expected = {'metadata': (1, 'no arena'), 'values': [3, 5, 8]}
    opcodes = pickle.dumps(expected)
    with tempfile.TemporaryDirectory() as temporary:
      path = Path(temporary) / 'empty-arena.pkl'
      path.write_bytes(struct.pack('<q', len(opcodes)) + opcodes)
      for function, selected_legacy in ((current, False), (legacy, True)):
        with self.subTest(legacy=selected_legacy):
          result, store = invoke(function, path, legacy=selected_legacy)
          self.assertEqual(result, expected)
          self.assertFalse(store.arenas)
          self.assertFalse(store.copies)
          self.assertFalse(store.source_views)

  def test_zero_single_byte_and_chunk_boundary_lengths(self):
    current, legacy = self.sources()
    chunk = 8 << 20
    opcodes = pickle.dumps({'boundary': 'same opcode envelope'})
    pattern = bytes(range(251))
    with tempfile.TemporaryDirectory() as temporary:
      path = Path(temporary) / 'boundary.pkl'
      for size in (0, 1, chunk - 1, chunk, chunk + 1):
        with self.subTest(size=size):
          payload = (pattern * (size // len(pattern) + 1))[:size]
          path.write_bytes(struct.pack('<q', len(opcodes)) + opcodes + payload)
          old, old_store = invoke(legacy, path, legacy=True)
          new, store = invoke(current, path)
          self.assertEqual(new, old)
          self.assertEqual(new, {'boundary': 'same opcode envelope'})
          expected = [(offset, offset, min(chunk, size - offset), 'uchar', 'uchar')
                      for offset in range(0, size, chunk)]
          self.assertEqual(store.copies, expected)
          self.assertEqual(store.source_views, [(row[2], 'uchar', row[1]) for row in expected])
          self.assertEqual(sum(row[2] for row in store.copies), size)
          if size:
            self.assertEqual(old_store.copies, [(0, 0, size, 'uchar', 'uchar')])
            self.assertEqual(bytes(store.arenas[0][4].storage), payload)
            self.assertEqual(bytes(old_store.arenas[0][4].storage), payload)
            self.assertEqual(store.arenas[0][4].storage[-1], payload[-1])
          else:
            self.assertFalse(store.arenas)
            self.assertFalse(old_store.arenas)
            self.assertFalse(old_store.copies)

  def test_truncated_envelope_header_rejects_before_copy(self):
    current, legacy = self.sources()
    with tempfile.TemporaryDirectory() as temporary:
      path = Path(temporary) / 'truncated.pkl'
      path.write_bytes(b'\x01\x00')
      for function, selected_legacy in ((current, False), (legacy, True)):
        with self.subTest(legacy=selected_legacy), self.assertRaises(struct.error):
          invoke(function, path, legacy=selected_legacy)


if __name__ == '__main__':
  unittest.main()
