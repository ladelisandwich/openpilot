from collections.abc import Callable
from typing import cast, NoReturn
import hashlib
import json
from types import SimpleNamespace

import pytest
from openpilot.starpilot.flm import native_primitives as native


@pytest.fixture
def fixture_binding(tmp_path, monkeypatch):
  here = tmp_path / 'repo/openpilot/starpilot/flm'
  here.mkdir(parents=True)
  monkeypatch.setattr(native, 'HERE', here)
  monkeypatch.setattr(native, '_enabled', False)
  # Metadata-only bindings must not inherit an extension imported by another test.
  monkeypatch.delitem(native.sys.modules, 'openpilot.starpilot.flm._primitive_walk', raising=False)
  clear_identity = native.native_identity.cache_clear
  clear_extension = native._extension.cache_clear
  clear_identity()
  clear_extension()
  for name, path in native.source_paths().items():
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('identity fixture ' + name)
  def write_binding():
    artifact = here / '_primitive_walk.so'
    artifact.write_bytes(b'fixture only; not executable native qualification')
    value = {'schema': 1, 'revision': native.REVISION, 'abi': native.runtime_abi(),
             'sources': {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in native.source_paths().items()},
             'artifact_sha256': hashlib.sha256(artifact.read_bytes()).hexdigest()}
    (here / '_primitive_walk.binding.json').write_text(json.dumps(value))
    native.native_identity.cache_clear()
    native._extension.cache_clear()
    return value
  yield here, write_binding
  clear_identity()
  clear_extension()


def test_producer_never_imports_or_loads_native_and_disabled_call_does_not_read_identity(fixture_binding, monkeypatch):
  _, write_binding = fixture_binding
  write_binding()
  monkeypatch.setattr(native.importlib.util, 'spec_from_file_location', lambda *args: cast(Callable[[str], NoReturn], pytest.fail)('producer native import'))
  assert json.loads(native.native_identity())['artifact_sha256']
  monkeypatch.setattr(native, 'native_identity', lambda: cast(Callable[[str], NoReturn], pytest.fail)('per-row source or ABI lookup'))
  assert native.try_native('whole_body_digest', {'value': -0.0}) is native.FALLBACK


def test_identity_binds_sources_executed_artifact_receipt_abi_and_absence(fixture_binding, monkeypatch):
  here, write_binding = fixture_binding
  missing = native.native_identity()
  assert json.loads(missing)['status'] == 'python-missing'
  write_binding()
  present = native.native_identity()
  assert present != missing
  for name in ('_primitive_walk.pyx', 'native_primitives.py', 'SConscript', 'SConstruct'):
    write_binding()
    path = native.source_paths()[name]
    path.write_text(path.read_text() + ' changed')
    native.native_identity.cache_clear()
    assert json.loads(native.native_identity())['status'] == 'python-invalid-binding'
  write_binding()
  (here / '_primitive_walk.so').write_bytes(b'different fixture artifact')
  native.native_identity.cache_clear()
  assert json.loads(native.native_identity())['status'] == 'python-invalid-binding'
  write_binding()
  monkeypatch.setattr(native, 'runtime_abi', lambda: {'different': 'ABI'})
  native.native_identity.cache_clear()
  assert json.loads(native.native_identity())['status'] == 'python-unsupported-abi'


def test_cached_binding_refuses_artifact_replacement_and_wrong_module_origin(fixture_binding, monkeypatch):
  here, write_binding = fixture_binding
  monkeypatch.setattr(native.sys, 'platform', 'linux')
  write_binding()
  # Fixture binds metadata only; it is never accepted as an executed extension.
  assert json.loads(native.native_identity())['status'] == 'native-bound'
  (here / '_primitive_walk.so').write_bytes(b'changed after identity capture')
  with pytest.raises(ValueError, match='artifact changed'):
    native._extension()
  write_binding()
  fake = SimpleNamespace(__file__=str(here.parent / 'unexpected.so'), compiled_abi=lambda: (3, 12, 64, native.REVISION))
  monkeypatch.setitem(native.sys.modules, 'openpilot.starpilot.flm._primitive_walk', fake)
  with pytest.raises(ValueError, match='unowned preloaded module'):
    native._extension()


def test_import_failure_is_fit_failure_not_native_marked_python_fallback(fixture_binding, monkeypatch):
  _, write_binding = fixture_binding
  monkeypatch.setattr(native.sys, 'platform', 'linux')
  write_binding()
  assert json.loads(native.native_identity())['status'] == 'native-bound'
  monkeypatch.setattr(native.importlib.util, 'spec_from_file_location', lambda *args: None)
  with pytest.raises(ValueError, match='module missing'):
    native._extension()


def test_worker_enable_requires_existing_caps_niceness_and_normal_scheduler(monkeypatch):
  import os
  import resource
  monkeypatch.setattr(native.sys, 'platform', 'linux')
  monkeypatch.setattr(resource, 'getrlimit', lambda kind: (20, 20) if kind == resource.RLIMIT_CPU else (2 * 1024**3, 2 * 1024**3))
  monkeypatch.setattr(os, 'getpriority', lambda *args: 10)
  monkeypatch.setattr(os, 'SCHED_OTHER', 0, raising=False)
  monkeypatch.setattr(os, 'sched_getscheduler', lambda pid: os.SCHED_OTHER, raising=False)
  monkeypatch.setattr(native, '_enabled', False)
  native.enable_fit_worker()
  assert native._enabled
  monkeypatch.setattr(os, 'getpriority', lambda *args: 0)
  monkeypatch.setattr(native, '_enabled', False)
  with pytest.raises(ValueError, match='worker context'):
    native.enable_fit_worker()
  assert not native._enabled


def test_whole_python_fallbacks_preserve_bits_and_alias_independence(monkeypatch):
  from openpilot.starpilot.flm import primitive_source_memo as memo, physical_input_observation as physical
  monkeypatch.setattr(native, '_enabled', False)
  shared = [True, 1, 1., -0.0, 'é\ud800', b'\x00\xff']
  value = {'first': shared, 'second': shared}
  result = memo.independent_primitive_graph(value)
  assert result['first'] is not result['second']
  assert physical.body_digest(result) == physical._body_digest_python(value)
  result['first'].append('mutation')
  assert result['second'] == shared
  cyclic = []
  cyclic.append(cyclic)
  with pytest.raises(ValueError, match='cyclic primitive memo input'):
    memo.independent_primitive_graph(cyclic)


def test_matching_origin_metadata_cannot_substitute_a_preloaded_python_object(fixture_binding, monkeypatch):
  here, write_binding = fixture_binding
  monkeypatch.setattr(native.sys, 'platform', 'linux')
  write_binding()
  name = 'openpilot.starpilot.flm._primitive_walk'
  path = here / '_primitive_walk.so'
  fake = SimpleNamespace(__spec__=native.importlib.util.spec_from_file_location(name, path), __file__=str(path),
                         compiled_abi=lambda: (3, 12, 64, native.REVISION), whole_body_digest=lambda value: 'forged')
  monkeypatch.setitem(native.sys.modules, name, fake)
  with pytest.raises(ValueError, match='unowned preloaded module'):
    native._extension()
