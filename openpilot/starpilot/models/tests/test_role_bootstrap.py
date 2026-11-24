"""Integrity and import isolation of the production role bootstrap."""
import importlib
import importlib.abc
import importlib.util
import os
import sys
import types
import unittest
from unittest.mock import patch

from openpilot.starpilot.models.runtime_bridge import bootstrap
from openpilot.starpilot.models.tests.runtime_bridge_fixtures import BootstrapFixture


class BootstrapTests(BootstrapFixture):
  def test_selected_sources_and_namespaces_are_isolated(self):
    other = self.base / 'other'
    (other / 'tinygrad_repo').mkdir(parents=True)
    (other / 'extra').mkdir()
    (other / 'extra/should_not_load.py').write_text('raise AssertionError("wrong runtime")')
    sys.path.insert(0, str(other))
    self.activate()
    tg = importlib.import_module('tinygrad')
    helpers = importlib.import_module('tinygrad_repo.examples.openpilot.helpers')
    direct = importlib.import_module('examples.openpilot.helpers')
    extra = importlib.import_module('extra.probe')
    self.assertEqual(tg.IDENTITY, 'selected legacy')
    self.assertEqual(helpers.IDENTITY, direct.IDENTITY)
    self.assertEqual(extra.IDENTITY, 'selected extra')
    self.assertEqual(list(sys.modules['tinygrad_repo'].__path__), [str(self.bundle / 'tinygrad_repo')])
    self.assertEqual(list(sys.modules['extra'].__path__), [str(self.bundle / 'tinygrad_repo/extra')])
    with self.assertRaises(ModuleNotFoundError):
      importlib.import_module('extra.should_not_load')
    self.assertFalse(list(self.bundle.rglob('*.pyc')))

  def test_cache_is_namespaced_and_both_cache_paths_are_set(self):
    result = self.activate()
    leaf = self.cache / bootstrap.LEGACY_REVISION / 'driving' / self.sha
    self.assertEqual(result['cache'], str(leaf))
    self.assertEqual(os.environ['CACHEDB'], str(leaf / 'tinygrad.db'))
    self.assertEqual(os.environ['XDG_CACHE_HOME'], str(leaf / 'xdg'))
    self.assertTrue(sys.dont_write_bytecode)
    self.assertEqual(os.environ['PYTHONDONTWRITEBYTECODE'], '1')

  def test_preloaded_package_and_child_aliases_are_rejected(self):
    for name in ('tinygrad', 'tinygrad.device', 'tinygrad_repo', 'tinygrad_repo.examples',
                 'examples', 'examples.openpilot.helpers', 'extra', 'extra.bench_log'):
      with self.subTest(name=name), patch.dict(sys.modules, {name: types.ModuleType(name)}):
        with self.assertRaises(RuntimeError):
          self.activate()
      self.assertEqual(sys.path, self.old_path)

  def test_unlisted_source_native_module_and_bytecode_are_rejected(self):
    for name in ('tinygrad_repo/tinygrad/unlisted.py', 'tinygrad_repo/tinygrad/device.so',
                 'tinygrad_repo/tinygrad/__pycache__/device.cpython-312.pyc'):
      with self.subTest(name=name):
        p = self.bundle / name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(b'unlisted')
        with self.assertRaises(ValueError):
          self.activate()
        p.unlink()

  def test_changed_and_missing_source_are_rejected(self):
    p = self.bundle / 'tinygrad_repo/tinygrad/__init__.py'
    p.write_text('changed')
    with self.assertRaises(ValueError):
      self.activate()
    p.unlink()
    with self.assertRaises(ValueError):
      self.activate()

  def test_symlink_file_and_directory_are_rejected(self):
    p = self.bundle / 'tinygrad_repo/tinygrad/__init__.py'
    p.unlink()
    target = self.base / 'outside.py'
    target.write_text(self.sources['tinygrad_repo/tinygrad/__init__.py'])
    p.symlink_to(target)
    with self.assertRaises(ValueError):
      self.activate()
    p.unlink()
    p.write_text(self.sources['tinygrad_repo/tinygrad/__init__.py'])
    link = self.bundle / 'tinygrad_repo/linked'
    link.symlink_to(self.base, target_is_directory=True)
    with self.assertRaises(ValueError):
      self.activate()

  def test_symlink_bundle_parent_is_rejected_before_resolve(self):
    link = self.base / 'alias'
    link.symlink_to(self.base, target_is_directory=True)
    with self.assertRaises(ValueError):
      self.activate(bundle=link / 'runtime')

  def test_manifest_symlink_and_wrong_pin_are_rejected(self):
    with self.assertRaises(ValueError):
      self.activate(manifest_sha256='0' * 64)
    p = self.bundle / 'runtime-manifest.json'
    target = self.base / 'manifest.json'
    p.rename(target)
    p.symlink_to(target)
    with self.assertRaises(ValueError):
      self.activate()

  def test_manifest_traversal_and_identity_are_rejected(self):
    self.manifest['files']['../outside.py'] = '0' * 64
    self.write_manifest()
    with self.assertRaises(ValueError):
      self.activate()
    self.seal()
    self.manifest['adapter'] = 'other'
    self.write_manifest()
    with self.assertRaises(ValueError):
      self.activate()
    for changes in ({'role': 'USB+AMD'}, {'compiler_revision': '02e'}, {'manifest_sha256': '../cache'}):
      with self.subTest(changes=changes), self.assertRaises(ValueError):
        self.activate(**changes)

  def test_symlink_cache_parent_and_database_are_rejected(self):
    real = self.base / 'real-cache'
    real.mkdir()
    self.cache.symlink_to(real, target_is_directory=True)
    with self.assertRaises(ValueError):
      self.activate()
    self.cache.unlink()
    leaf = self.cache / bootstrap.LEGACY_REVISION / 'driving' / self.sha
    leaf.mkdir(parents=True, mode=0o700)
    (leaf / 'tinygrad.db').symlink_to(self.base / 'outside.db')
    with self.assertRaises(ValueError):
      self.activate()

  def test_cache_inside_bundle_and_writable_leaf_are_rejected(self):
    with self.assertRaises(ValueError):
      self.activate(cache_root=self.bundle / 'cache')
    leaf = self.cache / bootstrap.LEGACY_REVISION / 'driving' / self.sha
    leaf.mkdir(parents=True, mode=0o700)
    leaf.chmod(0o777)
    with self.assertRaises(ValueError):
      self.activate()

  def test_failed_import_check_restores_process_state(self):
    with patch.object(bootstrap, '_check_source_package', side_effect=RuntimeError('wrong import')):
      with self.assertRaises(RuntimeError):
        self.activate()
    self.assertEqual(sys.path, self.old_path)
    self.assertEqual(sys.dont_write_bytecode, self.old_bytecode)
    self.assertEqual({k: os.environ.get(k) for k in bootstrap._ENV_KEYS}, self.old_env)
    self.assertNotIn('tinygrad_repo', sys.modules)
    self.assertNotIn('extra', sys.modules)


if __name__ == '__main__':
  unittest.main()


class SourceHookTests(BootstrapFixture):
  def test_selected_source_wins_over_ambient_meta_path_hook(self):
    outside = self.base / 'ambient'
    (outside / 'tinygrad').mkdir(parents=True)
    (outside / 'tinygrad/__init__.py').write_text("raise AssertionError('ambient source executed')")

    class AmbientHook(importlib.abc.MetaPathFinder):
      def find_spec(hook, fullname, path=None, target=None):
        if fullname == 'tinygrad':
          return importlib.util.spec_from_file_location(fullname, outside / 'tinygrad/__init__.py')
        return None

    sys.meta_path.insert(0, AmbientHook())
    self.activate()
    selected = importlib.import_module('tinygrad')
    self.assertEqual(selected.IDENTITY, 'selected legacy')
    self.assertEqual(selected.__file__, str(self.bundle / 'tinygrad_repo/tinygrad/__init__.py'))

  def test_selected_children_cannot_fall_back_to_ambient_hook(self):
    outside = self.base / 'unowned.py'
    outside.write_text("raise AssertionError('unowned child executed')")

    class AmbientHook(importlib.abc.MetaPathFinder):
      def find_spec(hook, fullname, path=None, target=None):
        if fullname == 'tinygrad.unowned':
          return importlib.util.spec_from_file_location(fullname, outside)
        return None

    sys.meta_path.insert(0, AmbientHook())
    self.activate(role='monitoring')
    with self.assertRaises(ModuleNotFoundError):
      importlib.import_module('tinygrad.unowned')
