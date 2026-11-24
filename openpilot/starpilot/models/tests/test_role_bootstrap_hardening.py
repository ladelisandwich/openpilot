"""Bytecode and cache hardening of the production role bootstrap."""
import importlib
import os
import sys
import unittest
from unittest.mock import patch

from openpilot.starpilot.models.runtime_bridge import bootstrap
from openpilot.starpilot.models.tests.runtime_bridge_fixtures import BootstrapFixture


class BootstrapV3Tests(BootstrapFixture):
  def test_external_bytecode_prefix_is_cleared_before_import(self):
    external = str(self.base / 'external-bytecode')
    with patch.object(sys, 'pycache_prefix', external), patch.dict(os.environ, {'PYTHONPYCACHEPREFIX': external}):
      self.activate()
      self.assertIsNone(sys.pycache_prefix)
      self.assertNotIn('PYTHONPYCACHEPREFIX', os.environ)
      self.assertEqual(importlib.import_module('tinygrad').IDENTITY, 'selected legacy')
      self.assertFalse(list(self.bundle.rglob('*.pyc')))

  def test_failed_activation_restores_external_bytecode_prefix(self):
    external = str(self.base / 'external-bytecode')
    with patch.object(sys, 'pycache_prefix', external), patch.dict(os.environ, {'PYTHONPYCACHEPREFIX': external}):
      with patch.object(bootstrap, '_check_source_package', side_effect=RuntimeError('wrong import')):
        with self.assertRaises(RuntimeError):
          self.activate()
      self.assertEqual(sys.pycache_prefix, external)
      self.assertEqual(os.environ['PYTHONPYCACHEPREFIX'], external)
      self.assertEqual(sys.path, self.old_path)
      self.assertNotIn('tinygrad_repo', sys.modules)
      self.assertNotIn('extra', sys.modules)

  def test_xdg_child_symlink_is_rejected(self):
    leaf = self.cache / bootstrap.LEGACY_REVISION / 'driving' / self.sha
    leaf.mkdir(parents=True, mode=0o700)
    external = self.base / 'outside-cache'
    external.mkdir()
    (leaf / 'xdg').symlink_to(external, target_is_directory=True)
    with self.assertRaises(ValueError):
      self.activate()
    self.assertEqual(sys.path, self.old_path)

  def test_xdg_child_permissions_are_checked(self):
    xdg = self.cache / bootstrap.LEGACY_REVISION / 'driving' / self.sha / 'xdg'
    xdg.mkdir(parents=True, mode=0o700)
    xdg.parent.chmod(0o700)
    xdg.chmod(0o777)
    with self.assertRaises(ValueError):
      self.activate()
    xdg.chmod(0o700)
    result = self.activate()
    self.assertEqual(os.environ['XDG_CACHE_HOME'], str(xdg))
    self.assertEqual(result['cache'], str(xdg.parent))


if __name__ == '__main__':
  unittest.main()
