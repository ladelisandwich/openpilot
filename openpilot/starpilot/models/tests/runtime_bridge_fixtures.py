"""Stdlib fixtures: synthetic sealed source bundles, with per-case process custody."""
import hashlib
import importlib
import json
import os
from pathlib import Path
import sys
import tempfile
from typing import TypedDict
import unittest

from openpilot.starpilot.models.runtime_bridge import bootstrap


class RuntimeManifest(TypedDict):
  compiler_revision: str
  adapter: str
  roles: dict[str, list[str]]
  files: dict[str, str]


class ProcessCustody(unittest.TestCase):
  """Restore state even when fixture construction or an assertion fails.

  No runtime or device package is imported to create these fixtures. The tests
  intentionally mutate interpreter globals and must run serially in a process.
  """

  def setUp(self):
    super().setUp()
    saved_modules = sys.modules.copy()
    saved_env = dict(os.environ)
    saved_path = sys.path[:]
    saved_meta_path = sys.meta_path[:]
    saved_importers = sys.path_importer_cache.copy()
    saved_bytecode = sys.dont_write_bytecode
    saved_prefix = sys.pycache_prefix

    def restore():
      sys.modules.clear()
      sys.modules.update(saved_modules)
      os.environ.clear()
      os.environ.update(saved_env)
      sys.path[:] = saved_path
      sys.meta_path[:] = saved_meta_path
      sys.dont_write_bytecode = saved_bytecode
      sys.pycache_prefix = saved_prefix
      importlib.invalidate_caches()
      sys.path_importer_cache.clear()
      sys.path_importer_cache.update(saved_importers)

    self.addCleanup(restore)

  def isolate_import_families(self, families):
    # Give synthetic packages exclusive names for this case. Cleanup restores
    # every pre-existing module object, including an ambient global tinygrad.
    for name in list(sys.modules):
      if any(name == family or name.startswith(family + '.') for family in families):
        sys.modules.pop(name)
    importlib.invalidate_caches()


class BootstrapFixture(ProcessCustody):
  def setUp(self):
    super().setUp()
    self.isolate_import_families(bootstrap.IMPORT_FAMILIES)
    self.assertFalse(any(n == f or n.startswith(f + '.') for n in sys.modules
                         for f in bootstrap.IMPORT_FAMILIES), 'Use a fresh interpreter for these tests')
    self.temp = tempfile.TemporaryDirectory()
    self.addCleanup(self.temp.cleanup)
    self.base = Path(self.temp.name).resolve()
    self.bundle = self.base / 'runtime'
    self.bundle.mkdir()
    self.cache = self.base / 'cache'
    self.sources = {
      'tinygrad_repo/LICENSE': 'test license\n',
      'tinygrad_repo/tinygrad/__init__.py': "IDENTITY = 'selected legacy'\n",
      'tinygrad_repo/examples/__init__.py': '',
      'tinygrad_repo/examples/openpilot/helpers.py': "IDENTITY = 'selected helper'\n",
      'tinygrad_repo/extra/probe.py': "IDENTITY = 'selected extra'\n",
    }
    for name, value in self.sources.items():
      p = self.bundle / name
      p.parent.mkdir(parents=True, exist_ok=True)
      p.write_text(value)
    self.old_path = sys.path[:]
    self.old_env = {k: os.environ.get(k) for k in bootstrap._ENV_KEYS}
    self.old_bytecode = sys.dont_write_bytecode
    self.seal()

  def seal(self):
    files = {name: hashlib.sha256((self.bundle / name).read_bytes()).hexdigest() for name in self.sources}
    self.manifest: RuntimeManifest = {
      'compiler_revision': bootstrap.LEGACY_REVISION, 'adapter': 'bounded-host-v2',
      'roles': {'driving': ['QCOM', 'USB+AMD:LLVM'], 'monitoring': ['QCOM']}, 'files': files,
    }
    self.write_manifest()

  def write_manifest(self):
    raw = json.dumps(self.manifest, sort_keys=True).encode()
    (self.bundle / 'runtime-manifest.json').write_bytes(raw)
    self.sha = hashlib.sha256(raw).hexdigest()

  def activate(self, **changes):
    args = {'bundle': self.bundle, 'manifest_sha256': self.sha,
            'compiler_revision': bootstrap.LEGACY_REVISION, 'role': 'driving', 'cache_root': self.cache}
    args.update(changes)
    return bootstrap.activate(**args)

