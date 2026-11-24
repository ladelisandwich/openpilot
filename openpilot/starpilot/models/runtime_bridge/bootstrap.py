"""Select the integrity-pinned driving and monitoring runtime before imports."""
import hashlib
import importlib
import importlib.abc
import importlib.machinery
import importlib.util
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import sys

LEGACY_REVISION = '9d0446a4ba8a532c8b674fb6ad795af015cd9dcf'
IMPORT_FAMILIES = ('tinygrad', 'tinygrad_repo', 'examples', 'extra')
_ENV_KEYS = ('CACHEDB', 'XDG_CACHE_HOME', 'PYTHONDONTWRITEBYTECODE', 'PYTHONPYCACHEPREFIX')


def _digest_file(path):
  h = hashlib.sha256()
  with path.open('rb') as source:
    while block := source.read(1 << 20):
      h.update(block)
  return h.hexdigest()


def _absolute_path(value):
  path = Path(value)
  if '..' in path.parts:
    raise ValueError('Parent traversal is not a runtime path')
  return path if path.is_absolute() else Path.cwd() / path


def _plain_path(path, missing_ok=False):
  """Check the supplied path before resolving can erase a symlink parent."""
  for current in (*reversed(path.parents), path):
    try:
      mode = current.lstat().st_mode
    except FileNotFoundError:
      if missing_ok:
        return
      raise ValueError('Missing runtime path: ' + str(current)) from None
    if stat.S_ISLNK(mode):
      raise ValueError('Symlink in runtime path: ' + str(current))
    if current != path and not stat.S_ISDIR(mode):
      raise ValueError('Non-directory runtime parent: ' + str(current))


def _inventory(bundle):
  files = set()
  for directory, dirs, names in os.walk(bundle, followlinks=False):
    for name in dirs + names:
      path = Path(directory) / name
      mode = path.lstat().st_mode
      if stat.S_ISLNK(mode):
        raise ValueError('Symlink in runtime bundle: ' + str(path))
      if stat.S_ISDIR(mode):
        continue
      if not stat.S_ISREG(mode):
        raise ValueError('Non-regular runtime source: ' + str(path))
      if path.suffix in ('.pyc', '.pyo'):
        raise ValueError('Cached Python bytecode in runtime bundle')
      files.add(path.relative_to(bundle).as_posix())
  return files


def _private_cache(path):
  _plain_path(path, missing_ok=True)
  for current in (*reversed(path.parents), path):
    try:
      current.mkdir(mode=0o700)
    except FileExistsError:
      pass
    mode = current.lstat().st_mode
    if stat.S_ISLNK(mode) or not stat.S_ISDIR(mode):
      raise ValueError('Invalid runtime cache directory')
  status = path.stat()
  if status.st_uid != os.geteuid() or stat.S_IMODE(status.st_mode) & 0o022:
    raise ValueError('Runtime cache must be owned and not writable by other users')
  for name in ('tinygrad.db', 'tinygrad.db-wal', 'tinygrad.db-shm'):
    leaf = path / name
    if leaf.exists() or leaf.is_symlink():
      mode = leaf.lstat().st_mode
      if stat.S_ISLNK(mode) or not stat.S_ISREG(mode):
        raise ValueError('Invalid runtime cache database')


def _namespace(name, directory):
  spec = importlib.machinery.ModuleSpec(name, loader=None, is_package=True)
  spec.submodule_search_locations = [str(directory)]
  return importlib.util.module_from_spec(spec)


class _PinnedSourceImports(importlib.abc.MetaPathFinder):
  """Resolve only verified source files, before ambient editable/import hooks."""

  def __init__(self, root):
    self.roots = {'tinygrad': root / 'tinygrad', 'examples': root / 'examples',
                  'tinygrad_repo': root, 'extra': root / 'extra'}

  def find_spec(self, fullname, path=None, target=None):
    family, *parts = fullname.split('.')
    if family not in self.roots:
      return None
    leaf = self.roots[family].joinpath(*parts)
    source = leaf / '__init__.py' if (leaf / '__init__.py').is_file() else leaf.with_suffix('.py')
    if source.is_file():
      _plain_path(source)
      return importlib.util.spec_from_file_location(fullname, source)
    if leaf.is_dir():
      _plain_path(leaf)
      return _namespace(fullname, leaf).__spec__
    raise ModuleNotFoundError('Module is outside selected runtime source: ' + fullname, name=fullname)


def _pin_source_imports(root):
  finder = _PinnedSourceImports(root)
  sys.meta_path.insert(0, finder)
  return finder


def _check_source_package(name, expected):
  spec = importlib.util.find_spec(name)
  if (spec is None or not isinstance(spec.loader, importlib.machinery.SourceFileLoader) or
      spec.origin is None or Path(spec.origin).resolve() != expected or
      list(spec.submodule_search_locations or ()) != [str(expected.parent)]):
    raise RuntimeError('Selected runtime does not own the ' + name + ' import')


def activate(bundle, manifest_sha256, compiler_revision, role, cache_root):
  if compiler_revision != LEGACY_REVISION or role not in ('driving', 'monitoring'):
    raise ValueError('Unsupported model runtime role or compiler identity')
  if not isinstance(manifest_sha256, str) or re.fullmatch('[0-9a-f]{64}', manifest_sha256) is None:
    raise ValueError('Invalid pinned runtime manifest hash')
  if any(name == family or name.startswith(family + '.')
         for name in sys.modules for family in IMPORT_FAMILIES):
    raise RuntimeError('Select the model runtime before importing its packages')
  bundle = _absolute_path(bundle)
  _plain_path(bundle)
  if not bundle.is_dir():
    raise ValueError('Runtime bundle is not a directory')
  manifest_path = bundle / 'runtime-manifest.json'
  _plain_path(manifest_path)
  if not stat.S_ISREG(manifest_path.lstat().st_mode):
    raise ValueError('Runtime manifest is not a regular file')
  raw = manifest_path.read_bytes()
  if hashlib.sha256(raw).hexdigest() != manifest_sha256:
    raise ValueError('Runtime manifest changed')
  manifest = json.loads(raw)
  if (not isinstance(manifest, dict) or manifest.get('compiler_revision') != compiler_revision or
      manifest.get('adapter') != 'bounded-host-v2' or
      manifest.get('roles') != {'driving': ['QCOM', 'USB+AMD:LLVM'], 'monitoring': ['QCOM']}):
    raise ValueError('Runtime identity mismatch')
  files = manifest.get('files')
  if not isinstance(files, dict) or not files:
    raise ValueError('Runtime source inventory is missing')
  for name, expected in files.items():
    if not isinstance(name, str):
      raise ValueError('Invalid runtime source path')
    relative = PurePosixPath(name)
    if (relative.is_absolute() or '..' in relative.parts or '\\' in name or
        str(relative) != name or name == 'runtime-manifest.json' or
        not isinstance(expected, str) or re.fullmatch('[0-9a-f]{64}', expected) is None):
      raise ValueError('Invalid runtime source inventory entry')
  if _inventory(bundle) != set(files) | {'runtime-manifest.json'}:
    raise ValueError('Runtime source inventory does not match the complete bundle')
  for name, expected in files.items():
    if _digest_file(bundle.joinpath(*PurePosixPath(name).parts)) != expected:
      raise ValueError('Runtime source changed: ' + name)
  tg_root = bundle / 'tinygrad_repo'
  for relative in ('tinygrad/__init__.py', 'examples/__init__.py'):
    if ('tinygrad_repo/' + relative) not in files:
      raise ValueError('Required runtime package is not pinned')
  for relative in ('extra',):
    if not (tg_root / relative).is_dir():
      raise ValueError('Required runtime namespace is missing')
  if (tg_root / '__init__.py').exists() or (tg_root / 'extra/__init__.py').exists():
    raise ValueError('Runtime namespace layout changed')
  cache = _absolute_path(cache_root) / compiler_revision / role / manifest_sha256
  if cache.is_relative_to(bundle):
    raise ValueError('Runtime cache must be outside the immutable bundle')
  _private_cache(cache)
  _private_cache(cache / 'xdg')
  old_path, old_bytecode, old_cache_prefix = sys.path[:], sys.dont_write_bytecode, sys.pycache_prefix
  old_meta_path = sys.meta_path[:]
  old_env = {key: os.environ.get(key) for key in _ENV_KEYS}
  try:
    sys.path[:0] = [str(bundle), str(tg_root)]
    sys.dont_write_bytecode = True
    sys.pycache_prefix = None
    os.environ.pop('PYTHONPYCACHEPREFIX', None)
    os.environ.update(CACHEDB=str(cache / 'tinygrad.db'), XDG_CACHE_HOME=str(cache / 'xdg'),
                      PYTHONDONTWRITEBYTECODE='1')
    # Fixed parent paths prevent namespace merging with the global runtime tree.
    sys.modules['tinygrad_repo'] = _namespace('tinygrad_repo', tg_root)
    sys.modules['extra'] = _namespace('extra', tg_root / 'extra')
    _pin_source_imports(tg_root)
    importlib.invalidate_caches()
    _check_source_package('tinygrad', tg_root / 'tinygrad/__init__.py')
    _check_source_package('examples', tg_root / 'examples/__init__.py')
  except BaseException:
    sys.path[:] = old_path
    sys.meta_path[:] = old_meta_path
    sys.dont_write_bytecode = old_bytecode
    sys.pycache_prefix = old_cache_prefix
    for name in ('tinygrad_repo', 'extra'):
      sys.modules.pop(name, None)
    for key, value in old_env.items():
      if value is None:
        os.environ.pop(key, None)
      else:
        os.environ[key] = value
    importlib.invalidate_caches()
    raise
  return {'compiler_revision': compiler_revision, 'manifest_sha256': manifest_sha256,
          'role': role, 'tinygrad_origin': str(tg_root / 'tinygrad/__init__.py'),
          'cache': str(cache), 'runtime_files': len(files)}
