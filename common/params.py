from __future__ import annotations

import datetime
import fcntl
import json
import os
import re
import tempfile
import time
from enum import IntEnum, IntFlag
from pathlib import Path

SETTINGS_SIMPLE = 0
SETTINGS_ADVANCED = 1


def _load_settings_tiers() -> dict[str, int]:
  params_keys = Path(__file__).with_name("params_keys.h")
  if not params_keys.exists():
    return {}

  tiers = {}
  for line in params_keys.read_text(encoding="utf-8", errors="ignore").splitlines():
    if not line.lstrip().startswith('{"'):
      continue
    parts = line.split('"')
    if len(parts) >= 2:
      tiers[parts[1]] = SETTINGS_SIMPLE if "SETTINGS_SIMPLE" in line else SETTINGS_ADVANCED
  return tiers


_SETTINGS_TIERS = _load_settings_tiers()

# --- keys declared in params_keys.h that the compiled params library does not know ---------------------
#
# common/params_pyx.so is a prebuilt artifact: it was last compiled before some keys were added to
# params_keys.h, and the device cannot rebuild it. For such a key the library raises UnknownKeyName from
# every get and put, so a settings toggle on it killed the UI and the car code only ever saw the default.
# These keys are kept in files instead: the same values on disk and the same lock and atomic write as the
# C++ (temp file, fsync, rename under <params>/.lock), but in a sibling directory <params>/dx, because
# Params::clearAll deletes every file under <params>/d it does not recognise on each manager start.
# Only keys without a CLEAR_ON_* flag are handled, so nothing the library would have cleared is kept.
_HEADER_KEY_RE = re.compile(r'^\s*\{"(?P<key>[^"]+)",\s*\{(?P<flags>[A-Z_ |]+?),\s*(?P<type>[A-Z]+)(?:,\s*"(?P<default>(?:[^"\\]|\\.)*)")?')


def _load_header_only_keys() -> dict[str, tuple[str, str | None, tuple[str, ...]]]:
  """key -> (type name, default, flag names) for every declared key with no CLEAR_ON_* flag."""
  params_keys = Path(__file__).with_name("params_keys.h")
  if not params_keys.exists():
    return {}
  keys: dict[str, tuple[str, str | None, tuple[str, ...]]] = {}
  for line in params_keys.read_text(encoding="utf-8", errors="ignore").splitlines():
    m = _HEADER_KEY_RE.match(line)
    if m is None or "CLEAR_ON" in m.group("flags"):
      continue
    default = m.group("default")
    flags = tuple(f.strip() for f in m.group("flags").split("|") if f.strip())
    keys[m.group("key")] = (m.group("type"), default.replace('\\"', '"') if default is not None else None, flags)
  return keys


_HEADER_ONLY_KEYS = _load_header_only_keys()


def _header_encode(type_name: str, dat) -> bytes:
  if isinstance(dat, bytes):
    return dat
  if type_name == "BOOL":
    return b"1" if dat else b"0"
  if type_name == "INT":
    return str(int(dat)).encode()
  if type_name == "FLOAT":
    return str(float(dat)).encode()
  if type_name == "TIME" and isinstance(dat, datetime.datetime):
    return dat.isoformat().encode()
  if type_name == "JSON" and isinstance(dat, (dict, list)):
    return json.dumps(dat).encode()
  return str(dat).encode()


def _header_decode(type_name: str, raw: bytes):
  if type_name == "BYTES":
    return raw
  if type_name == "BOOL":
    return raw == b"1"
  text = raw.decode("utf-8")
  if type_name == "INT":
    return int(float(text))
  if type_name == "FLOAT":
    return float(text)
  if type_name == "TIME":
    return datetime.datetime.fromisoformat(text)
  if type_name == "JSON":
    return json.loads(text)
  return text


class _HeaderOnlyStore:
  """File store for the header-only keys of one params root (see above)."""
  SUBDIR = "dx"

  def __init__(self, params_dir: str):
    # params_dir is the library's <params>/d
    self.root = os.path.dirname(params_dir.rstrip("/"))
    self.dir = os.path.join(self.root, self.SUBDIR)
    self.lock_path = os.path.join(self.root, ".lock")

  def path(self, key: str) -> str:
    return os.path.join(self.dir, key)

  def read(self, key: str) -> bytes | None:
    try:
      with open(self.path(key), "rb") as f:
        return f.read()
    except FileNotFoundError:
      return None

  def _locked(self):
    fd = os.open(self.lock_path, os.O_CREAT | os.O_WRONLY, 0o775)
    fcntl.flock(fd, fcntl.LOCK_EX)
    return fd

  def write(self, key: str, dat: bytes) -> None:
    os.makedirs(self.dir, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".tmp_value_", dir=self.root)
    try:
      with os.fdopen(fd, "wb") as f:
        f.write(dat)
        f.flush()
        os.fsync(f.fileno())
      lock = self._locked()
      try:
        os.rename(tmp, self.path(key))
        dfd = os.open(self.dir, os.O_RDONLY)
        try:
          os.fsync(dfd)
        finally:
          os.close(dfd)
      finally:
        os.close(lock)
    except BaseException:
      try:
        os.unlink(tmp)
      except OSError:
        pass
      raise

  def remove(self, key: str) -> None:
    lock = self._locked()
    try:
      try:
        os.unlink(self.path(key))
      except FileNotFoundError:
        pass
    finally:
      os.close(lock)

  def keys(self) -> list[str]:
    try:
      return sorted(os.listdir(self.dir))
    except FileNotFoundError:
      return []


_HEADER_STORES: dict[str, _HeaderOnlyStore] = {}


def _header_store(params_dir: str) -> _HeaderOnlyStore:
  store = _HEADER_STORES.get(params_dir)
  if store is None:
    store = _HEADER_STORES[params_dir] = _HeaderOnlyStore(params_dir)
  return store


def _compiled_params_class(base):
  """The Params class used with the compiled library: `base` is params_pyx.Params. Built by a function so
  the header-only key handling can be tested against a stand-in base without the compiled module."""

  class Params(base):
    def _header_only(self, key):
      """(name, type name, default, store) when the library rejected a key this wrapper handles, else None."""
      name = key.decode("utf-8") if isinstance(key, bytes) else str(key)
      info = _HEADER_ONLY_KEYS.get(name)
      if info is None:
        return None
      return name, info[0], info[1], _header_store(self.get_param_path())

    # Every remaining method of the compiled class that starts with check_key, so a caller that walks
    # all_keys() (the_galaxy, device_syncd) never hits UnknownKeyName on a header-only key.
    def get_tuning_level(self, key):
      try:
        return super().get_tuning_level(key)
      except UnknownKeyName:
        if self._header_only(key) is None:
          raise
        return 0

    def get_key_flag(self, key):
      try:
        return super().get_key_flag(key)
      except UnknownKeyName:
        shim = self._header_only(key)
        if shim is None:
          raise
        flag = ParamKeyFlag(0)
        for name in _HEADER_ONLY_KEYS[shim[0]][2]:
          if name in ParamKeyFlag.__members__:
            flag |= ParamKeyFlag[name]
        return flag

    def cpp2python(self, key, value):
      try:
        return super().cpp2python(key, value)
      except UnknownKeyName:
        shim = self._header_only(key)
        if shim is None:
          raise
        return _header_decode(shim[1], value if isinstance(value, bytes) else str(value).encode()) if value is not None else None

    def get_settings_tier(self, key):
      try:
        return super().get_settings_tier(key)
      except AttributeError:
        return _SETTINGS_TIERS.get(self.check_key(key), SETTINGS_ADVANCED)
      except UnknownKeyName:
        shim = self._header_only(key)
        if shim is None:
          raise
        return _SETTINGS_TIERS.get(shim[0], SETTINGS_ADVANCED)

    def get(self, key, block=False, return_default=False, encoding=None, default=None):
      try:
        value = super().get(key, block=block, return_default=return_default)
      except UnknownKeyName:
        shim = self._header_only(key)
        if shim is None:
          return default
        name, type_name, header_default, store = shim
        raw = store.read(name)
        while raw is None and block:
          time.sleep(0.1)
          raw = store.read(name)
        if raw is None and return_default and header_default is not None:
          raw = header_default.encode()
        value = _header_decode(type_name, raw) if raw is not None else None
      if value is None:
        return default
      if encoding is not None and isinstance(value, bytes):
        try:
          return value.decode(encoding)
        except Exception:
          return value.decode("utf-8", errors="replace")
      return value

    def get_bool(self, key, block=False, default=False):
      try:
        result = super().get(key, block=block, return_default=True)
        if result is None:
          return bool(default)
        return bool(result)
      except UnknownKeyName:
        shim = self._header_only(key)
        if shim is None:
          return bool(default)
        name, _, header_default, store = shim
        raw = store.read(name)
        if raw is None:
          return header_default == "1" if header_default is not None else bool(default)
        return raw == b"1"

    def get_int(self, key, block=False, return_default=False, default=0):
      val = self.get(key, block=block, return_default=return_default, encoding="utf-8")
      if val is None or val == "":
        return default
      try:
        return int(float(val))
      except ValueError:
        return default

    def get_float(self, key, block=False, return_default=False, default=0.0):
      val = self.get(key, block=block, return_default=return_default, encoding="utf-8")
      if val is None or val == "":
        return default
      try:
        return float(val)
      except ValueError:
        return default

    def put(self, key, dat):
      try:
        super().put(key, dat)
      except UnknownKeyName:
        shim = self._header_only(key)
        if shim is None:
          raise
        name, type_name, _, store = shim
        store.write(name, _header_encode(type_name, dat))

    def put_bool(self, key, val):
      try:
        super().put_bool(key, val)
      except UnknownKeyName:
        shim = self._header_only(key)
        if shim is None:
          raise
        name, _, _, store = shim
        store.write(name, b"1" if val else b"0")

    def put_nonblocking(self, key, dat):
      try:
        super().put_nonblocking(key, dat)
      except UnknownKeyName:
        self.put(key, dat)

    def put_bool_nonblocking(self, key, val):
      try:
        super().put_bool_nonblocking(key, val)
      except UnknownKeyName:
        self.put_bool(key, val)

    def remove(self, key):
      try:
        super().remove(key)
      except UnknownKeyName:
        shim = self._header_only(key)
        if shim is None:
          raise
        name, _, _, store = shim
        store.remove(name)

    def get_type(self, key):
      try:
        return super().get_type(key)
      except UnknownKeyName:
        shim = self._header_only(key)
        if shim is None:
          raise
        return ParamKeyType[shim[1]]

    def get_default_value(self, key):
      try:
        return super().get_default_value(key)
      except UnknownKeyName:
        shim = self._header_only(key)
        if shim is None:
          raise
        _, type_name, header_default, _ = shim
        return _header_decode(type_name, header_default.encode()) if header_default is not None else None

    def get_stock_value(self, key):
      try:
        return super().get_stock_value(key)
      except UnknownKeyName:
        if self._header_only(key) is None:
          raise
        return None

    def all_keys(self):
      keys = list(super().all_keys())
      known = {k.decode("utf-8") if isinstance(k, bytes) else str(k) for k in keys}
      keys += [name.encode() for name in _header_store(self.get_param_path()).keys() if name not in known and name in _HEADER_ONLY_KEYS]
      return keys

    def put_int(self, key, val):
      t = self.get_type(key)
      if t == ParamKeyType.FLOAT:
        self.put(key, float(val))
      elif t == ParamKeyType.INT:
        self.put(key, int(val))
      elif t == ParamKeyType.BOOL:
        self.put(key, bool(val))
      else:
        self.put(key, str(int(val)))

    def put_float(self, key, val):
      t = self.get_type(key)
      if t == ParamKeyType.FLOAT:
        self.put(key, float(val))
      elif t == ParamKeyType.INT:
        self.put(key, int(val))
      elif t == ParamKeyType.BOOL:
        self.put(key, bool(val))
      else:
        self.put(key, str(float(val)))

  return Params

try:
  from openpilot.common.params_pyx import Params as _Params, ParamKeyFlag, ParamKeyType, UnknownKeyName
except Exception:
  class UnknownKeyName(KeyError):
    pass

  class ParamKeyFlag(IntFlag):
    PERSISTENT = 0x02
    CLEAR_ON_MANAGER_START = 0x04
    CLEAR_ON_ONROAD_TRANSITION = 0x08
    CLEAR_ON_OFFROAD_TRANSITION = 0x10
    DONT_LOG = 0x20
    DEVELOPMENT_ONLY = 0x40
    CLEAR_ON_IGNITION_ON = 0x80
    ALL = 0xFFFFFFFF

  class ParamKeyType(IntEnum):
    STRING = 0
    BOOL = 1
    INT = 2
    FLOAT = 3
    TIME = 4
    JSON = 5
    BYTES = 6

  def _load_key_types() -> dict[str, ParamKeyType]:
    key_types: dict[str, ParamKeyType] = {}
    params_keys = Path(__file__).with_name("params_keys.h")
    if not params_keys.exists():
      return key_types

    for line in params_keys.read_text(encoding="utf-8", errors="ignore").splitlines():
      if not line.lstrip().startswith('{"'):
        continue

      parts = line.split('"')
      if len(parts) < 2:
        continue

      key = parts[1]
      for type_name in ParamKeyType.__members__:
        if f", {type_name}" in line:
          key_types[key] = ParamKeyType[type_name]
          break
      else:
        key_types[key] = ParamKeyType.STRING

    return key_types

  _KEY_TYPES = _load_key_types()
  _PERSISTENT_STORE: dict[str, object] = {}
  _MEMORY_STORE: dict[str, object] = {}

  class Params:
    def __init__(self, d: str | None = None, memory: bool = False, return_defaults: bool = False):
      self.d = d if d is not None else ""
      self.m = memory
      self.return_defaults = return_defaults
      self._store = _MEMORY_STORE if memory else _PERSISTENT_STORE

    def __reduce__(self):
      return type(self), (self.d, self.m, self.return_defaults)

    def clear_all(self, tx_flag=ParamKeyFlag.ALL):
      self._store.clear()

    def check_key(self, key):
      if isinstance(key, bytes):
        key = key.decode("utf-8")
      return str(key)

    def _coerce_bool(self, value) -> bool:
      if isinstance(value, bool):
        return value
      if isinstance(value, (int, float)):
        return bool(value)
      if isinstance(value, bytes):
        return value == b"1"
      if isinstance(value, str):
        return value.strip().lower() not in ("", "0", "false", "none", "null")
      return bool(value)

    def _coerce_value(self, key: str, value):
      value_type = self.get_type(key)
      if value_type == ParamKeyType.BOOL:
        return self._coerce_bool(value)
      if value_type == ParamKeyType.INT:
        return int(float(value))
      if value_type == ParamKeyType.FLOAT:
        return float(value)
      if value_type == ParamKeyType.BYTES and isinstance(value, str):
        return value.encode("utf-8")
      return value

    def get(self, key, block: bool = False, return_default: bool = False, encoding=None, default=None):
      key = self.check_key(key)
      value = self._store.get(key, default)
      if value is None:
        return default
      if encoding is not None and isinstance(value, bytes):
        try:
          return value.decode(encoding)
        except Exception:
          return value.decode("utf-8", errors="replace")
      return value

    def get_bool(self, key, block: bool = False, default: bool = False):
      value = self.get(key, block=block, return_default=True, default=default)
      return self._coerce_bool(value)

    def get_int(self, key, block: bool = False, return_default: bool = False, default: int = 0):
      value = self.get(key, block=block, return_default=return_default, encoding="utf-8", default=default)
      if value is None or value == "":
        return default
      try:
        return int(float(value))
      except (TypeError, ValueError):
        return default

    def get_float(self, key, block: bool = False, return_default: bool = False, default: float = 0.0):
      value = self.get(key, block=block, return_default=return_default, encoding="utf-8", default=default)
      if value is None or value == "":
        return default
      try:
        return float(value)
      except (TypeError, ValueError):
        return default

    def put(self, key, dat):
      key = self.check_key(key)
      self._store[key] = self._coerce_value(key, dat)

    def put_bool(self, key, val: bool):
      self.put(key, bool(val))

    def put_nonblocking(self, key, dat):
      self.put(key, dat)

    def put_bool_nonblocking(self, key, val: bool):
      self.put_bool(key, val)

    def put_int(self, key, val):
      self.put(key, int(val))

    def put_float(self, key, val):
      self.put(key, float(val))

    def remove(self, key):
      self._store.pop(self.check_key(key), None)

    def get_param_path(self, key: str = ""):
      base = Path(tempfile.gettempdir()) / ("params_memory" if self.m else "params")
      base.mkdir(parents=True, exist_ok=True)
      return str(base / key) if key else str(base)

    def get_type(self, key):
      return _KEY_TYPES.get(self.check_key(key), ParamKeyType.STRING)

    def all_keys(self):
      return list(_KEY_TYPES)

    def get_default_value(self, key):
      return None

    def cpp2python(self, key, value):
      return self._coerce_value(self.check_key(key), value)

    def get_key_flag(self, key):
      return ParamKeyFlag.PERSISTENT

    def get_stock_value(self, key):
      return None

    def get_tuning_level(self, key):
      return 0

    def get_settings_tier(self, key):
      return _SETTINGS_TIERS.get(self.check_key(key), SETTINGS_ADVANCED)

else:
  assert _Params
  assert ParamKeyFlag
  assert ParamKeyType
  assert UnknownKeyName

  Params = _compiled_params_class(_Params)


if __name__ == "__main__":
  import sys

  params = Params()
  key = sys.argv[1]
  assert params.check_key(key), f"unknown param: {key}"

  if len(sys.argv) == 3:
    val = sys.argv[2]
    print(f"SET: {key} = {val}")
    params.put(key, val)
  elif len(sys.argv) == 2:
    print(f"GET: {key} = {params.get(key)}")
