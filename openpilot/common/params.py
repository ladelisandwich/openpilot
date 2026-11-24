import sys
import fcntl
import json
import os
import re
import ctypes
import tempfile
import time
import weakref
import builtins
import datetime
from pathlib import Path
from enum import IntEnum, IntFlag

from openpilot.common.swaglog import cloudlog
from openpilot.common.params_paths import params_path


class ParamKeyFlag(IntFlag):
  PERSISTENT = 0x02
  CLEAR_ON_MANAGER_START = 0x04
  CLEAR_ON_ONROAD_TRANSITION = 0x08
  CLEAR_ON_OFFROAD_TRANSITION = 0x10
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


_suffix = ".dylib" if sys.platform == "darwin" else ".so"
lib = ctypes.CDLL(Path(__file__).with_name(f"libparams_c{_suffix}"))

ParamsHandle = ctypes.c_void_p


class ParamsBuffer(ctypes.Structure):
  _fields_ = [("data", ctypes.c_void_p), ("size", ctypes.c_size_t)]


def _bind_raw(name, args, result=None):
  function = getattr(lib, name)
  function.argtypes = args
  function.restype = result
  return function


params_last_error = _bind_raw("params_last_error", [], ctypes.c_char_p)


def _bind(name, args, result=None):
  function = _bind_raw(name, args, result)

  def checked(*call_args):
    value = function(*call_args)
    if error := params_last_error():
      raise RuntimeError(error.decode())
    return value

  return checked


params_create = _bind("params_create", [ctypes.c_char_p, ctypes.c_size_t], ParamsHandle)
params_destroy = _bind("params_destroy", [ParamsHandle])
params_clear_all = _bind("params_clear_all", [ParamsHandle, ctypes.c_uint])
params_check_key = _bind("params_check_key", [ParamsHandle, ctypes.c_char_p], ctypes.c_bool)
params_get_key_type = _bind("params_get_key_type", [ParamsHandle, ctypes.c_char_p], ctypes.c_int)
params_get_default = _bind("params_get_default", [ParamsHandle, ctypes.c_char_p], ParamsBuffer)
params_get = _bind("params_get", [ParamsHandle, ctypes.c_char_p, ctypes.c_bool], ParamsBuffer)
params_get_bool = _bind("params_get_bool", [ParamsHandle, ctypes.c_char_p, ctypes.c_bool], ctypes.c_bool)
params_put = _bind("params_put", [ParamsHandle, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_size_t, ctypes.c_bool], ctypes.c_int)
params_put_bool = _bind("params_put_bool", [ParamsHandle, ctypes.c_char_p, ctypes.c_bool, ctypes.c_bool], ctypes.c_int)
params_remove = _bind("params_remove", [ParamsHandle, ctypes.c_char_p], ctypes.c_int)
params_get_path = _bind("params_get_path", [ParamsHandle, ctypes.c_char_p, ctypes.c_size_t], ParamsBuffer)
params_keys_size = _bind("params_keys_size", [ParamsHandle], ctypes.c_size_t)
params_key_at = _bind("params_key_at", [ParamsHandle, ctypes.c_size_t], ParamsBuffer)

PYTHON_2_CPP = {
  (str, ParamKeyType.STRING): lambda v: v,
  (builtins.bool, ParamKeyType.BOOL): lambda v: "1" if v else "0",
  (int, ParamKeyType.INT): str,
  (float, ParamKeyType.FLOAT): str,
  (datetime.datetime, ParamKeyType.TIME): lambda v: v.isoformat(),
  (dict, ParamKeyType.JSON): json.dumps,
  (list, ParamKeyType.JSON): json.dumps,
  (bytes, ParamKeyType.BYTES): lambda v: v,
}
CPP_2_PYTHON = {
  ParamKeyType.STRING: lambda v: v.decode("utf-8"),
  ParamKeyType.BOOL: lambda v: v == b"1",
  ParamKeyType.INT: int,
  ParamKeyType.FLOAT: float,
  ParamKeyType.TIME: lambda v: datetime.datetime.fromisoformat(v.decode("utf-8")),
  ParamKeyType.JSON: json.loads,
  ParamKeyType.BYTES: lambda v: v,
}


def ensure_bytes(v):
  return v.encode() if isinstance(v, str) else v


def _copy_string(value):
  if value.data is None:
    return None
  return ctypes.string_at(value.data, value.size)


class UnknownKeyName(Exception):
  pass


class _CompiledParams:
  def __init__(self, d=""):
    path = ensure_bytes(params_path(d))
    self.p = params_create(path, len(path))
    self._finalizer = weakref.finalize(self, params_destroy, self.p)
    self._finalizer.atexit = False  # daemon threads can still be using live Params handles during interpreter shutdown
    self.d = d

  def __reduce__(self):
    return (type(self), (self.d,))

  def clear_all(self, tx_flag=ParamKeyFlag.ALL):
    params_clear_all(self.p, int(tx_flag))

  def check_key(self, key):
    key = ensure_bytes(key)
    if b"\0" in key or not params_check_key(self.p, key):
      raise UnknownKeyName(key)
    return key

  def python2cpp(self, proposed_type, expected_type, value, key):
    cast = PYTHON_2_CPP.get((proposed_type, expected_type))
    if cast:
      return cast(value)
    raise TypeError(f"Type mismatch while writing param {key}: {proposed_type=} {expected_type=} {value=}")

  def _cpp2python(self, t, value, default, key):
    if value is None:
      return None
    try:
      return CPP_2_PYTHON[t](value)
    except (KeyError, TypeError, ValueError):
      cloudlog.warning(f"Failed to cast param {key} with {value=} from type {t=}")
      return self._cpp2python(t, default, None, key)

  def _default(self, key):
    return _copy_string(params_get_default(self.p, key))

  def get(self, key, block=False, return_default=False):
    k = self.check_key(key)
    t = self.get_type(k)
    default = self._default(k) if return_default else None
    value = _copy_string(params_get(self.p, k, block))
    if value == b"":
      if block:
        raise KeyboardInterrupt
      return self._cpp2python(t, default, None, key)
    return self._cpp2python(t, value, default, key)

  def get_bool(self, key, block=False):
    return bool(params_get_bool(self.p, self.check_key(key), block))

  def _put_cast(self, key, dat):
    return ensure_bytes(self.python2cpp(type(dat), self.get_type(key), dat, key))

  def put(self, key, dat, block=False):
    """Write a parameter. block=True waits until it is persisted to disk."""
    k = self.check_key(key)
    value = self._put_cast(k, dat)
    params_put(self.p, k, value, len(value), block)

  def put_bool(self, key, val, block=False):
    params_put_bool(self.p, self.check_key(key), val, block)

  def remove(self, key):
    params_remove(self.p, self.check_key(key))

  def get_param_path(self, key=""):
    key = ensure_bytes(key)
    return _copy_string(params_get_path(self.p, key, len(key))).decode()

  def get_type(self, key):
    return ParamKeyType(params_get_key_type(self.p, self.check_key(key)))

  def all_keys(self):
    keys = []
    for i in range(params_keys_size(self.p)):
      keys.append(_copy_string(params_key_at(self.p, i)))
    return keys

  def get_default_value(self, key):
    k = self.check_key(key)
    return self._cpp2python(self.get_type(k), self._default(k), None, key)

  def cpp2python(self, key, value):
    return self._cpp2python(self.get_type(key), value, None, key)



# --- keys declared in params_keys.h that the compiled params library does not know ---------------------
#
# libparams_c.so is a prebuilt artifact: it was last compiled before some keys were added to params_keys.h,
# and the device cannot rebuild it. For such a key the library raises UnknownKeyName from every get and put,
# so a settings toggle on it killed the UI and the car code only ever saw the default. These keys are kept in
# files instead: the same values on disk and the same lock and atomic write as the C++ (temp file, fsync,
# rename under <params>/.lock), but in a sibling directory <params>/dx, because Params::clearAll deletes
# every file under <params>/d it does not recognise on each manager start. Only keys without a CLEAR_ON_*
# flag are handled, so nothing the library would have cleared is kept. A key the library does know is never
# served from here, whatever the header says.
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
    os.makedirs(self.root, exist_ok=True)
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


class Params(_CompiledParams):
  """The compiled library's params, plus the header-only keys above and the StarPilot 6 era conveniences
  (get_int/put_int, get_float/put_float, the *_nonblocking names, get(..., default=, encoding=))."""

  def _header_only(self, key):
    """(name, type name, default, store) for a key the library does not know but the header declares, else None."""
    name = key.decode("utf-8") if isinstance(key, bytes) else str(key)
    info = _HEADER_ONLY_KEYS.get(name)
    if info is None or b"\0" in name.encode() or params_check_key(self.p, name.encode()):
      return None
    return name, info[0], info[1], _header_store(self.get_param_path())

  def check_key(self, key):
    try:
      return super().check_key(key)
    except UnknownKeyName:
      if self._header_only(key) is None:
        raise
      return ensure_bytes(key)

  def get_type(self, key):
    shim = self._header_only(key)
    if shim is None:
      return super().get_type(key)
    return ParamKeyType[shim[1]]

  def get(self, key, block=False, return_default=False, encoding=None, default=None):
    shim = self._header_only(key)
    if shim is None:
      value = super().get(key, block=block, return_default=return_default)
    else:
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
      return value.decode(encoding, errors="replace")
    return value

  def get_bool(self, key, block=False, default=False):
    shim = self._header_only(key)
    if shim is None:
      return super().get_bool(key, block=block)
    name, _, header_default, store = shim
    raw = store.read(name)
    if raw is None:
      return header_default == "1" if header_default is not None else bool(default)
    return raw == b"1"

  def put(self, key, dat, block=False):
    shim = self._header_only(key)
    if shim is None:
      return super().put(key, dat, block=block)
    name, type_name, _, store = shim
    store.write(name, _header_encode(type_name, dat))
    return None

  def put_bool(self, key, val, block=False):
    shim = self._header_only(key)
    if shim is None:
      return super().put_bool(key, val, block=block)
    shim[3].write(shim[0], b"1" if val else b"0")
    return None

  def remove(self, key):
    shim = self._header_only(key)
    if shim is None:
      return super().remove(key)
    shim[3].remove(shim[0])
    return None

  def get_default_value(self, key):
    shim = self._header_only(key)
    if shim is None:
      return super().get_default_value(key)
    return _header_decode(shim[1], shim[2].encode()) if shim[2] is not None else None

  def cpp2python(self, key, value):
    shim = self._header_only(key)
    if shim is None:
      return super().cpp2python(key, value)
    return _header_decode(shim[1], value if isinstance(value, bytes) else str(value).encode()) if value is not None else None

  def all_keys(self):
    keys = super().all_keys()
    known = set(keys)
    return keys + [name.encode() for name in _HEADER_ONLY_KEYS if name.encode() not in known and self._header_only(name) is not None]

  def get_param_path(self, key=""):
    shim = self._header_only(key) if key else None
    if shim is None:
      return super().get_param_path(key)
    return shim[3].path(shim[0])

  # the StarPilot 6 era names, used by the ported Mazda code and tools
  def get_int(self, key, block=False, default=0):
    value = self.get(key, block=block, return_default=True)
    try:
      return int(value) if value is not None else int(default)
    except (TypeError, ValueError):
      return int(default)

  def put_int(self, key, val, block=False):
    self.put(key, int(val), block=block)

  def get_float(self, key, block=False, default=0.0):
    value = self.get(key, block=block, return_default=True)
    try:
      return float(value) if value is not None else float(default)
    except (TypeError, ValueError):
      return float(default)

  def put_float(self, key, val, block=False):
    self.put(key, float(val), block=block)

  def put_nonblocking(self, key, dat):
    self.put(key, dat)

  def put_bool_nonblocking(self, key, val):
    self.put_bool(key, val)

  def put_int_nonblocking(self, key, val):
    self.put_int(key, val)

  def put_float_nonblocking(self, key, val):
    self.put_float(key, val)


if __name__ == "__main__":
  import sys

  params = Params()
  key = sys.argv[1]
  params.check_key(key)
  if len(sys.argv) == 3:
    val = sys.argv[2]
    print(f"SET: {key} = {val}")
    params.put(key, val, block=True)
  elif len(sys.argv) == 2:
    print(f"GET: {key} = {params.get(key)}")
