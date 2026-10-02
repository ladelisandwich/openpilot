"""Keys declared in params_keys.h that the compiled params library does not know (see common/params.py).

The compiled module is not importable here, so the wrapper class is built on a stand-in base that behaves like
params_pyx.Params: it knows a few keys and raises UnknownKeyName for everything else."""
import os
import tempfile

import pytest

from openpilot.common import params as P
from openpilot.common.params import ParamKeyFlag, ParamKeyType, UnknownKeyName, _HEADER_ONLY_KEYS, _HeaderOnlyStore, _compiled_params_class


class FakeCompiled:
  """Stand-in for params_pyx.Params."""
  KNOWN = {"TorqueInterceptorEnabled": ("BOOL", "1"), "DongleId": ("STRING", None)}

  def __init__(self, d="", memory=False, return_defaults=False):
    self.store = {}
    self.d = d

  def check_key(self, key):
    key = key.decode() if isinstance(key, bytes) else key
    if key not in self.KNOWN:
      raise UnknownKeyName(key)
    return key

  def get_param_path(self, key=""):
    return os.path.join(self.d, "d", key) if key else os.path.join(self.d, "d")

  def get(self, key, block=False, return_default=False):
    return self.store.get(self.check_key(key))

  def get_bool(self, key, block=False):
    return self.store.get(self.check_key(key)) is True

  def put(self, key, dat):
    self.store[self.check_key(key)] = dat

  def put_bool(self, key, val):
    self.store[self.check_key(key)] = bool(val)

  def put_nonblocking(self, key, dat):
    self.put(key, dat)

  def put_bool_nonblocking(self, key, val):
    self.put_bool(key, val)

  def remove(self, key):
    self.store.pop(self.check_key(key), None)

  def get_type(self, key):
    return ParamKeyType[self.KNOWN[self.check_key(key)][0]]

  def get_default_value(self, key):
    return self.KNOWN[self.check_key(key)][1]

  def get_stock_value(self, key):
    self.check_key(key)
    return None

  def all_keys(self):
    return [k.encode() for k in self.KNOWN]

  def get_settings_tier(self, key):
    self.check_key(key)
    return 1

  def get_tuning_level(self, key):
    self.check_key(key)
    return 0

  def get_key_flag(self, key):
    self.check_key(key)
    return ParamKeyFlag.PERSISTENT

  def cpp2python(self, key, value):
    self.check_key(key)
    return value


Params = _compiled_params_class(FakeCompiled)


@pytest.fixture
def root(monkeypatch):
  with tempfile.TemporaryDirectory() as d:
    os.makedirs(os.path.join(d, "d"))
    monkeypatch.setattr(P, "_HEADER_STORES", {})
    yield d


class TestHeaderParsing:
  def test_mazda_keys_are_header_only_candidates(self):
    for key in ("RadarEmulationEnabled", "LowerMinSetSpeed", "MazdaHybridLong"):
      assert _HEADER_ONLY_KEYS[key] == ("BOOL", None, ("PERSISTENT",))
    assert _HEADER_ONLY_KEYS["TorqueInterceptorEnabled"] == ("BOOL", "1", ("PERSISTENT",))

  def test_cleared_keys_are_never_handled(self):
    # anything the library would clear on manager start or a transition must not be kept in files
    assert "AccessToken" not in _HEADER_ONLY_KEYS
    assert "CarParams" not in _HEADER_ONLY_KEYS


class TestStore:
  def test_files_live_beside_d_and_survive_a_clear_of_d(self, root):
    store = _HeaderOnlyStore(os.path.join(root, "d"))
    store.write("MazdaHybridLong", b"1")
    assert store.read("MazdaHybridLong") == b"1"
    assert os.path.isfile(os.path.join(root, "dx", "MazdaHybridLong"))
    assert not os.path.exists(os.path.join(root, "d", "MazdaHybridLong"))
    assert os.path.isfile(os.path.join(root, ".lock"))
    assert not [f for f in os.listdir(root) if f.startswith(".tmp_value_")]
    for f in os.listdir(os.path.join(root, "d")):   # what Params::clearAll does to unknown files under d
      os.unlink(os.path.join(root, "d", f))
    assert store.read("MazdaHybridLong") == b"1"
    store.remove("MazdaHybridLong")
    assert store.read("MazdaHybridLong") is None
    store.remove("MazdaHybridLong")   # idempotent


class TestWrapper:
  def test_known_keys_still_go_to_the_library(self, root):
    p = Params(root)
    p.put_bool("TorqueInterceptorEnabled", True)
    assert p.get_bool("TorqueInterceptorEnabled")
    assert p.store["TorqueInterceptorEnabled"] is True
    assert not os.path.exists(os.path.join(root, "dx"))

  def test_toggle_round_trip_for_a_header_only_key(self, root):
    p = Params(root)
    assert p.get("MazdaHybridLong") is None
    assert p.get_bool("MazdaHybridLong") is False
    p.put_bool("MazdaHybridLong", True)          # what the settings toggle does
    assert p.get_bool("MazdaHybridLong") is True
    assert p.get("MazdaHybridLong") is True
    assert Params(root).get_bool("MazdaHybridLong") is True   # another process sees the file
    p.put_bool("MazdaHybridLong", False)
    assert p.get_bool("MazdaHybridLong") is False
    p.put_bool_nonblocking("RadarEmulationEnabled", True)
    assert p.get_bool("RadarEmulationEnabled") is True
    p.remove("RadarEmulationEnabled")
    assert p.get("RadarEmulationEnabled") is None
    assert p.get_type("MazdaHybridLong") == ParamKeyType.BOOL
    assert p.get_default_value("MazdaHybridLong") is None
    assert p.get_stock_value("MazdaHybridLong") is None
    assert b"MazdaHybridLong" in p.all_keys() and b"DongleId" in p.all_keys()

  def test_every_method_the_galaxy_calls_on_all_keys(self, root, monkeypatch):
    # the_galaxy builds its default-params table from all_keys(); none of these may raise for a header-only key
    monkeypatch.setitem(_HEADER_ONLY_KEYS, "FakeInt", ("INT", "5", ("PERSISTENT", "DONT_LOG")))
    p = Params(root)
    p.put_bool("MazdaHybridLong", True)
    p.put_int("FakeInt", 9)
    for raw_key in p.all_keys():
      p.get_default_value(raw_key)
      p.get_type(raw_key)
      p.get_tuning_level(raw_key)
      p.get_key_flag(raw_key)
      p.get_settings_tier(raw_key)
      p.get_stock_value(raw_key)
    assert p.get_tuning_level("MazdaHybridLong") == 0
    assert p.get_key_flag("MazdaHybridLong") == ParamKeyFlag.PERSISTENT
    assert p.get_key_flag("FakeInt") == ParamKeyFlag.PERSISTENT | ParamKeyFlag.DONT_LOG
    assert p.get_settings_tier("MazdaHybridLong") in (0, 1)
    assert p.cpp2python("MazdaHybridLong", b"1") is True and p.cpp2python("FakeInt", b"9") == 9
    with pytest.raises(UnknownKeyName):
      p.get_tuning_level("NoSuchKey")

  def test_header_default_applies_when_unset(self, root, monkeypatch):
    monkeypatch.setitem(_HEADER_ONLY_KEYS, "FakeDefaultOn", ("BOOL", "1", ("PERSISTENT",)))
    p = Params(root)
    assert p.get_bool("FakeDefaultOn") is True
    assert p.get("FakeDefaultOn") is None
    assert p.get("FakeDefaultOn", return_default=True) is True
    assert p.get_default_value("FakeDefaultOn") is True
    p.put_bool("FakeDefaultOn", False)
    assert p.get_bool("FakeDefaultOn") is False

  def test_other_types(self, root, monkeypatch):
    monkeypatch.setitem(_HEADER_ONLY_KEYS, "FakeInt", ("INT", None, ("PERSISTENT", "DONT_LOG")))
    monkeypatch.setitem(_HEADER_ONLY_KEYS, "FakeFloat", ("FLOAT", None, ("PERSISTENT", "DONT_LOG")))
    monkeypatch.setitem(_HEADER_ONLY_KEYS, "FakeStr", ("STRING", None, ("PERSISTENT", "DONT_LOG")))
    monkeypatch.setitem(_HEADER_ONLY_KEYS, "FakeJson", ("JSON", None, ("PERSISTENT", "DONT_LOG")))
    p = Params(root)
    p.put_int("FakeInt", 7)
    assert p.get_int("FakeInt") == 7 and p.get("FakeInt") == 7
    p.put_float("FakeFloat", 2.5)
    assert p.get_float("FakeFloat") == 2.5
    p.put("FakeStr", "hello")
    assert p.get("FakeStr") == "hello" and p.get("FakeStr", encoding="utf-8") == "hello"
    p.put("FakeJson", {"a": [1, 2]})
    assert p.get("FakeJson") == {"a": [1, 2]}

  def test_truly_unknown_keys_behave_as_before(self, root):
    p = Params(root)
    assert p.get("NoSuchKey") is None
    assert p.get("NoSuchKey", default="x") == "x"
    assert p.get_bool("NoSuchKey") is False
    with pytest.raises(UnknownKeyName):
      p.put_bool("NoSuchKey", True)
    with pytest.raises(UnknownKeyName):
      p.put("NoSuchKey", "x")
    with pytest.raises(UnknownKeyName):
      p.remove("NoSuchKey")
    with pytest.raises(UnknownKeyName):
      p.get_type("NoSuchKey")

  def test_memory_params_get_their_own_store(self, root):
    mem_root = os.path.join(root, "shm")
    os.makedirs(os.path.join(mem_root, "d"))
    Params(mem_root).put_bool("MazdaHybridLong", True)
    assert Params(root).get_bool("MazdaHybridLong") is False
    assert Params(mem_root).get_bool("MazdaHybridLong") is True
