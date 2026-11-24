"""The guard libraries are pinned by hash; a device build must not overwrite them. Fake SCons nodes, no compilers."""

import hashlib
import json
import shutil
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import pytest

BRIDGE = Path(__file__).resolve().parents[1] / "runtime_bridge"
SOURCE = BRIDGE / "SConscript"
LIBRARIES = ("libbounded_wait.so", "libusb_guards.so")


class Returned(Exception):
  pass


def evaluate(directory, arch="comma_arm64"):
  env = Mock()
  env.subst_list.return_value = []
  guard_env = env.Clone.return_value

  def stop(*_):
    raise Returned()

  scope = {"env": env, "arch": arch, "Import": Mock(), "Return": stop,
           "Dir": lambda _: SimpleNamespace(srcnode=lambda: SimpleNamespace(abspath=str(directory)))}
  try:
    exec(compile(SOURCE.read_text(), str(SOURCE), "exec"), scope)
  except Returned:
    pass
  return guard_env


@pytest.fixture(autouse=True)
def fake_scons(monkeypatch):
  errors = ModuleType("SCons.Errors")
  errors.UserError = type("UserError", (Exception,), {})
  monkeypatch.setitem(sys.modules, "SCons", ModuleType("SCons"))
  monkeypatch.setitem(sys.modules, "SCons.Errors", errors)


@pytest.fixture
def bridge(tmp_path):
  for name in ("guard-policy.json", "bounded_wait.c", "usb_guards.c", *LIBRARIES):
    shutil.copy(BRIDGE / name, tmp_path / name)
  return tmp_path


def test_shipped_libraries_match_the_policy_that_pins_them():
  pinned = json.loads((BRIDGE / "guard-policy.json").read_bytes())["files"]
  for name in LIBRARIES:
    assert hashlib.sha256((BRIDGE / name).read_bytes()).hexdigest() == pinned[name]


def test_matching_shipped_libraries_are_kept_not_rebuilt(bridge):
  assert not evaluate(bridge).SharedLibrary.called


@pytest.mark.parametrize("damage", ("changed", "missing"))
@pytest.mark.parametrize("name", LIBRARIES)
def test_a_changed_or_missing_library_is_rebuilt_so_the_guard_can_refuse_it(bridge, name, damage):
  if damage == "changed":
    (bridge / name).write_bytes((bridge / name).read_bytes() + b"\0")
  else:
    (bridge / name).unlink()
  built = [call.args[0] for call in evaluate(bridge).SharedLibrary.call_args_list]
  assert built == ["bounded_wait", "usb_guards"]


def test_other_architectures_build_nothing(bridge):
  assert not evaluate(bridge, arch="x86_64").SharedLibrary.called
