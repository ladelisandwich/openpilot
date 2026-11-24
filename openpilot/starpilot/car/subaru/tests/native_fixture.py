"""Compile current native safety and C++ negotiation for Subaru caller tests."""

import subprocess
from pathlib import Path

import capnproto
import pytest

from opendbc.safety.tests.libsafety import libsafety_py


@pytest.fixture(scope='module')
def subaru_native_pair(tmp_path_factory):
  root = Path(__file__).resolve().parents[5]
  temporary = tmp_path_factory.mktemp('subaru-native')
  library = temporary / 'libsafety.so'
  bridge = temporary / 'pandad-order'
  subprocess.run(
    [
      'cc',
      '-fPIC',
      '-Wall',
      '-Wextra',
      '-Werror',
      '-nostdlib',
      '-fno-builtin',
      '-std=gnu11',
      '-Wfatal-errors',
      '-Wno-pointer-to-int-cast',
      '-g',
      '-O0',
      '-DALLOW_DEBUG',
      '-fsanitize=undefined',
      '-fno-sanitize-recover=undefined',
      '-I',
      str(root / 'opendbc_repo'),
      '-c',
      str(root / 'opendbc_repo/opendbc/safety/tests/libsafety/safety.c'),
      '-o',
      str(temporary / "libsafety.o"),
    ],
    check=True,
    timeout=35,
  )
  subprocess.run(
    ['cc', '-shared', str(temporary / 'libsafety.o'), '-o', str(library), '-fsanitize=undefined', '-fno-sanitize-recover=undefined'], check=True, timeout=35
  )
  generated = [root / ('openpilot/cereal/gen/cpp/' + name + '.capnp.c++') for name in ('log', 'car', 'custom', 'deprecated')]
  subprocess.run(
    [
      'c++',
      '-std=c++17',
      '-Wall',
      '-Wextra',
      '-Werror',
      '-O0',
      '-g',
      '-I',
      str(root),
      '-I',
      str(root / 'openpilot'),
      '-I',
      str(root / 'openpilot/cereal/gen/cpp'),
      '-I',
      capnproto.INCLUDE_DIR,
      str(Path(__file__).with_name('pandad_order.cc')),
      *map(str, generated),
      '-L',
      capnproto.LIB_DIR,
      '-lcapnp',
      '-lkj',
      '-o',
      str(bridge),
    ],
    check=True,
    timeout=35,
  )
  original = {key: vars(libsafety_py).get(key) for key in ('libsafety', 'libpath')}
  with pytest.MonkeyPatch.context() as environment:
    environment.setenv('SUBARU_NATIVE_LIBRARY', str(library))
    environment.setenv('SUBARU_PANDAD_BRIDGE', str(bridge))
    libsafety_py.load(library)
    try:
      yield library, bridge
    finally:
      for key, value in original.items():
        if value is None:
          vars(libsafety_py).pop(key, None)
        else:
          setattr(libsafety_py, key, value)
