"""Compile current native safety and C++ negotiation for Honda caller tests."""

from contextlib import contextmanager
from tempfile import TemporaryDirectory
from unittest.mock import patch
import subprocess
import hashlib
import json
import os
from pathlib import Path

import capnproto

from opendbc.safety.tests.libsafety import libsafety_py


@contextmanager
def honda_native_pair():
  root = Path(__file__).resolve().parents[4]
  with TemporaryDirectory(prefix='honda-native-') as directory:
    yield from _compiled_native_pair(root, Path(directory))


def _compiled_native_pair(root, temporary):
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
      '-MD',
      '-MF',
      str(temporary / 'libsafety.d'),
      '-c',
      str(root / 'opendbc_repo/opendbc/safety/tests/libsafety/safety.c'),
      '-o',
      str(temporary / "libsafety.o"),
    ],
    check=True,
    timeout=35,
  )
  subprocess.run(['cc', '-shared', str(temporary / 'libsafety.o'), '-o', str(library),
                  '-fsanitize=undefined', '-fno-sanitize-recover=undefined'], check=True, timeout=35)
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
      str(Path(__file__).with_name('honda_pandad_order.cc')),
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
  inputs = (temporary / 'libsafety.d').read_text().replace('\\\n', '').split(':', 1)[1].split()
  bridge_inputs = [root / name for name in ('openpilot/selfdrive/pandad/aol_protocol.h',
                                           'openpilot/selfdrive/pandad/aol_wire.h',
                                           'openpilot/starpilot/car/honda/aol_policy.h')]
  origins = {
    'root': str(root),
    'native_library': str(library),
    'bridge': str(bridge),
    'inputs': {name: hashlib.sha256(Path(name).read_bytes()).hexdigest() for name in inputs},
    'library_sha256': hashlib.sha256(library.read_bytes()).hexdigest(),
    'bridge_inputs': {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in bridge_inputs},
    'bridge_source_sha256': hashlib.sha256(Path(__file__).with_name('honda_pandad_order.cc').read_bytes()).hexdigest(),
    'schemas': {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in generated},
  }
  (temporary / 'origins.json').write_text(json.dumps(origins, indent=2) + '\n')
  original = {key: vars(libsafety_py).get(key) for key in ('libsafety', 'libpath')}
  with patch.dict('os.environ', {'HONDA_NATIVE_LIBRARY': str(library), 'HONDA_PANDAD_BRIDGE': str(bridge)}):
    libsafety_py.load(library)
    try:
      assert Path(libsafety_py.libpath).resolve() == library.resolve()
      assert Path(os.environ['HONDA_NATIVE_LIBRARY']).resolve() == library.resolve()
      assert Path(os.environ['HONDA_PANDAD_BRIDGE']).resolve() == bridge.resolve()
      yield origins
    finally:
      for key, value in original.items():
        if value is None:
          vars(libsafety_py).pop(key, None)
        else:
          setattr(libsafety_py, key, value)
