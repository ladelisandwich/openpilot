"""Actual exported native status + actual production C++ negotiate/write/HB/read order."""
import ctypes
import os
import subprocess


class PandadOrder:
  def __init__(self, safety, library_path, executable):
    self.safety = safety
    self.lib = ctypes.CDLL(str(library_path))
    self.process = subprocess.Popen([str(executable)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    self.heartbeat = False
    self.compatible = False
    self.axis_session = ''

  def status(self):
    # Board USB0xd5 initializer reads request first, permission second.
    request = int(self.safety.aol_get_request_mask())
    permission = int(self.safety.aol_get_permission_mask())
    mode = ctypes.c_uint16.in_dll(self.lib, 'current_safety_mode').value
    word = ctypes.c_uint16.in_dll(self.lib, 'current_safety_param').value
    capability = int(ctypes.c_void_p.in_dll(self.lib, 'aol_policy').value is not None)
    return request, permission, mode, word, capability

  def exchange(self, line):
    assert self.process.poll() is None
    self.process.stdin.write(line + '\n')
    self.process.stdin.flush()
    result = self.process.stdout.readline().strip().split()
    assert result, self.process.poll()
    return result

  def step(self, driven, published, now):
    before = self.status()
    axis_event = driven['aolAxisState']
    axis = axis_event.aolAxisState
    intent_event = published['aolIntentWire']
    fields = [now, int(axis_event.valid), int(axis.qualified), axis.sessionId, axis.observedMonoTime, axis.validUntilMonoTime,
              axis_event.logMonoTime, int(axis.desiredLateral), int(axis.desiredLongitudinal), int(intent_event.valid and published['carState'].valid),
              axis.sourceCarStateMonoTime, intent_event.logMonoTime, bytes(intent_event.aolIntentWire).hex(), str(axis.intentProducerSessionId) or '-']
    capable, request, retain = map(int, self.exchange('PREP ' + ' '.join(map(str, (*before, *fields)))))
    if capable:
      self.safety.aol_set_host_request(request)
    enabled = int(driven['selfdriveState'].selfdriveState.enabled)
    self.heartbeat = bool(int(self.exchange('HB ' + ' '.join(map(str, (*before, enabled, capable))))[0]))
    self.safety.set_aol_test_heartbeat(self.heartbeat)
    after = self.status()
    compatible, session = self.exchange('DONE ' + ' '.join(map(str, (*after, capable))))
    self.compatible = bool(int(compatible))
    self.axis_session = '' if session == '-' else session
    return None, {'before': before, 'plan': (capable, request, retain), 'heartbeat': self.heartbeat,
                  'after': after, 'session': session}

  def close(self):
    self.process.stdin.close()
    try:
      assert self.process.wait(timeout=10) == 0
    finally:
      self.process.stdout.close()


import hashlib
from pathlib import Path
import re

import pytest


@pytest.fixture(scope='module')
def hkg_pandad_order(tmp_path_factory):
  try:
    import capnproto
    include_dir, library_dir = capnproto.INCLUDE_DIR, capnproto.LIB_DIR
  except ModuleNotFoundError:
    prefix = Path(os.environ['CAPNP_PREFIX'])
    assert (prefix / 'include/capnp/message.h').is_file()
    include_dir, library_dir = str(prefix / 'include'), str(prefix / 'lib')

  root = Path(__file__).resolve().parents[4]
  directory = tmp_path_factory.mktemp('hkg-pandad-order')
  original = root / 'openpilot/starpilot/aol/tests/hyundai_pandad_order.cc'
  pandad = root / 'openpilot/selfdrive/pandad/pandad.cc'
  matched = re.search(r'      if \(aol_write_ok\) \{.*?\n      \}', pandad.read_text(), re.S)
  assert matched is not None
  source = directory / 'peer.cc'
  source.write_text(original.read_text().replace('// PRODUCTION_HEARTBEAT_BLOCK', matched[0]))
  generated = [root / ('openpilot/cereal/gen/cpp/' + name + '.capnp.c++') for name in ('log', 'car', 'custom', 'deprecated')]
  paths = [original, pandad, *generated, *list((root / 'openpilot/cereal/gen/cpp').glob('*.capnp.h'))]
  roots = [root, root / 'openpilot', root / 'opendbc_repo', root / 'openpilot/cereal/gen/cpp']
  closure = {}
  queue = [original, *paths[2:]]
  while queue:
    path = queue.pop()
    if path in closure:
      continue
    closure[path] = hashlib.sha256(path.read_bytes()).hexdigest()
    for quoted in re.findall(r'^\s*#include\s+"([^"\n]+)"', path.read_text(), re.M):
      included = next((r / quoted for r in [path.parent, *roots] if (r / quoted).is_file()), None)
      assert included is not None, (path, quoted)
      queue.append(included)
  closure[pandad] = hashlib.sha256(pandad.read_bytes()).hexdigest()
  executable = directory / 'peer'
  subprocess.run(['c++', '-std=c++17', '-O1', '-Wall', '-Wextra', '-Werror',
    *[flag for r in roots for flag in ('-I', str(r))], '-I', include_dir,
    str(source), *map(str, generated), '-L', library_dir, '-lcapnp', '-lkj', '-o', str(executable)],
    check=True, timeout=40)
  assert all(hashlib.sha256(path.read_bytes()).hexdigest() == value for path, value in closure.items())
  print({'productionHeartbeatSourceClosure': {str(p.relative_to(root)): h for p, h in closure.items()},
         'cppArtifactSha256': hashlib.sha256(executable.read_bytes()).hexdigest()})
  yield lambda native, library: PandadOrder(native, library, executable)
  assert all(hashlib.sha256(path.read_bytes()).hexdigest() == value for path, value in closure.items())
