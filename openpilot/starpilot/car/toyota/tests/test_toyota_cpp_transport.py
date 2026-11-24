import json
import pytest

@pytest.fixture(scope='module')
def host_heartbeat_bridge(tmp_path_factory):
  import hashlib
  import re
  import capnproto
  import subprocess
  import struct
  from pathlib import Path

  root = Path(__file__).resolve().parents[5]
  tmp = tmp_path_factory.mktemp('toyota-angle-host')
  pandad = (root / 'openpilot/selfdrive/pandad/pandad.cc').read_text()
  matched = re.search(r'      if \(aol_write_ok\) \{.*?\n      \}', pandad, re.S)
  assert matched is not None
  code = r'''
#include <cstring>
#include <vector>
#include "selfdrive/pandad/aol_wire.h"
#include "openpilot/cereal/gen/cpp/log.capnp.h"
#include "openpilot/starpilot/car/toyota/aol_policy.h"
static const AolSafetyProfile profiles[] = {TOYOTA_AOL_PROFILE};
static const AolProfileRegistry registry{profiles, std::size(profiles)};
static AolAxisNegotiator negotiator(registry);
static AolWritePlan pending;
static aol_safety_health_t snapshot;
extern "C" void reset_host() { negotiator = AolAxisNegotiator(registry); }
extern "C" uint32_t plan_host(uint8_t mode, uint16_t word, uint8_t request, uint8_t permission, uint64_t now,
 const uint8_t *axis_bytes, size_t axis_size, const uint8_t *intent_bytes, size_t intent_size, bool engaged, uint8_t capability) {
 snapshot = {AOL_SAFETY_PROTOCOL_MAGIC, AOL_SAFETY_PROTOCOL_VERSION, request, permission, mode, word, capability};
 AolAxisInput input;
 if (axis_size != 0U) {
  std::vector<capnp::word> words(axis_size / 8U);
  std::memcpy(words.data(), axis_bytes, axis_size);
  capnp::FlatArrayMessageReader reader(kj::ArrayPtr<const capnp::word>(words.data(), words.size()));
  auto event = reader.getRoot<cereal::Event>();
  auto axis = event.getAolAxisState();
  input = {true, axis.getQualified(), axis.getSessionId().cStr(), axis.getObservedMonoTime(),
           axis.getValidUntilMonoTime(), event.getLogMonoTime(), axis.getDesiredLateral(), axis.getDesiredLongitudinal()};
  if (intent_size != 0U) {
   std::vector<capnp::word> intent_words(intent_size / 8U);
   std::memcpy(intent_words.data(), intent_bytes, intent_size);
   capnp::FlatArrayMessageReader intent_reader(kj::ArrayPtr<const capnp::word>(intent_words.data(), intent_words.size()));
   auto intent = intent_reader.getRoot<cereal::Event>();
   input.retain_lateral_arm = aol_armed_intent(intent.getAolIntentWire(), intent.getLogMonoTime(),
        axis.getSourceCarStateMonoTime(), now, false, axis.getIntentProducerSessionId().cStr());
  }
 }
 auto plan = negotiator.prepare(snapshot, input, now, mode);
 pending = plan;
 const bool aol_write_ok = plan.capable;
''' + matched[0] + r'''
 return uint32_t(plan.request_mask) | (uint32_t(engaged) << 8U) | (uint32_t(plan.retain_lateral_arm) << 9U);
}
extern "C" void complete_host(uint8_t request, uint8_t permission) {
 snapshot.request_mask = request; snapshot.permission_mask = permission;
 negotiator.complete(pending, true, snapshot);
}
'''
  source = tmp / 'host.cc'
  code += r'''
#include <iostream>
template <typename T> bool read_value(T &value) { return bool(std::cin.read(reinterpret_cast<char *>(&value), sizeof(T))); }
static void respond(uint32_t value) { std::cout.write(reinterpret_cast<const char *>(&value), sizeof(value)); std::cout.flush(); }
int main() {
 uint8_t op;
 while (read_value(op)) {
  if (op == 0U) { reset_host(); respond(0U); }
  else if (op == 1U) {
   uint8_t mode, request, permission, engaged, capability; uint16_t word; uint64_t now; uint32_t axis_size, intent_size;
   if (!read_value(mode) || !read_value(word) || !read_value(request) || !read_value(permission) || !read_value(now) ||
       !read_value(axis_size) || !read_value(intent_size) || !read_value(engaged) || !read_value(capability)) return 2;
   std::vector<uint8_t> axis(axis_size), intent(intent_size);
   if (!std::cin.read(reinterpret_cast<char *>(axis.data()), axis_size) ||
       !std::cin.read(reinterpret_cast<char *>(intent.data()), intent_size)) return 3;
   respond(plan_host(mode, word, request, permission, now, axis.data(), axis_size, intent.data(), intent_size, engaged != 0U, capability));
  } else if (op == 2U) {
   uint8_t request, permission; if (!read_value(request) || !read_value(permission)) return 4;
   complete_host(request, permission); respond(0U);
  } else if (op == 3U) { respond(0U); return 0; }
  else if (op == 4U) { uint16_t word; if (!read_value(word)) return 6; respond(toyota_aol_param(word)); }
  else return 5;
 }
 return 0;
}
'''
  source.write_text(code)
  library = tmp / 'host-peer'
  generated = [root / ('openpilot/cereal/gen/cpp/' + name + '.capnp.c++') for name in ('log', 'car', 'custom', 'deprecated')]
  capnp_flags = ['-I', capnproto.INCLUDE_DIR, '-L', capnproto.LIB_DIR, '-lcapnp', '-lkj']
  subprocess.run(['c++', '-std=c++17', '-O1', '-Wall', '-Werror',
                  '-I', str(root), '-I', str(root / 'openpilot'), '-I', str(root / 'opendbc_repo'), '-I', str(root / 'openpilot/cereal/gen/cpp'),
                  str(source), *map(str, generated), *capnp_flags, '-o', str(library)], check=True, timeout=35)

  class Peer:
    def __init__(self):
      self.process = subprocess.Popen([str(library)], stdin=subprocess.PIPE, stdout=subprocess.PIPE)

    def exchange(self, raw):
      assert self.process.stdin is not None and self.process.stdout is not None
      assert self.process.stdin.write(raw) == len(raw)
      self.process.stdin.flush()
      result = self.process.stdout.read(4)
      assert len(result) == 4, self.process.poll()
      return struct.unpack('<I', result)[0]

    def reset_host(self):
      assert self.exchange(b'\x00') == 0

    def plan_host(self, mode, word, request, permission, now, axis, axis_size, intent, intent_size, engaged, capability):
      assert axis_size == len(axis) and intent_size == len(intent)
      return self.exchange(b'\x01' + struct.pack('<BHBBQIIBB', mode, word, request, permission, now,
                                               axis_size, intent_size, int(engaged), capability) + axis + intent)

    def complete_host(self, request, permission):
      assert self.exchange(b'\x02' + struct.pack('<BB', request, permission)) == 0

    def admits(self, word):
      return bool(self.exchange(b'\x04' + struct.pack('<H', word)))

    def close(self):
      try:
        if self.process.poll() is None:
          assert self.exchange(b'\x03') == 0
        assert self.process.wait(timeout=5) == 0
      finally:
        if self.process.poll() is None:
          self.process.kill()
          self.process.wait(timeout=5)
        if self.process.stdin is not None:
          self.process.stdin.close()
        if self.process.stdout is not None:
          self.process.stdout.close()

  paths = ['openpilot/selfdrive/pandad/pandad.cc', 'openpilot/selfdrive/pandad/aol_wire.h',
           'openpilot/selfdrive/pandad/aol_protocol.h', 'openpilot/starpilot/car/toyota/aol_policy.h',
           'opendbc_repo/opendbc/safety/modes/toyota.h', 'opendbc_repo/opendbc/safety/modes/toyota_aol.h']
  paths += [str(f.relative_to(root)) for f in generated]
  paths += [str(f.relative_to(root)) for f in (root / 'openpilot/cereal/gen/cpp').glob('*.capnp.h')]
  queued = [root / name for name in paths]
  closure = {}
  include_roots = [root, root / 'openpilot', root / 'opendbc_repo', root / 'openpilot/cereal/gen/cpp']
  while queued:
    path = queued.pop().resolve()
    name = str(path.relative_to(root))
    if name in closure:
      continue
    closure[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    if name == 'openpilot/selfdrive/pandad/pandad.cc':
      continue
    for quoted in re.findall(r'^\s*#include\s+"([^"\n]+)"', path.read_text(), re.M):
      target = next((directory / quoted for directory in [path.parent, *include_roots] if (directory / quoted).is_file()), None)
      assert target is not None, (name, quoted)
      queued.append(target)
  print(json.dumps({'hostHeartbeatSourceClosure': closure, 'artifactSha256': hashlib.sha256(library.read_bytes()).hexdigest()}))
  peer = Peer()
  try:
    yield peer
  finally:
    peer.close()
