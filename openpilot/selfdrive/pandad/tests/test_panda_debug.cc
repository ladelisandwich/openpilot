#include <algorithm>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "common/tests/native_test.h"
#include "selfdrive/pandad/panda.h"

class DebugHandle : public PandaCommsHandle {
public:
  std::string payload;
  int calls = 0;
  int writes = 0;
  bool endless = false;
  int error = 0;
  int control_write(uint8_t, uint16_t, uint16_t, unsigned int) override { ++writes; return 0; }
  int control_read(uint8_t request, uint16_t p1, uint16_t p2, unsigned char *out, uint16_t length, unsigned int) override {
    ++calls;
    CHECK(request == 0xb6);
    CHECK(p1 == 0 && p2 == 0 && length > 0 && length <= 64);
    if (error) return error;
    if (endless) { memset(out, 'x', length); return length; }
    const size_t count = std::min<size_t>(length, payload.size());
    memcpy(out, payload.data(), count);
    payload.erase(0, count);
    return count;
  }
  int bulk_write(unsigned char, unsigned char *, int, unsigned int) override { ++writes; return 0; }
  int bulk_read(unsigned char, unsigned char *, int, unsigned int) override { return 0; }
  void cleanup() override {}
};
class DebugPanda : public Panda {
public:
  explicit DebugPanda(std::unique_ptr<DebugHandle> handle) : Panda(std::move(handle), cereal::PandaState::PandaType::CUATRO) {}
};
void test_debug_transport() {
  auto handle = std::make_unique<DebugHandle>(); auto *io = handle.get();
  DebugPanda panda(std::move(handle));
  io->payload = std::string(83, 'a') + "Temporary fault occurred: 0x00000004\r\n";
  const std::string expected = io->payload;
  CHECK(panda.serial_read() == expected);
  CHECK(io->calls == 3 && io->payload.empty() && io->writes == 0);
  io->calls = 0; CHECK(panda.serial_read().empty()); CHECK(io->calls == 1);
  io->calls = 0; io->endless = true;
  CHECK(panda.serial_read() == std::string(1024, 'x'));
  CHECK(io->calls == 16 && io->writes == 0);
  io->endless = false; io->error = -1; io->calls = 0;
  CHECK(panda.serial_read().empty()); CHECK(io->calls == 1);
  io->error = 65; io->calls = 0;
  CHECK(panda.serial_read().empty()); CHECK(io->calls == 1 && io->writes == 0);
}
int main() { return run_native_test(test_debug_transport); }
