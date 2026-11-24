// Private bench transport. Compile current production pandad into this TU;
// never link panda_usb.cc, spi.cc, libpanda.a, or libusb.
#include <dlfcn.h>
#include <sys/prctl.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include "common/tests/native_test.h"
#include "common/hardware/hw.h"

// Virtualize the remaining physical peripheral write. Inherit real hardware
// identity/PC() so the production main retains its real FIFO/core setup.
#if __COMMA_HARDWARE__
class BenchHardware : public HardwareComma {
public:
  static void set_ir_power(int) {}
};
#else
class BenchHardware : public HardwarePC {
public:
  static void set_ir_power(int) {}
};
#endif
#undef Hardware
#define Hardware BenchHardware
#include "selfdrive/pandad/pandad.cc"

namespace {
constexpr const char *SERIAL = "bench-virtual-owner";
struct Frame { uint32_t address; uint8_t bus; std::vector<uint8_t> data; };
struct TransportSource { uint8_t bus; bool returned; bool rejected; };
// Match Panda::unpack_can_buffer exactly. CP1 physical buses are 0..3;
// historical TX return/rejection markers are separate transport categories.
static_assert(CAN_RETURNED_BUS_OFFSET == 0x80U && CAN_REJECTED_BUS_OFFSET == 0xC0U);
TransportSource transport_source(uint8_t source) {
  if (source < 4) return {source, false, false};
  if (source >= CAN_RETURNED_BUS_OFFSET && source < CAN_RETURNED_BUS_OFFSET + 4)
    return {static_cast<uint8_t>(source - CAN_RETURNED_BUS_OFFSET), true, false};
  if (source >= CAN_REJECTED_BUS_OFFSET && source < CAN_REJECTED_BUS_OFFSET + 4)
    return {static_cast<uint8_t>(source - CAN_REJECTED_BUS_OFFSET), false, true};
  CHECK(false); return {}; // Unknown source / additional Panda bus is forbidden.
}
struct Batch { uint64_t offset; std::vector<Frame> frames; };
std::vector<Batch> trace;
std::string library_path, release_path, receipt_path;
uint64_t duration_ns = 0, start_ns = 0;
size_t cursor = 0;
std::mutex native_lock;
uint64_t rx_count = 0, rx_rejected = 0, tx_count = 0, tx_accepted = 0, active_tx_accepted = 0;
uint64_t max_feed_late_ns = 0;
uint64_t trace_rx = 0, trace_returned = 0, trace_rejected = 0;
uint64_t playback_returned = 0, playback_rejected = 0;
struct AcceptedTxSample {
  uint64_t index, mono_ns;
  uint32_t address;
  uint8_t bus, permission;
  bool controls;
  std::string payload_hex;
};
std::vector<AcceptedTxSample> granted_tx_samples, actuator_hint_tx_samples;
AcceptedTxSample tx_sample(const Frame &frame, uint8_t permission, bool controls) {
  static constexpr char HEX[] = "0123456789abcdef";
  std::string payload; payload.reserve(frame.data.size() * 2);
  for (const auto byte : frame.data) { payload.push_back(HEX[byte >> 4]); payload.push_back(HEX[byte & 15]); }
  return {tx_count, aol_monotonic_ns(), frame.address, frame.bus, permission, controls, std::move(payload)};
}
void write_tx_samples(std::ostream &out, const std::vector<AcceptedTxSample> &samples) {
  out << '[';
  for (size_t i = 0; i < samples.size(); ++i) {
    const auto &sample = samples[i]; if (i) out << ',';
    out << "{\"nativeAccepted\":true,\"txIndex\":" << sample.index << ",\"monoNs\":" << sample.mono_ns
        << ",\"address\":" << sample.address << ",\"bus\":" << (unsigned)sample.bus
        << ",\"permissionMask\":" << (unsigned)sample.permission
        << ",\"controlsAllowed\":" << (sample.controls ? "true" : "false")
        << ",\"payloadHex\":\"" << sample.payload_hex << "\"}";
  }
  out << ']';
}

template <typename T> T read(std::ifstream &in) {
  T value{}; in.read(reinterpret_cast<char *>(&value), sizeof(value)); CHECK(in.good()); return value;
}
void load_trace(const char *path) {
  std::ifstream in(path, std::ios::binary); CHECK(in.good());
  CHECK(read<uint64_t>(in) == 0x3143415254425053ULL); // SPBTRAC1, little endian
  duration_ns = read<uint64_t>(in);
  CHECK(duration_ns >= 1000000000ULL && duration_ns <= 120000000000ULL);
  const auto count = read<uint32_t>(in); CHECK(count > 0 && count <= 24000);
  uint64_t previous = 0;
  for (uint32_t i = 0; i < count; ++i) {
    Batch batch; batch.offset = read<uint64_t>(in);
    CHECK(batch.offset >= previous && batch.offset < duration_ns); previous = batch.offset;
    const auto frames = read<uint32_t>(in); CHECK(frames <= 1024);
    for (uint32_t n = 0; n < frames; ++n) {
      Frame frame; frame.address = read<uint32_t>(in); frame.bus = read<uint8_t>(in);
      const auto length = read<uint8_t>(in); CHECK(length <= 64);
      const auto source = transport_source(frame.bus);
      trace_rx += !source.returned && !source.rejected;
      trace_returned += source.returned; trace_rejected += source.rejected;
      frame.data.resize(length); in.read(reinterpret_cast<char *>(frame.data.data()), length); CHECK(in.good());
      batch.frames.push_back(std::move(frame));
    }
    size_t bytes = 0; for (const auto &frame : batch.frames) bytes += sizeof(can_header) + frame.data.size();
    CHECK(bytes <= RECV_SIZE);
    trace.push_back(std::move(batch));
  }
  CHECK(in.peek() == std::ifstream::traits_type::eof());
}
CANPacket_t packet(const Frame &frame) {
  const auto source = transport_source(frame.bus);
  CANPacket_t result{}; result.addr = frame.address; result.bus = source.bus;
  result.returned = source.returned; result.rejected = source.rejected;
  result.fd = frame.data.size() > 8; result.extended = frame.address >= 0x800;
  bool found = false;
  for (uint8_t i = 0; i < std::size(dlc_to_len); ++i) if (dlc_to_len[i] == frame.data.size()) {
    result.data_len_code = i; found = true; break;
  }
  CHECK(found); memcpy(result.data, frame.data.data(), frame.data.size()); return result;
}
struct Native {
  void *library;
  int (*set_hooks)(uint16_t, uint16_t); void (*init)(); void (*timer)(uint32_t); void (*tick)();
  void (*set_ae)(int); void (*heartbeat)(bool); void (*request)(uint8_t);
  bool (*rx)(CANPacket_t *); bool (*tx)(CANPacket_t *); bool (*controls)(); bool (*relay)();
  int (*mode)(); int (*param)(); uint8_t (*permission)(); uint8_t (*mask)();
  int *ae; bool *rx_invalid; bool *optional; void **policy;
  uint64_t last_tick = 0;
  template <typename T> T symbol(const char *name) {
    auto ptr = dlsym(library, name); CHECK(ptr != nullptr); return reinterpret_cast<T>(ptr);
  }
  Native() {
    library = dlopen(library_path.c_str(), RTLD_NOW | RTLD_LOCAL); CHECK(library != nullptr);
    set_hooks = symbol<decltype(set_hooks)>("set_safety_hooks"); init = symbol<decltype(init)>("init_tests");
    timer = symbol<decltype(timer)>("set_timer"); tick = symbol<decltype(tick)>("safety_tick");
    set_ae = symbol<decltype(set_ae)>("set_alternative_experience");
    heartbeat = symbol<decltype(heartbeat)>("set_aol_test_heartbeat"); request = symbol<decltype(request)>("aol_set_host_request");
    rx = symbol<decltype(rx)>("safety_rx_hook"); tx = symbol<decltype(tx)>("safety_tx_hook");
    controls = symbol<decltype(controls)>("get_controls_allowed"); relay = symbol<decltype(relay)>("get_relay_malfunction");
    mode = symbol<decltype(mode)>("get_current_safety_mode"); param = symbol<decltype(param)>("get_current_safety_param");
    permission = symbol<decltype(permission)>("aol_get_permission_mask"); mask = symbol<decltype(mask)>("aol_get_request_mask");
    ae = symbol<int *>("alternative_experience"); rx_invalid = symbol<bool *>("safety_rx_checks_invalid");
    optional = symbol<bool *>("aol_optional_release_capable"); policy = symbol<void **>("aol_policy");
    set_ae(0); CHECK(set_hooks((uint16_t)cereal::CarParams::SafetyModel::NO_OUTPUT, 0) == 0); init();
  }
  void clock() {
    const auto now = aol_monotonic_ns();
    timer(static_cast<uint32_t>(now / 1000ULL)); // Actual clock; native timer wrap is normal.
    if (last_tick && now - last_tick >= 1000000000ULL) tick();
    if (!last_tick || now - last_tick >= 1000000000ULL) last_tick = now;
  }
  aol_safety_health_t status() {
    return {AOL_SAFETY_PROTOCOL_MAGIC, AOL_SAFETY_PROTOCOL_VERSION, mask(), permission(),
            static_cast<uint8_t>(mode()), static_cast<uint16_t>(param()),
            static_cast<uint8_t>(*policy ? (1U | (*optional ? 2U : 0U)) : 0U)};
  }
  ~Native() { set_ae(0); set_hooks((uint16_t)cereal::CarParams::SafetyModel::NO_OUTPUT, 0); init(); dlclose(library); }
};
std::unique_ptr<Native> native;

int write_control(uint8_t command, uint16_t p1, uint16_t p2) {
  std::lock_guard lock(native_lock); native->clock();
  if (command == 0xdc) {
    const int ae = *native->ae;
    CHECK(native->set_hooks(p1, p2) == 0); native->init(); native->set_ae(ae); native->clock();
  } else if (command == 0xdf) native->set_ae(p1);
  else if (command == 0xf3) native->heartbeat(p1 != 0);
  else if (command == 0xf5) native->request(p1);
  // Other setup/peripheral requests terminate here, without physical I/O.
  return 0;
}
int read_control(uint8_t command, unsigned char *data, uint16_t length) {
  std::lock_guard lock(native_lock); native->clock();
  if (command == 0xc1 && length == 1) { data[0] = (uint8_t)cereal::PandaState::PandaType::RED_PANDA; return 1; }
  if ((command == 0xd3 || command == 0xd4) && length == 64) {
    const auto fw = util::read_file("../../../panda/board/obj/panda_h7.bin.signed"); CHECK(fw.size() >= 128);
    memcpy(data, fw.data() + fw.size() - 128 + (command == 0xd4 ? 64 : 0), 64); return 64;
  }
  if (command == 0xd2 && length == sizeof(health_t)) {
    health_t state{}; state.flags_pkt = HEALTH_FLAG_IGNITION_LINE;
    if (native->controls()) state.flags_pkt |= HEALTH_FLAG_CONTROLS_ALLOWED;
    if (*native->rx_invalid) state.flags_pkt |= HEALTH_FLAG_SAFETY_RX_CHECKS_INVALID;
    state.safety_mode_pkt = native->mode(); state.safety_param_pkt = native->param(); state.alternative_experience_pkt = *native->ae;
    if (native->relay()) state.faults_pkt |= 1U;
    memcpy(data, &state, sizeof(state)); return sizeof(state);
  }
  if (command == 0xd5 && length == sizeof(aol_safety_health_t)) {
    const auto state = native->status(); memcpy(data, &state, sizeof(state)); return sizeof(state);
  }
  if ((command == 0xc2 && length == sizeof(can_health_t)) || (command == 0xb2 && length == 2)) {
    memset(data, 0, length); return length;
  }
  return 0;
}
int write_bulk(unsigned char endpoint, unsigned char *data, int length) {
  CHECK(endpoint == 3); std::lock_guard lock(native_lock); native->clock();
  for (int pos = 0; pos < length;) {
    CHECK(pos + (int)sizeof(can_header) <= length); can_header header{}; memcpy(&header, data + pos, sizeof(header));
    const auto size = dlc_to_len[header.data_len_code]; CHECK(pos + (int)sizeof(header) + size <= length);
    uint8_t checksum = 0; for (size_t i = 0; i < sizeof(header) + size; ++i) checksum ^= data[pos + i]; CHECK(checksum == 0);
    Frame frame{header.addr, header.bus, {data + pos + sizeof(header), data + pos + sizeof(header) + size}};
    auto msg = packet(frame); const bool accepted = native->tx(&msg); ++tx_count; tx_accepted += accepted;
    const uint8_t permission = native->permission(); const bool controls = native->controls();
    const bool granted = (permission & 3U) != 0 || controls;
    active_tx_accepted += accepted && granted;
    if (accepted && granted && granted_tx_samples.size() < 16)
      granted_tx_samples.push_back(tx_sample(frame, permission, controls));
    // Bounded observer hint only: Ioniq LKAS_ALT/LFA ActToiSta is LE bit52|2.
    // Root must independently decode the full recorded payload and CP identity.
    if (accepted && (frame.address == 0x110 || frame.address == 0x12A) && frame.data.size() >= 7 &&
        ((frame.data[6] >> 4) & 3U) != 0 && actuator_hint_tx_samples.size() < 16)
      actuator_hint_tx_samples.push_back(tx_sample(frame, permission, controls));
    pos += sizeof(header) + size;
  }
  return length;
}
int read_bulk(unsigned char endpoint, unsigned char *data, int length) {
  CHECK(endpoint == 0x81);
  if (!start_ns) {
    const auto deadline = aol_monotonic_ns() + 15000000000ULL;
    while (!do_exit && !std::filesystem::exists(release_path)) {
      CHECK(aol_monotonic_ns() < deadline); std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::ifstream release(release_path); start_ns = read<uint64_t>(release); CHECK(start_ns > 0);
  }
  const auto now = aol_monotonic_ns();
  if (now >= start_ns + duration_ns) { do_exit = true; return 0; }
  if (now < start_ns || cursor >= trace.size() || trace[cursor].offset > now - start_ns) return 0;
  std::lock_guard lock(native_lock); native->clock();
  int pos = 0;
  // Keep due CAN in source order, including overdue batches. Never reset the
  // source anchor or invent a healthy replacement for missed input.
  while (cursor < trace.size() && trace[cursor].offset <= now - start_ns) {
    size_t bytes = 0; for (const auto &frame : trace[cursor].frames) bytes += sizeof(can_header) + frame.data.size();
    CHECK(bytes <= static_cast<size_t>(length)); if (pos + bytes > static_cast<size_t>(length)) break;
    max_feed_late_ns = std::max(max_feed_late_ns, now - start_ns - trace[cursor].offset);
    for (const auto &frame : trace[cursor].frames) {
      const auto source = transport_source(frame.bus);
      auto msg = packet(frame);
      if (!source.returned && !source.rejected) {
        ++rx_count; rx_rejected += !native->rx(&msg);
      } else {
        // A recorded TX echo/rejection is transport history, never a new
        // vehicle RX packet or evidence for native health/permission.
        playback_returned += source.returned; playback_rejected += source.rejected;
      }
      can_header header{}; header.addr = frame.address; header.bus = source.bus;
      header.returned = source.returned; header.rejected = source.rejected;
      header.data_len_code = msg.data_len_code; header.extended = msg.extended;
      memcpy(data + pos, &header, sizeof(header)); memcpy(data + pos + sizeof(header), frame.data.data(), frame.data.size());
      uint8_t checksum = 0; for (size_t i = 0; i < sizeof(header) + frame.data.size(); ++i) checksum ^= data[pos + i];
      header.checksum = checksum; memcpy(data + pos, &header, sizeof(header)); pos += sizeof(header) + frame.data.size();
    }
    ++cursor;
  }
  return pos;
}
void opened(PandaCommsHandle *handle, const std::string &serial) { CHECK(serial == SERIAL); handle->hw_serial = serial; }
} // namespace

PandaUsbHandle::PandaUsbHandle(std::string serial) { opened(this, serial); }
PandaUsbHandle::~PandaUsbHandle() { cleanup(); }
void PandaUsbHandle::cleanup() {}
std::vector<std::string> PandaUsbHandle::list() { return {SERIAL}; }
int PandaUsbHandle::control_write(uint8_t r, uint16_t p1, uint16_t p2, unsigned int) { return write_control(r, p1, p2); }
int PandaUsbHandle::control_read(uint8_t r, uint16_t, uint16_t, unsigned char *d, uint16_t n, unsigned int) { return read_control(r, d, n); }
int PandaUsbHandle::bulk_write(unsigned char e, unsigned char *d, int n, unsigned int) { return write_bulk(e, d, n); }
int PandaUsbHandle::bulk_read(unsigned char e, unsigned char *d, int n, unsigned int) { return read_bulk(e, d, n); }
PandaSpiHandle::PandaSpiHandle(std::string) { throw std::runtime_error("SPI unavailable in private bench"); }
PandaSpiHandle::~PandaSpiHandle() {}
void PandaSpiHandle::cleanup() {}
std::vector<std::string> PandaSpiHandle::list() { return {}; }
int PandaSpiHandle::control_write(uint8_t, uint16_t, uint16_t, unsigned int) { CHECK(false); return -1; }
int PandaSpiHandle::control_read(uint8_t, uint16_t, uint16_t, unsigned char *, uint16_t, unsigned int) { CHECK(false); return -1; }
int PandaSpiHandle::bulk_write(unsigned char, unsigned char *, int, unsigned int) { CHECK(false); return -1; }
int PandaSpiHandle::bulk_read(unsigned char, unsigned char *, int, unsigned int) { CHECK(false); return -1; }

#define main production_pandad_entry
#include "selfdrive/pandad/main.cc"
#undef main

int main(int argc, char **argv) {
  CHECK(argc == 5);
  CHECK(getenv("SP_PRIVATE_NATIVE_BENCH") && std::string(getenv("SP_PRIVATE_NATIVE_BENCH")) == "1");
  CHECK(getenv("OPENPILOT_PREFIX") && std::string(getenv("OPENPILOT_PREFIX")).rfind("replay-", 0) == 0);
  CHECK(getenv("PARAMS_ROOT") && std::string(getenv("PARAMS_ROOT")).rfind("/data/params", 0) != 0);
  for (const char *name : {"REPLAY", "SIMULATION", "STARTED", "FAKESEND", "BOARDD_LOOPBACK", "BOARDD_SKIP_FW_CHECK"}) CHECK(!getenv(name));
  for (const char *name : {"PANDAD_MAXOUT", "CEREAL_FAKE", "STARPILOT_PANDA_FIRMWARE_MAP", "STARPILOT_PANDA_FIRMWARE_NAME"}) CHECK(!getenv(name));
  CHECK(!getenv("AOL_REPLAY_RUNTIME") || std::string(getenv("AOL_REPLAY_RUNTIME")) == "0");
  const auto parent = getppid(); CHECK(parent > 1);
  CHECK(prctl(PR_SET_PDEATHSIG, SIGKILL) == 0 && getppid() == parent);
  library_path = argv[1]; release_path = argv[3]; receipt_path = argv[4]; load_trace(argv[2]);
  native = std::make_unique<Native>(); setenv("NO_FAN_CONTROL", "1", 1);
  std::string command = "private-virtual-pandad", serial = SERIAL;
  char *args[] = {command.data(), serial.data()}; const int result = production_pandad_entry(2, args);
  std::ofstream out(receipt_path); CHECK(out.good());
  out << "{\"physicalCanIo\":false,\"actualProductionPandad\":true,\"actualNativeSafety\":true,"
      << "\"nativeTimer\":\"host CLOCK_MONOTONIC microseconds\",\"traceBatches\":" << trace.size()
      << ",\"consumedBatches\":" << cursor << ",\"rx\":" << rx_count << ",\"rxRejected\":" << rx_rejected
      << ",\"traceIncomingFrames\":" << trace_rx << ",\"traceHistoricalReturnedFrames\":" << trace_returned
      << ",\"traceHistoricalRejectedFrames\":" << trace_rejected
      << ",\"publishedHistoricalReturnedFrames\":" << playback_returned
      << ",\"publishedHistoricalRejectedFrames\":" << playback_rejected
      << ",\"tx\":" << tx_count << ",\"txAccepted\":" << tx_accepted << ",\"txAcceptedWhileNativeGranted\":" << active_tx_accepted
      << ",\"maxFeedLateNs\":" << max_feed_late_ns << ",\"permissionMask\":" << (unsigned)native->permission()
      << ",\"requestMask\":" << (unsigned)native->mask() << ",\"nativeRxInvalid\":" << (*native->rx_invalid ? "true" : "false")
      << ",\"acceptedGrantedTxSamples\":";
  write_tx_samples(out, granted_tx_samples);
  out << ",\"acceptedIoniqActuatorHintSamples\":"; write_tx_samples(out, actuator_hint_tx_samples); out << "}\n";
  native.reset(); return result;
}
