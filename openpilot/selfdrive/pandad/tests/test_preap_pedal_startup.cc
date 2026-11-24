#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <memory>
#include <stdexcept>
#include <unistd.h>
#include <vector>
#include "common/tests/native_test.h"
#include "selfdrive/pandad/pandad.h"
#include "selfdrive/pandad/preap_pedal_wire.h"

struct NativePedal {
  void *library = nullptr;
  int (*hooks)(uint16_t, uint16_t);
  int (*mode)();
  int (*param)();
  void (*alternative)(int);
  void (*heartbeat)(bool);
  bool (*word)(uint16_t, uint16_t);
  unsigned int (*read)(uint8_t *);
  template<class T> T symbol(const char *name) {
    auto result = reinterpret_cast<T>(dlsym(library, name));
    if (result == nullptr) throw std::runtime_error(name);
    return result;
  }
  NativePedal() {
    const char *path = getenv("TESLA_PEDAL_LIBSAFETY");
    if (path == nullptr || (library = dlopen(path, RTLD_NOW | RTLD_LOCAL)) == nullptr)
      throw std::runtime_error("TESLA_PEDAL_LIBSAFETY must name the actual source-matched native library");
    hooks = symbol<decltype(hooks)>("set_safety_hooks");
    mode = symbol<decltype(mode)>("get_current_safety_mode");
    param = symbol<decltype(param)>("get_current_safety_param");
    alternative = symbol<decltype(alternative)>("set_alternative_experience");
    heartbeat = symbol<decltype(heartbeat)>("set_aol_test_heartbeat");
    word = symbol<decltype(word)>("tesla_preap_test_config_word");
    read = symbol<decltype(read)>("tesla_preap_test_config_read");
    heartbeat(false); CHECK(hooks(0, 0) == 0);
  }
  ~NativePedal() { heartbeat(false); hooks(0, 0); dlclose(library); }
};

class NativePedalHandle : public PandaCommsHandle {
public:
  NativePedal &native;
  bool old_firmware = false;
  std::vector<uint16_t> modes;
  size_t word_writes = 0;
  explicit NativePedalHandle(NativePedal &api) : native(api) { hw_serial = "actual-preap-native"; }
  int control_write(uint8_t request, uint16_t p1, uint16_t p2, unsigned int) override {
    if (request == 0xdc) { modes.push_back(p1); return native.hooks(p1, p2) == 0 ? 0 : -1; }
    if (request == 0xdf) { native.alternative(p1); return 0; }
    if (request == 0xb7) {
      ++word_writes;
      if (!old_firmware) (void)native.word(p1, p2);
      return 0;  // Like the board handler, write completion is not an admission ack.
    }
    return 0;
  }
  int control_read(uint8_t request, uint16_t, uint16_t, unsigned char *data, uint16_t length, unsigned int) override {
    if (request == 0xb8 && length == 48) return old_firmware ? 0 : static_cast<int>(native.read(data));
    if (request == 0xd2 && length == sizeof(health_t)) {
      health_t health{};
      health.safety_mode_pkt = native.mode(); health.safety_param_pkt = native.param();
      health.alternative_experience_pkt = 32;
      memcpy(data, &health, length); return length;
    }
    return -1;
  }
  int bulk_write(unsigned char, unsigned char *, int, unsigned int) override { return 0; }
  int bulk_read(unsigned char, unsigned char *, int, unsigned int) override { return 0; }
  void cleanup() override {}
};
class NativePedalPanda : public Panda {
public:
  explicit NativePedalPanda(std::unique_ptr<NativePedalHandle> h) :
    Panda(std::move(h), cereal::PandaState::PandaType::RED_PANDA, 0) {}
};

std::string serialized(capnp::MallocMessageBuilder &builder) {
  const auto words = capnp::messageToFlatArray(builder);
  const auto bytes = words.asBytes();
  return std::string(reinterpret_cast<const char *>(bytes.begin()), bytes.size());
}
std::string capsule(const std::string &cp_bytes, uint16_t bus, uint16_t word, float factor,
                    uint16_t version = 1, uint16_t kind = 5, bool wrong_hash = false) {
  capnp::MallocMessageBuilder message;
  auto context = message.initRoot<cereal::StarPilotCarControl::PreapPedalStartup>();
  context.setVersion(version); context.setKind(kind); context.setSafetyModel(39); context.setSafetyParam(word);
  context.setPedalBus(bus); context.setFactor(factor); context.setZero(0.0F); context.setSourceMonoTime(aol_monotonic_ns());
  auto hash = preap_sha256(cp_bytes);
  if (wrong_hash) hash[0] ^= 1;
  context.setCarParamsSha256(kj::arrayPtr(hash.data(), hash.size()));
  return serialized(message);
}

void independent_sha256_vectors() {
  const std::vector<std::pair<size_t, std::string>> vectors = {
    {0, "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"},
    {1, "6e340b9cffb37a989ca544e6bb780a2c78901d3fb33738768511a30617afa01d"},
    {3, "ae4b3280e56e2faf83f414a6e3dabe9d5fbe18976544c05fed121accb85b53fc"},
    {55, "463eb28e72f82e0a96c0a4cc53690c571281131f672aa229e0d45ae59b598b59"},
    {56, "da2ae4d6b36748f2a318f23e7ab1dfdf45acdc9d049bd80e59de82a60895f562"},
    {57, "2fe741af801cc238602ac0ec6a7b0c3a8a87c7fc7d7f02a3fe03d1c12eac4d8f"},
    {63, "29af2686fd53374a36b0846694cc342177e428d1647515f078784d69cdb9e488"},
    {64, "fdeab9acf3710362bd2658cdc9a29e8f9c757fcf9811603a8c447cd1d9151108"},
    {65, "4bfd2c8b6f1eec7a2afeb48b934ee4b2694182027e6d0fc075074f2fabb31781"},
    {119, "da18797ed7c3a777f0847f429724a2d8cd5138e6ed2895c3fa1a6d39d18f7ec6"},
    {120, "f52b23db1fbb6ded89ef42a23ce0c8922c45f25c50b568a93bf1c075420bbb7c"},
    {127, "92ca0fa6651ee2f97b884b7246a562fa71250fedefe5ebf270d31c546bfea976"},
    {128, "471fb943aa23c511f6f72f8d1652d9c880cfa392ad80503120547703e56a2be5"},
    {129, "5099c6a56203f9687f7d33f4bfdf576d31dc91f6b695ecea38b2770c87631135"},
    {1024, "785b0751fc2c53dc14a4ce3d800e69ef9ce1009eb327ccf458afe09c242c26c9"},
    {1048576, "fbbab289f7f94b25736c58be46a994c441fd02552cc6022352e3d86d2fab7c83"},
  };
  constexpr char hex[] = "0123456789abcdef";
  for (const auto &[size, expected] : vectors) {
    std::string input(size, '\0');
    for (size_t i = 0; i < size; ++i) input[i] = static_cast<char>(i & 255U);
    const auto hash = preap_sha256(input);
    std::string actual;
    for (uint8_t byte : hash) { actual += hex[byte >> 4]; actual += hex[byte & 15U]; }
    CHECK(actual == expected);
  }
}

void actual_panda_safety_to_native_startup() {
  independent_sha256_vectors();
  const char *old = getenv("OPENPILOT_PREFIX");
  const std::string original_prefix = old ? old : "";
  const std::string prefix = "test-preap-pedal-startup-" + std::to_string(getpid());
  setenv("OPENPILOT_PREFIX", prefix.c_str(), 1);
  Params params;
  struct Restore {
    Params &params; std::string previous;
    ~Restore() {
      for (const auto *key : {"CarParams", "TeslaPreAPPedalStartup", "FirmwareQueryDone", "ControlsReady", "ObdMultiplexingChanged"}) params.remove(key);
      if (previous.empty()) unsetenv("OPENPILOT_PREFIX"); else setenv("OPENPILOT_PREFIX", previous.c_str(), 1);
    }
  } restore{params, original_prefix};
  NativePedal native;
  auto device = std::make_unique<NativePedalHandle>(native);
  auto *handle = device.get();
  NativePedalPanda panda(std::move(device));
  for (const uint16_t bus : {0, 2}) for (const uint16_t word : {1, 3, 7}) {
    capnp::MallocMessageBuilder cp_message;
    auto cp = cp_message.initRoot<cereal::CarParams>();
    cp.setBrand("tesla"); cp.setCarFingerprint("TESLA_MODEL_S_PREAP"); cp.setWheelbase(2.96F); cp.setSteerRatio(15.0F);
    cp.setOpenpilotLongitudinalControl(true); cp.setPcmCruise(false); cp.setRadarUnavailable(word == 1);
    cp.setSteerAtStandstill(true); cp.setSteerControlType(cereal::CarParams::SteerControlType::ANGLE);
    cp.setTransmissionType(cereal::CarParams::TransmissionType::AUTOMATIC);
    cp.setFingerprintSource(cereal::CarParams::FingerprintSource::CAN); cp.setAlternativeExperience(32);
    auto safety = cp.initSafetyConfigs(1);
    safety[0].setSafetyModel(cereal::CarParams::SafetyModel::TESLA_PREAP); safety[0].setSafetyParam(word);
    const std::string cp_bytes = serialized(cp_message);
    params.put("CarParams", cp_bytes); params.putBool("FirmwareQueryDone", true); params.putBool("ControlsReady", true);
    params.put("TeslaPreAPPedalStartup", capsule(cp_bytes, bus, word, 1.0F));
    PandaSafety owner({&panda});
    owner.configureSafetyMode(true);
    CHECK(native.mode() == 39); CHECK(native.param() == word);
    auto expected = preap_pedal_words(params.get("TeslaPreAPPedalStartup"), cp_bytes, cp.asReader());
    CHECK(expected.has_value()); CHECK(panda.matches_preap_pedal(*expected));
    CHECK(!native.word(0, 1));  // Actual car-mode mutation guard, not a mock.
    // Native reset must force an actual new stage/echo instead of trusting cached CP.
    CHECK(native.hooks(0, 0) == 0);
    const auto before_reset = handle->word_writes;
    owner.configureSafetyMode(true);
    CHECK(native.mode() == 39); CHECK(handle->word_writes > before_reset);
    // A source/configuration restart withdraws before switching calibration.
    const auto before_change = handle->modes.size();
    params.put("TeslaPreAPPedalStartup", capsule(cp_bytes, bus, word, 2.0F));
    owner.configureSafetyMode(true);
    CHECK(handle->modes.size() > before_change); CHECK(handle->modes[before_change] == static_cast<uint16_t>(cereal::CarParams::SafetyModel::NO_OUTPUT));
    CHECK(native.mode() == 39);
    // Both wrong CP provenance and unknown versions/kinds must remain NO_OUTPUT.
    for (const auto &bad : {capsule(cp_bytes, bus, word, 1.0F, 1, 5, true),
                            capsule(cp_bytes, bus, word, 1.0F, 2, 5), capsule(cp_bytes, bus, word, 1.0F, 1, 4)}) {
      owner.configureSafetyMode(false); params.put("TeslaPreAPPedalStartup", bad);
      const auto before_bad = handle->word_writes;
      owner.configureSafetyMode(true);
      CHECK(native.mode() == static_cast<int>(cereal::CarParams::SafetyModel::NO_OUTPUT));
      CHECK(handle->word_writes == before_bad);
    }
    owner.configureSafetyMode(false);
    params.put("TeslaPreAPPedalStartup", capsule(cp_bytes, bus, word, 1.0F));
    handle->old_firmware = true;
    owner.configureSafetyMode(true);
    CHECK(native.mode() == static_cast<int>(cereal::CarParams::SafetyModel::NO_OUTPUT));
    handle->old_firmware = false;
  }
}
int main() { return run_native_test(actual_panda_safety_to_native_startup); }
