#pragma once
#include <array>
#include <cmath>
#include <cstring>
#include <optional>
#include <string>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include "selfdrive/pandad/preap_sha256.h"
#include "openpilot/cereal/gen/cpp/custom.capnp.h"
#include "openpilot/cereal/messaging/messaging.h"
#include "selfdrive/pandad/aol_protocol.h"

using PreapPedalWords = std::array<uint16_t, 24>;

inline std::string read_preap_pedal_capsule(const std::string &path) {
  const int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_NOFOLLOW);
  if (fd < 0) return {};
  struct stat before{}, after{}, current{};
  std::array<char, 512> bytes{};
  const bool sized = fstat(fd, &before) == 0 && S_ISREG(before.st_mode) && before.st_size >= 16 && before.st_size <= 512;
  const ssize_t count = sized ? read(fd, bytes.data(), static_cast<size_t>(before.st_size)) : -1;
  const bool stable = fstat(fd, &after) == 0 && lstat(path.c_str(), &current) == 0 && S_ISREG(current.st_mode) &&
                      before.st_dev == after.st_dev && before.st_ino == after.st_ino &&
                      before.st_dev == current.st_dev && before.st_ino == current.st_ino &&
                      before.st_size == after.st_size && before.st_size == current.st_size &&
                      before.st_mtime == after.st_mtime && before.st_mtime == current.st_mtime;
  close(fd);
  if (!sized || !stable || count != before.st_size) return {};
  return std::string(bytes.data(), static_cast<size_t>(count));
}

inline std::optional<PreapPedalWords> preap_pedal_words(const std::string &raw,
                                                       const std::string &cp_bytes,
                                                       cereal::CarParams::Reader cp) {
  if (raw.size() < 16 || raw.size() > 512 || raw.size() % 8 || cp_bytes.empty() || cp_bytes.size() > 1024 * 1024 ||
      cp.getBrand() != "tesla" || cp.getCarFingerprint() != "TESLA_MODEL_S_PREAP" ||
      cp.getPassive() || cp.getNotCar() || cp.getDashcamOnly() || !cp.getOpenpilotLongitudinalControl() ||
      cp.getPcmCruise() || cp.getAlphaLongitudinalAvailable() || cp.getFlags() != 0 ||
      (cp.getAlternativeExperience() != 0 && cp.getAlternativeExperience() != 32) ||
      cp.getSafetyConfigs().size() != 1 || cp.getSafetyConfigs()[0].getSafetyModel() != cereal::CarParams::SafetyModel::TESLA_PREAP ||
      cp.getTransmissionType() == cereal::CarParams::TransmissionType::MANUAL ||
      cp.getFingerprintSource() == cereal::CarParams::FingerprintSource::FIXED ||
      cp.getSteerControlType() != cereal::CarParams::SteerControlType::ANGLE || !cp.getSteerAtStandstill() ||
      !std::isfinite(cp.getWheelbase()) || std::fabs(cp.getWheelbase() - 2.96F) > 1e-6F || cp.getSteerRatio() != 15.0F) return std::nullopt;
  const uint16_t word = cp.getSafetyConfigs()[0].getSafetyParam();
  if ((word != 1 && word != 3 && word != 7) || cp.getRadarUnavailable() != (word == 1)) return std::nullopt;
  uint32_t segments = 0, words = 0;
  memcpy(&segments, raw.data(), 4);
  memcpy(&words, raw.data() + 4, 4);
  if (segments != 0 || words > 63 || raw.size() != 8U + static_cast<size_t>(words) * 8U) return std::nullopt;
  try {
    AlignedBuffer aligned;
    capnp::ReaderOptions options;
    options.traversalLimitInWords = 63;
    options.nestingLimit = 4;
    capnp::FlatArrayMessageReader reader(aligned.align(raw.data(), raw.size()), options);
    const auto context = reader.getRoot<cereal::StarPilotCarControl::PreapPedalStartup>();
    const auto hash = context.getCarParamsSha256();
    const auto actual_hash = preap_sha256(cp_bytes);
    const float factor = context.getFactor(), zero = context.getZero();
    const uint16_t bus = context.getPedalBus();
    if (context.getVersion() != 1 || context.getKind() != 5 || context.getSafetyModel() != 39 ||
        context.getSafetyParam() != word || hash.size() != 32 || memcmp(hash.begin(), actual_hash.data(), 32) != 0 ||
        context.getSourceMonoTime() == 0 || context.getSourceMonoTime() > aol_monotonic_ns() ||
        !std::isfinite(factor) || std::fabs(factor) <= 1e-6F || !std::isfinite(zero) || (bus != 0 && bus != 2)) return std::nullopt;
    PreapPedalWords result{1, bus, word, 0xD051};
    uint32_t fbits = 0, zbits = 0;
    memcpy(&fbits, &factor, 4); memcpy(&zbits, &zero, 4);
    result[4] = fbits & 65535; result[5] = fbits >> 16;
    result[6] = zbits & 65535; result[7] = zbits >> 16;
    for (size_t i = 0; i < 16; ++i) result[8 + i] = hash[2 * i] | (static_cast<uint16_t>(hash[2 * i + 1]) << 8);
    return result;
  } catch (const kj::Exception &) {
    return std::nullopt;
  }
}
