#pragma once

// Descriptive log metadata only. A load receipt does not establish output health
// or grant any control authority. Never open/hash a model artifact here.
#include <algorithm>
#include <cerrno>
#include <limits>
#include <cstdint>
#include <fcntl.h>
#include <map>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include "json11/json11.hpp"

namespace logger_model_metadata {

using Fields = std::map<std::string, std::string>;
constexpr char PREFIX[] = "starpilot.model.";
constexpr size_t MAX_CATALOG_BYTES = 2 * 1024 * 1024;

inline bool same_file(const struct stat &a, const struct stat &b) {
  return a.st_dev == b.st_dev && a.st_ino == b.st_ino && a.st_size == b.st_size &&
         a.st_mtime == b.st_mtime && a.st_ctime == b.st_ctime &&
#ifdef __APPLE__
         a.st_mtimespec.tv_nsec == b.st_mtimespec.tv_nsec && a.st_ctimespec.tv_nsec == b.st_ctimespec.tv_nsec;
#else
         a.st_mtim.tv_nsec == b.st_mtim.tv_nsec && a.st_ctim.tv_nsec == b.st_ctim.tv_nsec;
#endif
}

// Nonblocking/no-follow regular-file reads, bounded even for zero-sized procfs
// files. Atomic replacement or a changed file is unavailable for this snapshot.
inline std::string read_bounded(const std::string &path, size_t limit, bool private_parent = false) {
  int directory = AT_FDCWD;
  std::string name = path;
  if (private_parent) {
    const auto slash = path.find_last_of('/');
    if (slash == std::string::npos) return {};
    directory = open(path.substr(0, slash).c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
    if (directory < 0) return {};
    struct stat parent{};
    if (fstat(directory, &parent) != 0 || parent.st_uid != getuid() || (parent.st_mode & 0777) != 0700) {
      close(directory);
      return {};
    }
    name = path.substr(slash + 1);
  }
  const int fd = openat(directory, name.c_str(), O_RDONLY | O_NONBLOCK | O_NOFOLLOW | O_CLOEXEC);
  struct stat before{}, after{}, named{};
  bool ok = fd >= 0 && fstat(fd, &before) == 0 && S_ISREG(before.st_mode) &&
            before.st_size >= 0 && static_cast<uint64_t>(before.st_size) <= limit;
  std::string raw;
  char buffer[4096];
  while (ok) {
    const ssize_t count = read(fd, buffer, std::min(sizeof(buffer), limit + 1 - raw.size()));
    if (count < 0 && errno == EINTR) continue;
    if (count < 0) { ok = false; break; }
    if (count == 0) break;
    raw.append(buffer, static_cast<size_t>(count));
    if (raw.size() > limit) { ok = false; break; }
  }
  ok = ok && fstat(fd, &after) == 0 && fstatat(directory, name.c_str(), &named, AT_SYMLINK_NOFOLLOW) == 0 &&
       same_file(before, after) && same_file(before, named);
  if (fd >= 0) close(fd);
  if (directory != AT_FDCWD) close(directory);
  return ok ? raw : std::string{};
}

inline json11::Json document(const std::string &raw) {
  std::string error;
  const auto value = json11::Json::parse(raw, error);
  return error.empty() && value.is_object() ? value : json11::Json{};
}

inline bool identifier(const std::string &value) {
  return !value.empty() && value.size() <= 64 &&
         std::all_of(value.begin(), value.end(), [](unsigned char c) {
           return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '_';
         });
}

inline bool display_name(const std::string &value) {
  return !value.empty() && value.size() <= 512 &&
         std::none_of(value.begin(), value.end(), [](unsigned char c) { return c < 32 || c == 127; });
}

struct Entry { std::string name, version; bool external; };
using Catalog = std::map<std::string, Entry>;

inline Catalog extend_catalog(const Catalog &base, const std::string &path) {
  const auto data = document(read_bounded(path, MAX_CATALOG_BYTES));
  if (data["generation"].string_value() != "v26" ||
      data["compiler_revision"].string_value() != "9d0446a4ba8a532c8b674fb6ad795af015cd9dcf" ||
      data["artifact_abi"].string_value() != "tinygrad_single_v1_arena" ||
      !data["models"].is_array() || data["models"].array_items().size() > 2048) return base;
  Catalog updates;
  for (const auto &row : data["models"].array_items()) {
    const std::string id = row["id"].string_value(), name = row["name"].string_value(), version = row["version"].string_value();
    const auto &gpu = row["uses_external_gpu"];
    if (!identifier(id) || id == "bundled-current" || !display_name(name) || updates.count(id) ||
        (!gpu.is_null() && !gpu.is_bool()) ||
        (version != "v8" && version != "v9" && version != "v10" && version != "v11" && version != "v12" &&
         version != "v13" && version != "v14" && version != "v15" && version != "v16")) return base;
    const auto previous = base.find(id);
    if (previous != base.end() && (previous->second.version != version || previous->second.external != gpu.bool_value())) return base;
    updates[id] = {name, version, gpu.bool_value()};
  }
  Catalog result = base;
  result.insert(updates.begin(), updates.end());
  for (const auto &[id, entry] : updates) result[id] = entry;
  return result;
}

// The flat receipt writer uses unsigned JSON integer tokens. Parse those from
// the original bytes so uint64 nanoseconds never pass through json11's double.
inline uint64_t receipt_integer(const std::string &raw, const std::string &key) {
  const auto token = '"' + key + '"';
  auto at = raw.find(token);
  if (at == std::string::npos) return 0;
  at = raw.find(':', at + token.size());
  if (at == std::string::npos) return 0;
  at = raw.find_first_not_of(" \t\r\n", at + 1);
  uint64_t value = 0;
  size_t digits = 0;
  while (at < raw.size() && raw[at] >= '0' && raw[at] <= '9') {
    const unsigned digit = raw[at++] - '0';
    if (value > (std::numeric_limits<uint64_t>::max() - digit) / 10) return 0;
    value = value * 10 + digit;
    ++digits;
  }
  at = raw.find_first_not_of(" \t\r\n", at);
  return digits && at != std::string::npos && (raw[at] == ',' || raw[at] == '}') ? value : 0;
}

inline uint64_t process_ticks(const std::string &raw) {
  const auto end = raw.rfind(')');
  if (end == std::string::npos) return 0;
  std::istringstream input(raw.substr(end + 1));
  std::string state, field;
  if (!(input >> state) || state == "Z" || state == "X" || state == "x") return 0;
  for (int i = 0; i < 18; ++i) if (!(input >> field)) return 0;
  uint64_t ticks = 0;
  if (!(input >> ticks)) return 0;
  return ticks;
}

inline bool receipt_schema(const std::string &raw, const json11::Json &data) {
  static const std::vector<std::string> keys = {"version", "pid", "processStartTicks", "loadedMonoNs",
                                               "modelId", "variant", "artifactSha256", "fallbackReason"};
  // The authoritative writer emits ASCII flat fields without escapes. Reject
  // escapes and duplicate keys rather than accepting a last-key-wins receipt.
  if (!data.is_object() || data.object_items().size() != keys.size() || raw.find('\\') != std::string::npos) return false;
  for (const auto &key : keys) {
    if (!data.object_items().count(key)) return false;
    const auto token = '"' + key + '"';
    size_t at = 0, count = 0;
    while ((at = raw.find(token, at)) != std::string::npos) {
      at += token.size();
      const auto next = raw.find_first_not_of(" \t\r\n", at);
      if (next != std::string::npos && raw[next] == ':') ++count;
    }
    if (count != 1) return false;
  }
  return receipt_integer(raw, "version") == 1;
}

inline Fields collect(const std::string &bundled_catalog, const std::string &model_root,
                      const std::string &shm_root, const std::string &prefix,
                      const std::string &proc_root, uint64_t snapshot_mono_ns) {
  const Catalog bundled = extend_catalog({}, bundled_catalog);
  Catalog entries = extend_catalog(bundled, model_root + "/catalog.json");
  // This compatibility alias is the static identity defined in catalog.py.
  entries["bundled-current"] = {"Regret Driven Framework V4", "v15", false};
  const auto prefs = document(read_bounded(model_root + "/preferences.json", 16384));
  std::string small = "rdf43", big;
  Fields request_sources;
  for (const std::string profile : {"small", "big"}) {
    const std::string field = profile == "small" ? "requestedSmallSource" : "requestedChestnutSource";
    request_sources[field] = "default; saved profile unavailable";
    if (!prefs.object_items().count(profile)) continue;
    const auto &saved = prefs[profile];
    const std::string selected = saved.string_value();
    // Log raw saved intent even if the catalog cannot resolve it. This is not
    // manager.preferences()'s validated effective/default runtime selection.
    (profile == "small" ? small : big) = saved.is_string() && (identifier(selected) || selected.empty()) ? selected : "invalid-saved-value";
    request_sources[field] = "raw saved profile; effective runtime selection not inferred";
  }
  Fields fields = {{"snapshotMonoTime", std::to_string(snapshot_mono_ns)}, {"clockDomain", "CLOCK_MONOTONIC"},
                   {"requestedSmallId", small}, {"requestedChestnutId", big.empty() ? "none" : big},
                   {"requestRandomizer", prefs["randomizer"].bool_value() ? "enabled" : "disabled"},
                   {"requestSemantics", "Raw saved profiles or explicit defaults; executable selection may validate, randomize, normalize aliases or fall back"},
                   {"receiptStatus", "unavailable at snapshot"},
                   {"lastConfirmedLoadedName", "unavailable"}, {"lastConfirmedLoadedId", "unavailable"},
                   {"currentIdentityStatus", "not established by initData; use modelIdentity transition events"},
                   {"semantics", "Saved requests and last-confirmed loads are distinct; receipt invalidation is best-effort, and a receipt is not current-output or control-authority proof"}};
  fields.insert(request_sources.begin(), request_sources.end());
  for (const auto &[field, id] : Fields{{"requestedSmallName", small}, {"requestedChestnutName", big}}) {
    const auto entry = entries.find(id);
    fields[field] = id.empty() ? "none" : entry == entries.end() ? "unresolved" : entry->second.name;
  }
  const bool valid_prefix = !prefix.empty() && prefix.size() <= 32 && std::all_of(prefix.begin(), prefix.end(), [](unsigned char c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
  });
  if (!valid_prefix || !snapshot_mono_ns) return fields;
  const std::string path = shm_root + "/starpilot_modeld_" + prefix + "/load-v1.json";
  const auto raw = read_bounded(path, 1024, true);
  if (raw.empty()) return fields;
  const auto load = document(raw);
  fields["receiptStatus"] = "rejected receipt at snapshot";
  if (!receipt_schema(raw, load)) return fields;
  const uint64_t pid = receipt_integer(raw, "pid"), ticks = receipt_integer(raw, "processStartTicks"), loaded = receipt_integer(raw, "loadedMonoNs");
  const std::string id = load["modelId"].string_value(), variant = load["variant"].string_value(), digest = load["artifactSha256"].string_value();
  const auto entry = entries.find(id);
  const auto &reason = load["fallbackReason"];
  const std::string fallback = reason.string_value();
  if (!pid || pid > 2147483647 || !ticks || !loaded || loaded > snapshot_mono_ns || !identifier(id) ||
      (variant != "small" && variant != "chestnut") || digest.size() != 64 ||
      !std::all_of(digest.begin(), digest.end(), [](char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); }) ||
      (!reason.is_null() && (!reason.is_string() || (fallback != "chestnut-load-failed" && fallback != "chestnut-run-stalled" &&
                                                    fallback != "selected-load-failed"))) ||
      (entry != entries.end() && id != "bundled-current" && entry->second.external != (variant == "chestnut"))) return fields;
  const auto proc = proc_root + "/" + std::to_string(pid) + "/stat";
  if (process_ticks(read_bounded(proc, 4096)) != ticks || read_bounded(path, 1024, true) != raw) return fields;
  fields["receiptStatus"] = "process-matched last-confirmed load receipt; current identity not established";
  fields["lastConfirmedLoadedId"] = id;
  fields["lastConfirmedLoadedName"] = entry == entries.end() ? "unresolved" : entry->second.name;
  fields["lastConfirmedLoadedVariant"] = variant;
  fields["lastConfirmedArtifactSha256"] = digest;
  fields["lastConfirmedFallbackReason"] = reason.is_null() ? "none" : fallback;
  fields["lastConfirmedLoadedMonoNs"] = std::to_string(loaded);
  fields["modeldPid"] = std::to_string(pid);
  fields["processStartTicks"] = std::to_string(ticks);
  return fields;
}

}  // namespace logger_model_metadata
