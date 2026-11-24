#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "system/loggerd/logger.h"
#include "system/loggerd/model_metadata.h"

namespace fs = std::filesystem;
using logger_model_metadata::collect;
using logger_model_metadata::Fields;

static void require(bool value, const char *message) {
  if (!value) { std::cerr << message << '\n'; std::exit(1); }
}

static void put(const fs::path &path, const std::string &data) {
  fs::create_directories(path.parent_path());
  std::ofstream stream(path, std::ios::binary);
  stream << data;
  require(stream.good(), "fixture write");
}

static std::string receipt(const std::string &id = "sc23", const std::string &variant = "small",
                           const std::string &reason = "null", const std::string &stamp = "100") {
  return "{\"version\":1,\"pid\":123,\"processStartTicks\":321,\"loadedMonoNs\":" + stamp +
         ",\"modelId\":\"" + id + "\",\"variant\":\"" + variant + "\",\"artifactSha256\":\"" +
         std::string(64, 'a') + "\",\"fallbackReason\":" + reason + "}";
}

int main() {
  const char *base = std::getenv("TMPDIR");
  const std::string pattern = std::string(base ? base : "/tmp") + "/starpilot-model-metadata-XXXXXX";
  std::vector<char> writable(pattern.begin(), pattern.end());
  writable.push_back(0);
  const char *temporary = mkdtemp(writable.data());
  require(temporary != nullptr, "private fixture directory");
  const fs::path root(temporary), catalog = root / "bundled.json", models = root / "models";
  const fs::path load = root / "shm/starpilot_modeld_test/load-v1.json", proc = root / "proc/123/stat";
  const std::string head = "{\"generation\":\"v26\",\"compiler_revision\":\"9d0446a4ba8a532c8b674fb6ad795af015cd9dcf\","
                           "\"artifact_abi\":\"tinygrad_single_v1_arena\",\"models\":[";
  put(catalog, head + "{\"id\":\"rdf43\",\"name\":\"RDF V4\",\"version\":\"v15\"},"
                     "{\"id\":\"sc23\",\"name\":\"Secret Good\",\"version\":\"v8\"},"
                     "{\"id\":\"cinquev3\",\"name\":\"Cinque\",\"version\":\"v8\",\"uses_external_gpu\":true}]}");
  const auto sample = [&](uint64_t now = 1000) { return collect(catalog, models, root / "shm", "test", root / "proc", now); };
  auto fields = sample();
  require(fields["requestedSmallId"] == "rdf43" && fields["requestedSmallName"] == "RDF V4", "startup saved default");
  require(fields["lastConfirmedLoadedId"] == "unavailable" && fields["receiptStatus"] == "unavailable at snapshot", "startup never selected-as-loaded");
  put(models / "preferences.json", "{\"small\":\"sc23\",\"big\":\"cinquev3\",\"randomizer\":true}");
  std::string stat = "123 (modeld with ) parentheses) S";
  for (int i = 0; i < 18; ++i) stat += " 0";
  put(proc, stat + " 321 0 0\n");
  put(load, receipt());
  chmod(load.parent_path().c_str(), 0700);
  fields = sample();
  require(fields["requestedSmallName"] == "Secret Good" && fields["requestedChestnutId"] == "cinquev3", "requested profiles distinct");
  require(fields["lastConfirmedLoadedId"] == "sc23" && fields["lastConfirmedLoadedName"] == "Secret Good" && fields["lastConfirmedLoadedVariant"] == "small", "exact live load");
  require(fields["currentIdentityStatus"].find("not established") != std::string::npos, "even a retained receipt is not current-identity proof");
  require(fields["clockDomain"] == "CLOCK_MONOTONIC" && fields["snapshotMonoTime"] == "1000", "explicit snapshot clock");
  put(load, receipt("rdf43", "small", "\"selected-load-failed\""));
  fields = sample();
  require(fields["lastConfirmedLoadedId"] == "rdf43" && fields["requestedSmallId"] == "sc23" && fields["lastConfirmedFallbackReason"] == "selected-load-failed", "fallback separate from request");
  put(load, receipt("rdf43", "small", "\"chestnut-run-stalled\""));
  require(sample()["lastConfirmedFallbackReason"] == "chestnut-run-stalled", "recovery reason retained");
  put(load, receipt("cinquev3", "chestnut"));
  require(sample()["lastConfirmedLoadedVariant"] == "chestnut", "runtime switch actual receipt");
  put(models / "preferences.json", "{\"small\":\"rdf43\",\"big\":\"\"}");
  fields = sample();
  require(fields["lastConfirmedLoadedId"] == "cinquev3" && fields["requestedSmallId"] == "rdf43", "next-start selection does not rewrite actual load");
  put(models / "preferences.json", "{\"small\":\"unknown-saved-model\",\"big\":\"\"}");
  fields = sample();
  require(fields["requestedSmallId"] == "unknown-saved-model" && fields["requestedSmallName"] == "unresolved", "unknown saved intent is not a fabricated RDF default");
  require(fields["lastConfirmedLoadedId"] == "cinquev3", "unresolved selection does not rewrite receipt identity");
  put(models / "preferences.json", "{\"small\":\"sc23\",\"big\":\"\"}");
  put(load, receipt());
  put(proc, stat + " 322\n");
  require(sample()["lastConfirmedLoadedId"] == "unavailable", "PID reuse/start ticks mismatch refused");
  put(proc, stat + " 321\n");
  require(sample(99)["lastConfirmedLoadedId"] == "unavailable", "future load clock refused");
  require(sample(10000000000000000000ULL)["lastConfirmedLoadedId"] == "sc23", "long lived load has no age TTL");
  put(load, receipt("sc23", "small", "null", "9007199254740993"));
  require(sample(9007199254740993ULL)["lastConfirmedLoadedMonoNs"] == "9007199254740993", "uint64 timestamp exact above double precision");
  put(load, receipt("sc23", "small", "null", "18446744073709551616"));
  require(sample()["lastConfirmedLoadedId"] == "unavailable", "uint64 overflow refused");
  put(load, receipt() + std::string(1025, ' '));
  require(sample()["lastConfirmedLoadedId"] == "unavailable", "oversize receipt refused");
  fs::remove(load);
  mkfifo(load.c_str(), 0600);
  require(sample()["lastConfirmedLoadedId"] == "unavailable", "FIFO does not block");
  fs::remove(load);
  put(root / "outside.json", receipt());
  fs::create_symlink(root / "outside.json", load);
  require(sample()["lastConfirmedLoadedId"] == "unavailable", "receipt symlink refused");
  fs::remove(load);
  put(load, receipt());
  auto duplicate = receipt();
  duplicate.insert(1, "\"pid\":456,");
  put(load, duplicate);
  require(sample()["lastConfirmedLoadedId"] == "unavailable", "duplicate receipt field refused");
  put(load, receipt());
  fs::remove(catalog);
  require(sample()["lastConfirmedLoadedName"] == "unresolved", "catalog failure never substitutes selected name");
  require(sample()["requestedSmallId"] == "sc23" && sample()["requestedSmallName"] == "unresolved", "catalog failure preserves raw requested ID");

  // Real Cap'n Proto serialization and zstd qlog opening bytes. The production
  // logger calls this same snapshot-copy function before writing each segment.
  MessageBuilder original;
  auto event = original.initEvent();
  event.setLogMonoTime(1234);
  auto init = event.initInitData();
  init.setWallTimeNanos(9876);
  init.setKernelVersion("Linux version 4.9.103 traceable-original");
  auto entries = init.initCommands().initEntries(2);
  entries[0].setKey("original");
  entries[0].setValue(capnp::Data::Reader(reinterpret_cast<const kj::byte *>("kept"), 4));
  entries[1].setKey("starpilot.model.lastConfirmedLoadedId");
  entries[1].setValue(capnp::Data::Reader(reinterpret_cast<const kj::byte *>("old"), 3));
  const auto words = capnp::messageToFlatArray(original);
  auto updated = logger_model_init_data(words.asPtr(), sample());
  for (const auto *name : {"qlog.zst", "rlog.zst"}) {
    ZstdFileWriter writer((root / name).string(), LOG_COMPRESSION_LEVEL);
    writer.write(updated.asBytes());
  }
  const auto qlog = zstd_decompress(util::read_file((root / "qlog.zst").string()));
  require(qlog == zstd_decompress(util::read_file((root / "rlog.zst").string())), "both logs retain identical opening metadata");
  AlignedBuffer aligned;
  capnp::FlatArrayMessageReader reader(aligned.align(qlog.data(), qlog.size()));
  const auto recorded = reader.getRoot<cereal::Event>();
  require(recorded.getLogMonoTime() == 1234 && recorded.getInitData().getWallTimeNanos() == 9876, "original route clocks preserved");
  require(recorded.getInitData().getKernelVersion() == "Linux version 4.9.103 traceable-original", "kernel log remains truthful");
  Fields values;
  for (auto entry : recorded.getInitData().getCommands().getEntries()) {
    const auto data = entry.getValue();
    values[entry.getKey().cStr()] = std::string(reinterpret_cast<const char *>(data.begin()), data.size());
  }
  require(values["original"] == "kept" && values["starpilot.model.lastConfirmedLoadedId"] == "sc23", "opening metadata refreshed without old identity");
  require(values["starpilot.model.clockDomain"] == "CLOCK_MONOTONIC", "clock domain serialized");
  fs::remove_all(root);
  return 0;
}
