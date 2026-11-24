#include "tools/replay/filereader.h"

#include <fstream>
#include <memory>
#include <new>
#include <optional>
#include <zstd.h>
#include <unistd.h>
#include <filesystem>

#include "common/util.h"
#include "tools/replay/py_tools.h"

std::string FileReader::read(const std::string &file, std::atomic<bool> *abort) {
  if (abort && *abort) return {};
  auto read_bounded = [&](const std::string &path) {
    std::ifstream source(path, std::ios::binary);
    std::string data;
    if (source && max_bytes_) {
      std::error_code error;
      const auto size = std::filesystem::file_size(path, error);
      if (!error && size <= max_bytes_) data.reserve(size);
    }
    char chunk[65536];
    while (source && !(abort && *abort)) {
      source.read(chunk, sizeof(chunk));
      const auto count = source.gcount();
      if (max_bytes_ && data.size() + count > max_bytes_) return std::string{};
      data.append(chunk, count);
    }
    if (source.bad() || (abort && *abort)) return std::string{};
    return data;
  };
  const bool is_remote = (file.find("https://") == 0) || (file.find("http://") == 0);
  if (is_remote) {
    std::string local_path = PyTools::download(file, cache_to_local_, abort);
    if (local_path.empty()) return {};
    return read_bounded(local_path);
  }
  char header[4] = {};
  std::ifstream stream(file, std::ios::binary);
  stream.read(header, sizeof(header));
  const std::string magic(header, stream.gcount());
  if (magic == "\x28\xB5\x2F\xFD") {
    std::unique_ptr<ZSTD_DStream, decltype(&ZSTD_freeDStream)> decoder(ZSTD_createDStream(), ZSTD_freeDStream);
    if (!decoder) return {};
    std::string data;
    auto decode = [&](bool retain) -> std::optional<size_t> {
      stream.clear();
      stream.seekg(0);
      if (ZSTD_isError(ZSTD_initDStream(decoder.get()))) return std::nullopt;
      char input_bytes[65536], output_bytes[65536];
      size_t remaining = 1, total = 0;
      while (stream && !(abort && *abort)) {
        stream.read(input_bytes, sizeof(input_bytes));
        ZSTD_inBuffer input{input_bytes, size_t(stream.gcount()), 0};
        bool drain = false;
        while (input.pos < input.size || drain) {
          if (abort && *abort) return std::nullopt;
          ZSTD_outBuffer output{output_bytes, sizeof(output_bytes), 0};
          remaining = ZSTD_decompressStream(decoder.get(), &output, &input);
          if (ZSTD_isError(remaining) || (max_bytes_ && output.pos > max_bytes_ - total)) return std::nullopt;
          total += output.pos;
          if (retain) data.append(output_bytes, output.pos);
          drain = output.pos == sizeof(output_bytes) && remaining != 0;
        }
      }
      if (!stream.eof() || remaining || (abort && *abort)) return std::nullopt;
      return total;
    };
    try {
      if (max_bytes_) {
        try {
          data.reserve(max_bytes_);
        } catch (const std::bad_alloc &) {
          // Count only when reserving the full cap cannot fit; normal reads stay single-pass.
          const auto size = decode(false);
          if (!size) return {};
          data.reserve(*size);
        }
      }
      if (!decode(true)) return {};
      return data;
    } catch (const std::bad_alloc &) {
      return {};
    }
  }
  if (util::ends_with(file, ".bz2") || util::ends_with(file, ".zst") ||
      util::starts_with(magic, "BZh") || magic == "\x28\xB5\x2F\xFD") {
    std::string local_path = PyTools::decompress(file, abort, max_bytes_);
    if (local_path.empty()) return {};
    std::string data = read_bounded(local_path);
    unlink(local_path.c_str());
    return data;
  }
  return read_bounded(file);
}
