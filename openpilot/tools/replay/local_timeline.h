#pragma once

#include <array>
#include <string>
#include <vector>

#include "tools/replay/replay.h"

// Private to machine-mode Replay; the developer-tool Timeline has separate semantics.
class LocalTimeline {
public:
  void consume(int segment, const std::vector<Event> &events, bool qlog);
  void skip(int segment);
  bool write(const std::string &path, uint64_t origin, const std::vector<Replay::SeekRange> &ranges);
  bool incomplete() const { return incomplete_; }

private:
  struct Observation {
    uint64_t stamp = 0;
    unsigned flags = 0;
    bool valid = false;
    std::string session;
    bool operator!=(const Observation &other) const {
      return stamp != other.stamp || flags != other.flags || valid != other.valid || session != other.session;
    }
  };
  struct Row { uint64_t begin, end; int kind; };
  void advance(uint64_t time);
  void append(std::vector<Row> &rows, uint64_t begin, uint64_t end, int kind);
  std::array<Observation, 3> state_{};
  std::vector<Row> modes_, errors_, bookmarks_;
  uint64_t cursor_ = 0;
  int segment_ = -2;
  bool qlog_ = false, rlog_ = false, overflow_ = false, incomplete_ = false;
};
