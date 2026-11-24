#include "tools/replay/local_timeline.h"

#include <fcntl.h>
#include <unistd.h>
#include <tuple>

#include "common/util.h"
#include "json11/json11.hpp"

namespace {
// shortcut: historical state follows Replay's 200 ms display tolerance; revisit if recorded service cadence changes.
constexpr uint64_t HOLD = 200000000ULL;
// shortcut: pathological histories disable colors; revisit the cap if real routes exceed it.
constexpr size_t MAX_ROWS = 65536, MAX_BYTES = 8 * 1024 * 1024, PAGE_ROWS = 256;
constexpr const char *KINDS[] = {"chill", "experimental", "aol", "error", "bookmark"};
}

void LocalTimeline::append(std::vector<Row> &rows, uint64_t begin, uint64_t end, int kind) {
  if (overflow_ || begin >= end) return;
  if (!rows.empty() && rows.back().kind == kind && rows.back().end == begin) {
    rows.back().end = end;
  } else if (modes_.size() + errors_.size() + bookmarks_.size() >= MAX_ROWS) {
    overflow_ = true;
  } else {
    rows.push_back({begin, end, kind});
  }
}

void LocalTimeline::advance(uint64_t time) {
  if (!cursor_) cursor_ = time;
  while (cursor_ < time && !overflow_) {
    auto next = time;
    for (const auto &s : state_) if (s.valid && s.stamp + HOLD > cursor_) next = std::min(next, s.stamp + HOLD);
    auto current = [&](int i) { return state_[i].valid && state_[i].stamp <= cursor_ && cursor_ - state_[i].stamp < HOLD; };
    if (current(0)) {
      if (state_[0].flags == 3 && current(1) && (state_[1].flags & 1)) {
        append(modes_, cursor_, next, (state_[1].flags & 2) ? 1 : 0);
      } else if (state_[0].flags == 1 && current(2) && state_[2].flags == 1) {
        append(modes_, cursor_, next, 2);
      }
    }
    if (current(1) && (state_[1].flags & 4)) append(errors_, cursor_, next, 3);
    cursor_ = next;
  }
}

void LocalTimeline::skip(int segment) {
  if (cursor_ && cursor_ <= UINT64_MAX - HOLD) advance(cursor_ + HOLD);
  state_ = {};
  cursor_ = 0;
  segment_ = segment;
  incomplete_ = true;
}

void LocalTimeline::consume(int segment, const std::vector<Event> &events, bool qlog) {
  if (segment != segment_ + 1) {
    if (segment_ >= 0) skip(segment - 1);
    state_ = {};
    cursor_ = 0;
  }
  segment_ = segment;
  qlog_ |= qlog;
  rlog_ |= !qlog;
  for (size_t i = 0; i < events.size() && !overflow_;) {
    const auto time = events[i].mono_time;
    size_t end = i + 1;
    while (end < events.size() && events[end].mono_time == time) ++end;
    if (time > 0 && time <= UINT64_MAX - HOLD) {
      std::array<std::optional<Observation>, 3> updates;
      bool relevant = false;
      for (size_t j = i; j < end; ++j) {
        const auto &e = events[j];
        int stream = e.which == cereal::Event::Which::CAR_CONTROL ? 0 :
                     e.which == cereal::Event::Which::SELFDRIVE_STATE ? 1 :
                     e.which == cereal::Event::Which::AOL_AXIS_STATE ? 2 : -1;
        if (stream < 0 && e.which != cereal::Event::Which::USER_BOOKMARK) continue;
        relevant = true;
        capnp::FlatArrayMessageReader reader(e.data);
        const auto event = reader.getRoot<cereal::Event>();
        if (stream < 0) {
          if (event.getValid()) {
            if (modes_.size() + errors_.size() + bookmarks_.size() >= MAX_ROWS) overflow_ = true;
            else if (bookmarks_.empty() || bookmarks_.back().begin != time) bookmarks_.push_back({time, time, 4});
          }
          continue;
        }
        Observation value{time, 0, event.getValid(), {}};
        if (stream == 0) {
          const auto cc = event.getCarControl();
          value.flags = unsigned(cc.getLatActive()) | unsigned(cc.getLongActive()) << 1;
        } else if (stream == 1) {
          const auto ss = event.getSelfdriveState();
          value.flags = unsigned(ss.getEnabled()) | unsigned(ss.getExperimentalMode()) << 1 |
            unsigned(ss.getAlertSize() != cereal::SelfdriveState::AlertSize::NONE &&
                     (ss.getAlertStatus() == cereal::SelfdriveState::AlertStatus::USER_PROMPT ||
                      ss.getAlertStatus() == cereal::SelfdriveState::AlertStatus::CRITICAL)) << 2;
        } else {
          const auto axis = event.getAolAxisState();
          value.flags = axis.getMode() == cereal::AolAxisState::Mode::LATERAL_ONLY && axis.getQualified() &&
                        axis.getNativeAcknowledged() && axis.getLateralActive() && !axis.getLongitudinalActive();
          if (axis.getSessionId().size() > 128) value.valid = false;
          else value.session = axis.getSessionId().cStr();
        }
        if (updates[stream] && *updates[stream] != value) value.valid = false;
        updates[stream] = std::move(value);
      }
      if (!relevant) { i = end; continue; }
      if (time < cursor_) {
        for (int s = 0; s < 3; ++s) if (updates[s]) state_[s].valid = false;
        incomplete_ = true;
      } else {
        advance(time);
        for (int s = 0; s < 3; ++s) if (updates[s]) {
          if (state_[s].stamp == time && state_[s] != *updates[s]) updates[s]->valid = false;
          state_[s] = std::move(*updates[s]);
        }
      }
    }
    i = end;
  }
}

bool LocalTimeline::write(const std::string &path, uint64_t origin, const std::vector<Replay::SeekRange> &ranges) {
  if (cursor_ && cursor_ <= UINT64_MAX - HOLD) advance(cursor_ + HOLD);
  if (overflow_) return false;
  struct Output { double begin, end; int kind; };
  std::vector<Output> rows;
  for (const auto *layer : {&modes_, &errors_, &bookmarks_}) for (const auto &row : *layer) {
    if (row.end < origin) continue;
    const double begin = row.begin >= origin ? (row.begin - origin) / 1e9 : 0;
    const double end = (row.end - origin) / 1e9;
    for (const auto &range : ranges) {
      if (row.kind == 4 ? begin >= range.begin && begin <= range.end : end > range.begin && begin < range.end) {
        if (rows.size() >= MAX_ROWS) return false;
        rows.push_back({std::max(begin, range.begin), std::min(end, range.end), row.kind});
        if (row.kind == 4) break;
      }
    }
  }
  std::sort(rows.begin(), rows.end(), [](const auto &a, const auto &b) {
    return std::tie(a.begin, a.end, a.kind) < std::tie(b.begin, b.end, b.kind);
  });
  const std::string temporary = path + ".tmp";
  int fd = open(temporary.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600);
  if (fd < 0) return false;
  bool ok = true;
  size_t bytes = 0;
  for (size_t offset = 0; ok && (offset < rows.size() || offset == 0); offset += PAGE_ROWS) {
    json11::Json::array page;
    for (size_t i = offset; i < std::min(rows.size(), offset + PAGE_ROWS); ++i) {
      page.emplace_back(json11::Json::array{rows[i].begin, rows[i].end, KINDS[rows[i].kind]});
    }
    const auto line = json11::Json(json11::Json::object{{"schemaVersion", 1}, {"rowOffset", double(offset)},
      {"totalRows", double(rows.size())}, {"source", qlog_ ? (rlog_ ? "mixed" : "qlog") : "rlog"}, {"rows", page}}).dump() + "\n";
    bytes += line.size();
    ok = line.size() <= 65536 && bytes <= MAX_BYTES && HANDLE_EINTR(::write(fd, line.data(), line.size())) == ssize_t(line.size());
  }
  if (close(fd)) ok = false;
  if (ok) ok = rename(temporary.c_str(), path.c_str()) == 0;
  if (!ok) unlink(temporary.c_str());
  return ok;
}
