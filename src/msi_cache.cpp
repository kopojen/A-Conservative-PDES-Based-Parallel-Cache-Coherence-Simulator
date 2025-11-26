/**
 * @file msi_cache.cpp
 * @brief MSI cache coherence protocol simulator.
 */

#include "msi_cache.h"
#include <stdexcept>

MSICache::MSICache(uint32_t num_cores) : num_cores_(num_cores) {
  if (num_cores_ == 0) {
    throw std::invalid_argument("num_cores must be > 0");
  }
  stats_.resize(num_cores_);
}

void MSICache::Process(const TraceEvent& ev) {
  if (ev.core_id >= num_cores_) {
    throw std::runtime_error("core_id out of range: " +
                             std::to_string(ev.core_id));
  }

  LineInfo& line = lines_[ev.address];
  if (ev.is_write) {
    stats_[ev.core_id].writes++;
    ProcessWrite(ev, line);
  } else {
    stats_[ev.core_id].reads++;
    ProcessRead(ev, line);
  }
}

void MSICache::ProcessRead(const TraceEvent& ev, LineInfo& line) {
  auto& s = stats_[ev.core_id];
  int core = static_cast<int>(ev.core_id);

  switch (line.state) {
    case State::Invalid:
      // I → S: fetch from memory
      s.misses++;
      line.state = State::Shared;
      line.sharers = {core};
      line.owner = -1;
      break;

    case State::Shared:
      // S → S: hit if already sharer, miss otherwise
      if (line.sharers.find(core) != line.sharers.end()) {
        s.hits++;
      } else {
        s.misses++;
        line.sharers.insert(core);
      }
      break;

    case State::Modified:
      // M → ? depending on owner
      if (line.owner == core) {
        s.hits++;  // M → M (owner reads own data)
      } else {
        // M → S: non-owner reads modified data (requires writeback)
        s.misses++;
        line.state = State::Shared;
        line.sharers.clear();
        line.sharers.insert(line.owner);
        line.sharers.insert(core);
        line.owner = -1;
      }
      break;
  }
}

void MSICache::ProcessWrite(const TraceEvent& ev, LineInfo& line) {
  auto& s = stats_[ev.core_id];
  int core = static_cast<int>(ev.core_id);

  switch (line.state) {
    case State::Invalid:
      // I → M: acquire exclusive ownership
      s.misses++;
      line.state = State::Modified;
      line.owner = core;
      line.sharers.clear();
      break;

    case State::Shared:
      // S → M: upgrade (invalidate other sharers)
      s.misses++;
      line.state = State::Modified;
      line.owner = core;
      line.sharers.clear();
      break;

    case State::Modified:
      // M → M depending on owner
      if (line.owner == core) {
        s.hits++;  // M → M (owner writes own data)
      } else {
        // M → M: non-owner writes (ownership transfer)
        s.misses++;
        line.owner = core;
      }
      break;
  }
}
