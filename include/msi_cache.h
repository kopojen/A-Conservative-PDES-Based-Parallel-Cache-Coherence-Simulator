/**
 * @file msi_cache.h
 * @brief MSI (Modified-Shared-Invalid) cache coherence protocol simulator.
 *
 * States:
 *   Invalid (I):  Cache line invalid
 *   Shared (S):   Multiple cores can read (read-only)
 *   Modified (M): Single core owns (read-write)
 */

#pragma once

#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "trace_event.h"

struct CoreStats {
  uint64_t reads{0};
  uint64_t writes{0};
  uint64_t hits{0};
  uint64_t misses{0};
};

class MSICache {
 public:
  explicit MSICache(uint32_t num_cores);

  void Process(const TraceEvent& ev);
  const std::vector<CoreStats>& stats() const { return stats_; }

 private:
  enum class State { Invalid, Shared, Modified };

  struct LineInfo {
    State state{State::Invalid};
    int owner{-1};
    std::unordered_set<int> sharers;
  };

  uint32_t num_cores_{0};
  std::unordered_map<uint64_t, LineInfo> lines_;
  std::vector<CoreStats> stats_;

  void ProcessRead(const TraceEvent& ev, LineInfo& line);
  void ProcessWrite(const TraceEvent& ev, LineInfo& line);
};
