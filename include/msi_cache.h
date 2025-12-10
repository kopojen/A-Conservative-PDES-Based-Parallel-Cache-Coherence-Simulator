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
#include <list>
#include <unordered_map>
#include <vector>

#include "trace_event.h"

struct CoreStats {
  uint64_t reads{0};
  uint64_t writes{0};
  uint64_t hits{0};
  uint64_t misses{0};
};

/**
 * @brief Private MSI cache for a single core with fixed line size/capacity.
 *
 * This class only tracks the local view (lines + states) plus LRU info. Global
 * coherence is coordinated externally through message-based snooping.
 */
class MSICache {
 public:
  enum class State { Invalid, Shared, Modified };

  MSICache(uint32_t core_id, size_t cache_size_bytes, size_t line_size_bytes,
           size_t associativity);

  uint32_t core_id() const { return core_id_; }

  State GetState(uint64_t line_addr) const;
  void Touch(uint64_t line_addr);
  void InsertOrUpdate(uint64_t line_addr, State state);
  void SetState(uint64_t line_addr, State state);
  void Invalidate(uint64_t line_addr);
  void HandleExternalReadMiss(uint64_t line_addr);
  void HandleExternalWriteMiss(uint64_t line_addr);

 private:
  struct LineInfo {
    State state{State::Invalid};
    uint32_t set_idx{0};
    std::list<uint64_t>::iterator lru_pos;
  };

  uint32_t ComputeSetIndex(uint64_t line_addr) const;
  void EvictIfNecessary(uint32_t set_idx);

  size_t associativity_{0};
  uint32_t num_sets_{0};
  uint32_t line_size_shift_{0};
  uint32_t core_id_{0};
  std::unordered_map<uint64_t, LineInfo> lines_;
  std::vector<std::list<uint64_t>> lru_per_set_;
};
