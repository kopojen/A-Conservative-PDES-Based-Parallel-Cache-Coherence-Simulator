/**
 * @file msi_cache.cpp
 * @brief Private cache + snoopy MSI controller implementation.
 */

#include "msi_cache.h"

#include <stdexcept>

namespace {

constexpr bool IsPowerOfTwo(uint64_t value) {
  return value != 0 && (value & (value - 1)) == 0;
}

}  // namespace

MSICache::MSICache(uint32_t core_id, size_t cache_size_bytes,
                   size_t line_size_bytes, size_t associativity)
    : associativity_(associativity), core_id_(core_id) {
  if (line_size_bytes == 0 || !IsPowerOfTwo(line_size_bytes)) {
    throw std::invalid_argument("line_size must be a power-of-two > 0");
  }
  if (cache_size_bytes == 0 || cache_size_bytes < line_size_bytes ||
      (cache_size_bytes % line_size_bytes) != 0) {
    throw std::invalid_argument("cache_size must be >= line_size and aligned");
  }
  if (associativity_ == 0) {
    throw std::invalid_argument("associativity must be > 0");
  }

  const size_t num_lines = cache_size_bytes / line_size_bytes;
  if ((num_lines % associativity_) != 0) {
    throw std::invalid_argument(
        "cache_size/line_size must be divisible by associativity");
  }

  num_sets_ = static_cast<uint32_t>(num_lines / associativity_);
  if (num_sets_ == 0) {
    throw std::invalid_argument("number of sets must be >= 1");
  }

  line_size_shift_ = 0;
  uint64_t size = line_size_bytes;
  while ((1ull << line_size_shift_) < size) {
    ++line_size_shift_;
  }

  lru_per_set_.resize(num_sets_);
}

MSICache::State MSICache::GetState(uint64_t line_addr) const {
  auto it = lines_.find(line_addr);
  if (it == lines_.end()) {
    return State::Invalid;
  }
  return it->second.state;
}

void MSICache::Touch(uint64_t line_addr) {
  auto it = lines_.find(line_addr);
  if (it == lines_.end()) {
    return;
  }
  auto& lru_list = lru_per_set_[it->second.set_idx];
  lru_list.splice(lru_list.begin(), lru_list, it->second.lru_pos);
}

void MSICache::InsertOrUpdate(uint64_t line_addr, State state) {
  auto it = lines_.find(line_addr);
  if (it != lines_.end()) {
    it->second.state = state;
    Touch(line_addr);
    return;
  }

  uint32_t set_idx = ComputeSetIndex(line_addr);
  EvictIfNecessary(set_idx);
  auto& lru_list = lru_per_set_[set_idx];
  lru_list.push_front(line_addr);

  LineInfo li;
  li.state = state;
  li.set_idx = set_idx;
  li.lru_pos = lru_list.begin();
  lines_.emplace(line_addr, std::move(li));
}

void MSICache::SetState(uint64_t line_addr, State state) {
  auto it = lines_.find(line_addr);
  if (it == lines_.end()) {
    InsertOrUpdate(line_addr, state);
  } else {
    it->second.state = state;
  }
}

void MSICache::Invalidate(uint64_t line_addr) {
  auto it = lines_.find(line_addr);
  if (it == lines_.end()) {
    return;
  }
  auto& lru_list = lru_per_set_[it->second.set_idx];
  lru_list.erase(it->second.lru_pos);
  lines_.erase(it);
}

uint32_t MSICache::ComputeSetIndex(uint64_t line_addr) const {
  uint64_t idx = (line_addr >> line_size_shift_) % num_sets_;
  return static_cast<uint32_t>(idx);
}

void MSICache::EvictIfNecessary(uint32_t set_idx) {
  auto& lru_list = lru_per_set_[set_idx];
  if (lru_list.size() < associativity_) {
    return;
  }
  uint64_t victim = lru_list.back();
  lru_list.pop_back();
  lines_.erase(victim);
}

void MSICache::HandleExternalReadMiss(uint64_t line_addr) {
  auto it = lines_.find(line_addr);
  if (it == lines_.end()) {
    return;
  }
  if (it->second.state == State::Modified) {
    it->second.state = State::Shared;
  }
}

void MSICache::HandleExternalWriteMiss(uint64_t line_addr) {
  Invalidate(line_addr);
}
