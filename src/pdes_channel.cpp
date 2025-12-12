/**
 * @file pdes_channel.cpp
 * @brief Implementation of Conservative PDES channel with CMB protocol.
 */

#include "pdes_channel.h"

#include <algorithm>
#include <stdexcept>

// ============================================================================
// PdesInbox Implementation
// ============================================================================

PdesInbox::PdesInbox(uint32_t num_cores)
    : channel_lb_(num_cores), per_source_msgs_(num_cores),
      num_cores_(num_cores) {
  // Initialize all channel lower bounds to 0 (unknown)
  for (uint32_t i = 0; i < num_cores; ++i) {
    channel_lb_[i].store(0, std::memory_order_relaxed);
  }
}

void PdesInbox::Push(const PdesMsg &msg) {
  if (msg.kind == PdesMsgKind::Null) {
    // Null message: just update the lower bound
    UpdateChannelLowerBound(msg.src, msg.ts);
    // Wake up waiting threads
    cv_.notify_all();
    return;
  }

  // Real message: add to heap and per-source map
  {
    std::lock_guard<std::mutex> lock(mutex_);
    heap_.push(msg);

    // Add to per-source multimap for efficient min tracking
    per_source_msgs_[msg.src].emplace(msg.ts, msg);
  }
  cv_.notify_all();
}

bool PdesInbox::TryPop(PdesMsg &msg, uint64_t safe_time) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (heap_.empty()) {
    return false;
  }

  const PdesMsg &top = heap_.top();
  if (top.ts > safe_time) {
    return false;
  }

  msg = top;
  heap_.pop();

  // Remove from per-source map - O(log n) operation
  auto &src_map = per_source_msgs_[msg.src];
  auto range = src_map.equal_range(msg.ts);
  for (auto it = range.first; it != range.second; ++it) {
    // Find the exact message by comparing all fields
    if (it->second.kind == msg.kind && it->second.src == msg.src &&
        it->second.dst == msg.dst && it->second.line_addr == msg.line_addr) {
      src_map.erase(it);
      break;
    }
  }

  return true;
}

uint64_t PdesInbox::PeekMinTimestamp() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (heap_.empty()) {
    return kTimestampInfinity;
  }
  return heap_.top().ts;
}

uint64_t PdesInbox::GetChannelLowerBound(uint32_t src) const {
  return channel_lb_[src].load(std::memory_order_acquire);
}

void PdesInbox::UpdateChannelLowerBound(uint32_t src, uint64_t lb) {
  uint64_t current = channel_lb_[src].load(std::memory_order_relaxed);
  while (lb > current) {
    if (channel_lb_[src].compare_exchange_weak(current, lb,
                                               std::memory_order_release,
                                               std::memory_order_relaxed)) {
      break;
    }
  }
}

uint64_t PdesInbox::ComputeSafeTime(uint32_t self_core) const {
  uint64_t safe_time = kTimestampInfinity;

  std::lock_guard<std::mutex> lock(mutex_);

  for (uint32_t src = 0; src < num_cores_; ++src) {
    if (src == self_core) {
      continue; // Skip self
    }

    // cmin[src] = min(qmin[src], lb[src])
    uint64_t lb = channel_lb_[src].load(std::memory_order_acquire);

    // Get minimum timestamp from per-source map - O(1) operation
    uint64_t qmin = kTimestampInfinity;
    const auto &src_map = per_source_msgs_[src];
    if (!src_map.empty()) {
      qmin = src_map.begin()->first; // First element has minimum timestamp
    }

    uint64_t cmin = std::min(lb, qmin);

    if (cmin < safe_time) {
      safe_time = cmin;
    }
  }

  return safe_time;
}

bool PdesInbox::Empty() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return heap_.empty();
}

uint64_t PdesInbox::GetMinRealMsgTimestamp(uint32_t src) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto &src_map = per_source_msgs_[src];
  if (src_map.empty()) {
    return kTimestampInfinity;
  }
  return src_map.begin()->first; // O(1) - first element has minimum timestamp
}

void PdesInbox::WaitForMessage() {
  std::unique_lock<std::mutex> lock(mutex_);
  cv_.wait_for(lock, std::chrono::milliseconds(1));
}

void PdesInbox::NotifyAll() { cv_.notify_all(); }

// ============================================================================
// PdesChannelManager Implementation
// ============================================================================

PdesChannelManager::PdesChannelManager(uint32_t num_cores)
    : num_cores_(num_cores) {
  if (num_cores == 0) {
    throw std::invalid_argument("num_cores must be > 0");
  }

  inboxes_.reserve(num_cores);
  for (uint32_t i = 0; i < num_cores; ++i) {
    inboxes_.push_back(std::make_unique<PdesInbox>(num_cores));
  }
}

PdesInbox &PdesChannelManager::GetInbox(uint32_t core_id) {
  return *inboxes_[core_id];
}

void PdesChannelManager::Send(const PdesMsg &msg) {
  inboxes_[msg.dst]->Push(msg);
}

void PdesChannelManager::BroadcastReal(uint32_t src_core, uint64_t event_time,
                                       PdesMsgKind kind, uint64_t line_addr) {
  // Real message arrives at time = event_time + bus_latency
  uint64_t arrival_time = event_time + kBusLatency;

  for (uint32_t dst = 0; dst < num_cores_; ++dst) {
    if (dst == src_core) {
      continue;
    }

    PdesMsg msg;
    msg.kind = kind;
    msg.src = src_core;
    msg.dst = dst;
    msg.ts = arrival_time;
    msg.line_addr = line_addr;

    Send(msg);
  }
}

void PdesChannelManager::BroadcastNull(uint32_t src_core, uint64_t null_ts) {
  for (uint32_t dst = 0; dst < num_cores_; ++dst) {
    if (dst == src_core) {
      continue;
    }

    PdesMsg msg;
    msg.kind = PdesMsgKind::Null;
    msg.src = src_core;
    msg.dst = dst;
    msg.ts = null_ts;
    msg.line_addr = 0;

    Send(msg);
  }
}
