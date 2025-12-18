/**
 * @file pdes_channel.cpp
 * @brief Lock-free broadcast log implementation for PDES coherence events.
 */

#include "pdes_channel.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <thread>

// ============================================================================
// Ring buffer (multi-writer, multiple readers)
// ============================================================================

void PdesRing::Append(uint64_t ts, PdesMsgKind kind, uint32_t src,
                      uint64_t line_addr, uint64_t src_msg_id) {
  const uint64_t seq =
      next_seq_.fetch_add(1, std::memory_order_acq_rel);
  auto &slot = buffer_[seq % capacity_];

  slot.kind = kind;
  slot.src = src;
  slot.ts = ts;
  slot.line_addr = line_addr;
  slot.src_msg_id = src_msg_id;
  slot.seq.store(seq + 1, std::memory_order_release);
}

bool PdesRing::Read(uint64_t seq, PdesMsg &msg) const {
  const uint64_t published = next_seq_.load(std::memory_order_acquire);
  if (seq >= published) {
    return false;
  }

  const auto &slot = buffer_[seq % capacity_];
  const uint64_t slot_seq = slot.seq.load(std::memory_order_acquire);
  const uint64_t desired = seq + 1;

  if (slot_seq < desired) {
    return false; // Not yet published for this seq
  }
  if (slot_seq > desired) {
    throw std::runtime_error("Ring buffer overrun for PDES channel");
  }

  msg.kind = slot.kind;
  msg.src = slot.src;
  msg.ts = slot.ts;
  msg.line_addr = slot.line_addr;
  msg.src_msg_id = slot.src_msg_id;
  return true;
}

uint64_t PdesRing::PublishedSeq() const {
  return next_seq_.load(std::memory_order_acquire);
}

// ============================================================================
// PdesInbox Implementation
// ============================================================================

PdesInbox::PdesInbox(uint32_t self_core, const PdesRing &ring,
                     const std::vector<ChannelLowerBound> &channel_lb)
    : ring_(&ring), channel_lb_(&channel_lb),
      per_src_queues_(channel_lb.size()), num_cores_(channel_lb.size()),
      self_core_(self_core) {}

void PdesInbox::Drain() {
  if (ring_ == nullptr) {
    return;
  }

  while (true) {
    uint64_t published = ring_->PublishedSeq();
    if (drain_seq_ >= published) {
      break;
    }

    PdesMsg msg;
    while (!ring_->Read(drain_seq_, msg)) {
      std::this_thread::yield();
    }
    ++drain_seq_;

    if (msg.src == self_core_) {
      continue;
    }
    per_src_queues_[msg.src].push_back(msg);
  }
}

uint64_t PdesInbox::NextTimestampFrom(uint32_t src) const {
  if (src == self_core_) {
    return kTimestampInfinity;
  }
  const auto &queue = per_src_queues_[src];
  if (queue.empty()) {
    return kTimestampInfinity;
  }
  return queue.front().ts;
}

uint64_t PdesInbox::PeekMinTimestamp() const {
  uint64_t min_ts = kTimestampInfinity;

  for (uint32_t src = 0; src < num_cores_; ++src) {
    if (src == self_core_) {
      continue;
    }
    uint64_t ts = NextTimestampFrom(src);
    if (ts < min_ts) {
      min_ts = ts;
    }
  }

  return min_ts;
}

uint64_t PdesInbox::GetChannelLowerBound(uint32_t src) const {
  return channel_lb_->at(src).value.load(std::memory_order_acquire);
}

PdesInbox::ScanResult PdesInbox::Scan() const {
  ScanResult res;

  for (uint32_t src = 0; src < num_cores_; ++src) {
    if (src == self_core_) {
      continue;
    }

    const auto &queue = per_src_queues_[src];
    uint64_t qmin = queue.empty() ? kTimestampInfinity : queue.front().ts;
    uint64_t lb = channel_lb_->at(src).value.load(std::memory_order_acquire);
    uint64_t cmin = std::min(lb, qmin);

    if (cmin < res.safe_time) {
      res.safe_time = cmin;
    }

    if (!queue.empty()) {
      const auto &front = queue.front();
      bool update = false;
      if (front.ts < res.min_ts) {
        update = true;
      } else if (front.ts == res.min_ts) {
        if (src < res.min_src) {
          update = true;
        } else if (src == res.min_src &&
                   front.src_msg_id < res.min_src_msg_id) {
          update = true;
        }
      }

      if (update) {
        res.min_ts = front.ts;
        res.min_src = src;
        res.min_src_msg_id = front.src_msg_id;
      }
    }
  }

  return res;
}

uint64_t PdesInbox::ComputeSafeTime(uint32_t self_core) const {
  (void)self_core;
  uint64_t safe_time = kTimestampInfinity;

  for (uint32_t src = 0; src < num_cores_; ++src) {
    if (src == self_core_) {
      continue;
    }

    uint64_t lb = channel_lb_->at(src).value.load(std::memory_order_acquire);
    uint64_t qmin = NextTimestampFrom(src);
    uint64_t cmin = std::min(lb, qmin);

    if (cmin < safe_time) {
      safe_time = cmin;
    }
  }

  return safe_time;
}

uint64_t PdesInbox::GetMinRealMsgTimestamp(uint32_t src) const {
  if (src == self_core_) {
    return kTimestampInfinity;
  }
  return NextTimestampFrom(src);
}

bool PdesInbox::TryPop(PdesMsg &msg, uint64_t safe_time) {
  uint64_t min_ts = kTimestampInfinity;
  uint32_t min_src = num_cores_;
  uint64_t min_msg_id = 0;

  for (uint32_t src = 0; src < num_cores_; ++src) {
    if (src == self_core_) {
      continue;
    }
    const auto &queue = per_src_queues_[src];
    if (queue.empty()) {
      continue;
    }
    const auto &front = queue.front();
    if (front.ts < min_ts ||
        (front.ts == min_ts &&
         (src < min_src ||
          (src == min_src && front.src_msg_id < min_msg_id)))) {
      min_ts = front.ts;
      min_src = src;
      min_msg_id = front.src_msg_id;
    }
  }

  if (min_src == num_cores_ || min_ts > safe_time) {
    return false;
  }

  auto &queue = per_src_queues_[min_src];
  msg = queue.front();
  queue.pop_front();
  msg.dst = self_core_;
  return true;
}

bool PdesInbox::TryPopFrom(uint32_t src, uint64_t safe_time, PdesMsg &msg) {
  if (src >= num_cores_ || src == self_core_) {
    return false;
  }

  auto &queue = per_src_queues_[src];
  if (queue.empty()) {
    return false;
  }
  const auto &front = queue.front();
  if (front.ts > safe_time) {
    return false;
  }
  msg = front;
  queue.pop_front();
  msg.dst = self_core_;
  return true;
}

void PdesInbox::WaitForMessage() {
  if (wait_backoff_ < 8) {
    std::this_thread::yield();
  } else {
    uint32_t exp = wait_backoff_ - 8;
    uint32_t us = 1u << std::min<uint32_t>(exp, 6);
    std::this_thread::sleep_for(std::chrono::microseconds(us));
  }
  if (wait_backoff_ < 32) {
    ++wait_backoff_;
  }
}

void PdesInbox::ResetWaitBackoff() { wait_backoff_ = 0; }

thread_local uint32_t PdesInbox::wait_backoff_{0};

// ============================================================================
// PdesChannelManager Implementation
// ============================================================================

PdesChannelManager::PdesChannelManager(uint32_t num_cores)
    : num_cores_(num_cores) {
  if (num_cores == 0) {
    throw std::invalid_argument("num_cores must be > 0");
  }

  if (static_cast<size_t>(num_cores) >
      std::numeric_limits<size_t>::max() / kRingCapacityPerSource) {
    throw std::overflow_error("Too many cores for PDES ring capacity");
  }
  size_t ring_capacity =
      kRingCapacityPerSource * static_cast<size_t>(num_cores);
  ring_ = std::make_unique<PdesRing>(ring_capacity);

  channel_lb_.reserve(num_cores);
  inboxes_.reserve(num_cores);

  for (uint32_t i = 0; i < num_cores; ++i) {
    channel_lb_.push_back(ChannelLowerBound{});
  }

  for (uint32_t i = 0; i < num_cores; ++i) {
    inboxes_.push_back(
        std::make_unique<PdesInbox>(i, *ring_, channel_lb_));
  }
}

PdesInbox &PdesChannelManager::GetInbox(uint32_t core_id) {
  return *inboxes_[core_id];
}

void PdesChannelManager::AppendReal(uint32_t src_core, uint64_t arrival_time,
                                    PdesMsgKind kind, uint64_t line_addr,
                                    uint64_t src_msg_id) {
  ring_->Append(arrival_time, kind, src_core, line_addr, src_msg_id);
}

void PdesChannelManager::UpdateLowerBound(uint32_t src_core, uint64_t lb) {
  // Single-writer per entry (each core only updates its own lb), so a simple
  // store is sufficient and avoids CAS traffic.
  channel_lb_[src_core].value.store(lb, std::memory_order_release);
}
