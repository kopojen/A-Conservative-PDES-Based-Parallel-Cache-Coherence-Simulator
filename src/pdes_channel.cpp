/**
 * @file pdes_channel.cpp
 * @brief Lock-free broadcast log implementation for PDES coherence events.
 */

#include "pdes_channel.h"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <thread>

// ============================================================================
// Ring buffer (single writer, multiple readers)
// ============================================================================

void PdesRing::Append(uint64_t ts, PdesMsgKind kind, uint32_t src,
                      uint64_t line_addr) {
  const uint64_t seq = next_seq_.load(std::memory_order_relaxed);
  auto &slot = buffer_[seq % capacity_];

  slot.kind = kind;
  slot.src = src;
  slot.ts = ts;
  slot.line_addr = line_addr;
  slot.seq.store(seq, std::memory_order_release);

  next_seq_.store(seq + 1, std::memory_order_release);
}

bool PdesRing::Read(uint64_t seq, PdesMsg &msg) const {
  const uint64_t published = next_seq_.load(std::memory_order_acquire);
  if (seq >= published) {
    return false;
  }

  const auto &slot = buffer_[seq % capacity_];
  const uint64_t slot_seq = slot.seq.load(std::memory_order_acquire);

  if (slot_seq < seq) {
    return false; // Not yet published for this seq
  }
  if (slot_seq != seq) {
    throw std::runtime_error("Ring buffer overrun for PDES channel");
  }

  msg.kind = slot.kind;
  msg.src = slot.src;
  msg.ts = slot.ts;
  msg.line_addr = slot.line_addr;
  return true;
}

uint64_t PdesRing::PublishedSeq() const {
  return next_seq_.load(std::memory_order_acquire);
}

// ============================================================================
// PdesInbox Implementation
// ============================================================================

PdesInbox::PdesInbox(uint32_t self_core,
                     const std::vector<std::unique_ptr<PdesRing>> &rings,
                     const std::vector<ChannelLowerBound> &channel_lb)
    : rings_(&rings), channel_lb_(&channel_lb), num_cores_(rings.size()),
      self_core_(self_core) {
  read_seq_.assign(num_cores_, 0);
}

uint64_t PdesInbox::NextTimestampFrom(uint32_t src) const {
  // Fast path: if the source hasn't published anything beyond our read pointer,
  // we know there is no readable message without touching the ring slots.
  const auto &ring = *rings_->at(src);
  const uint64_t published = ring.PublishedSeq();
  if (read_seq_[src] >= published) {
    return kTimestampInfinity;
  }

  PdesMsg tmp;
  if (ring.Read(read_seq_[src], tmp)) {
    return tmp.ts;
  }
  return kTimestampInfinity;
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

PdesInbox::ScanResult PdesInbox::Scan(uint32_t self_core) const {
  ScanResult res;

  for (uint32_t src = 0; src < num_cores_; ++src) {
    if (src == self_core) {
      continue;
    }

    uint64_t lb = channel_lb_->at(src).value.load(std::memory_order_acquire);
    uint64_t qmin = NextTimestampFrom(src);
    uint64_t cmin = std::min(lb, qmin);

    if (cmin < res.safe_time) {
      res.safe_time = cmin;
    }
    if (qmin < res.min_ts) {
      res.min_ts = qmin;
      res.min_src = src;
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

  for (uint32_t src = 0; src < num_cores_; ++src) {
    if (src == self_core_) {
      continue;
    }

    uint64_t ts = NextTimestampFrom(src);
    if (ts < min_ts) {
      min_ts = ts;
      min_src = src;
    }
  }

  if (min_src == num_cores_ || min_ts > safe_time) {
    return false;
  }

  if (!rings_->at(min_src)->Read(read_seq_[min_src], msg)) {
    return false;
  }

  msg.dst = self_core_;
  read_seq_[min_src]++;
  return true;
}

bool PdesInbox::TryPopFrom(uint32_t src, uint64_t safe_time, PdesMsg &msg) {
  if (src >= num_cores_ || src == self_core_) {
    return false;
  }

  if (!rings_->at(src)->Read(read_seq_[src], msg)) {
    return false;
  }
  if (msg.ts > safe_time) {
    return false;
  }
  msg.dst = self_core_;
  read_seq_[src]++;
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

  rings_.reserve(num_cores);
  channel_lb_.reserve(num_cores);
  inboxes_.reserve(num_cores);

  for (uint32_t i = 0; i < num_cores; ++i) {
    rings_.push_back(std::make_unique<PdesRing>());
    channel_lb_.push_back(ChannelLowerBound{});
  }

  for (uint32_t i = 0; i < num_cores; ++i) {
    inboxes_.push_back(
        std::make_unique<PdesInbox>(i, rings_, channel_lb_));
  }
}

PdesInbox &PdesChannelManager::GetInbox(uint32_t core_id) {
  return *inboxes_[core_id];
}

void PdesChannelManager::AppendReal(uint32_t src_core, uint64_t arrival_time,
                                    PdesMsgKind kind, uint64_t line_addr) {
  rings_[src_core]->Append(arrival_time, kind, src_core, line_addr);
}

void PdesChannelManager::UpdateLowerBound(uint32_t src_core, uint64_t lb) {
  // Single-writer per entry (each core only updates its own lb), so a simple
  // store is sufficient and avoids CAS traffic.
  channel_lb_[src_core].value.store(lb, std::memory_order_release);
}
