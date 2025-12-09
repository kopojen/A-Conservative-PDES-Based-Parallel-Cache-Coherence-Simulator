/**
 * @file bus.cpp
 * @brief Lock-free global ring buffer for snoopy events.
 */

#include "bus.h"

#include <memory>
#include <stdexcept>
#include <thread>

BusSubscription::BusSubscription(Bus* bus, uint32_t core_id,
                                 uint64_t start_seq)
    : bus_(bus), core_id_(core_id), read_index_(start_seq) {}

bool BusSubscription::Next(BusMessage& msg) {
  if (!bus_) return false;
  return bus_->Poll(read_index_, msg);
}

Bus::Bus(uint32_t num_cores) {
  if (num_cores == 0) {
    throw std::invalid_argument("num_cores must be > 0");
  }
  slots_ = std::make_unique<Slot[]>(capacity_);
  for (size_t i = 0; i < capacity_; ++i) {
    slots_[i].ready.store(0, std::memory_order_relaxed);
  }
}

BusSubscription Bus::RegisterSubscription(uint32_t core_id) {
  return BusSubscription(this, core_id, tail_.load(std::memory_order_acquire));
}

void Bus::Broadcast(const BusMessage& msg) {
  const uint64_t seq = tail_.fetch_add(1, std::memory_order_acq_rel);
  const size_t idx = static_cast<size_t>(seq & mask_);
  slots_[idx].msg = msg;
  slots_[idx].ready.store(seq + 1, std::memory_order_release);
}

bool Bus::Poll(uint64_t& read_index, BusMessage& msg) {
  while (true) {
    uint64_t current_tail = tail_.load(std::memory_order_acquire);
    if (read_index >= current_tail) {
      return false;
    }

    const uint64_t safe_start =
        (current_tail > capacity_) ? current_tail - capacity_ : 0;
    if (read_index < safe_start) {
      read_index = safe_start;
      continue;
    }

    const size_t idx = static_cast<size_t>(read_index & mask_);
    const uint64_t expected = read_index + 1;
    const uint64_t ready_value =
        slots_[idx].ready.load(std::memory_order_acquire);

    if (ready_value < expected) {
      std::this_thread::yield();
      continue;
    }

    msg = slots_[idx].msg;
    read_index++;
    return true;
  }
}
