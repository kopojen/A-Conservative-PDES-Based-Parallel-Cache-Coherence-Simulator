/**
 * @file bus.cpp
 * @brief Message bus implementation for cache coherence events.
 */

#include "bus.h"

#include <stdexcept>

void BusMailbox::Push(const BusMessage& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  queue_.push(msg);
}

bool BusMailbox::TryPop(BusMessage& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (queue_.empty()) {
    return false;
  }
  msg = queue_.front();
  queue_.pop();
  return true;
}

Bus::Bus(uint32_t num_cores) : mailboxes_(num_cores) {
  if (num_cores == 0) {
    throw std::invalid_argument("num_cores must be > 0");
  }
}

std::shared_ptr<BusMailbox> Bus::RegisterMailbox(uint32_t core_id) {
  auto mailbox = std::make_shared<BusMailbox>();
  std::lock_guard<std::mutex> lock(mutex_);
  if (core_id >= mailboxes_.size()) {
    throw std::out_of_range("core_id out of range in RegisterMailbox");
  }
  mailboxes_[core_id] = mailbox;
  return mailbox;
}

void Bus::Broadcast(const BusMessage& msg) {
  std::vector<std::shared_ptr<BusMailbox>> snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot = mailboxes_;
  }

  for (uint32_t core = 0; core < snapshot.size(); ++core) {
    if (core == msg.source_core) {
      continue;
    }
    const auto& mailbox = snapshot[core];
    if (mailbox) {
      mailbox->Push(msg);
    }
  }
}
