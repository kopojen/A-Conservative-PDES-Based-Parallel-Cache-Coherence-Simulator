/**
 * @file bus.h
 * @brief Simple message bus for propagating cross-core cache events.
 */

#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>

enum class BusMessageType { ReadMiss, WriteMiss };

struct BusMessage {
  BusMessageType type{};
  uint32_t source_core{0};
  uint64_t line_addr{0};
};

/**
 * @brief Thread-safe mailbox for a single core.
 */
class BusMailbox {
 public:
  void Push(const BusMessage& msg);
  bool TryPop(BusMessage& msg);

 private:
  std::mutex mutex_;
  std::queue<BusMessage> queue_;
};

/**
 * @brief Shared bus used to broadcast cache events between cores.
 */
class Bus {
 public:
  explicit Bus(uint32_t num_cores);

  std::shared_ptr<BusMailbox> RegisterMailbox(uint32_t core_id);
  void Broadcast(const BusMessage& msg);

 private:
  std::vector<std::shared_ptr<BusMailbox>> mailboxes_;
  std::mutex mutex_;
};
