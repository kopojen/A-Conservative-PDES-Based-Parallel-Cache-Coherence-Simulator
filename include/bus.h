/**
 * @file bus.h
 * @brief Lock-free snoopy bus based on a global ring buffer.
 */

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>

enum class BusMessageType { ReadMiss, WriteMiss };

struct BusMessage {
  BusMessageType type{};
  uint32_t source_core{0};
  uint64_t line_addr{0};
};

class Bus;

/**
 * @brief Per-core reader that advances through the global event stream.
 */
class BusSubscription {
 public:
  BusSubscription() = default;
  bool Next(BusMessage& msg);
  uint32_t core_id() const { return core_id_; }

 private:
  friend class Bus;
  BusSubscription(Bus* bus, uint32_t core_id, uint64_t start_seq);

  Bus* bus_{nullptr};
  uint32_t core_id_{0};
  uint64_t read_index_{0};
};

/**
 * @brief Bus with global ring buffer for snoopy events.
 */
class Bus {
 public:
  explicit Bus(uint32_t num_cores);

  BusSubscription RegisterSubscription(uint32_t core_id);
  void Broadcast(const BusMessage& msg);

 private:
  friend class BusSubscription;
  struct Slot {
    BusMessage msg;
    std::atomic<uint64_t> ready{0};
  };

  bool Poll(uint64_t& read_index, BusMessage& msg);

  static constexpr size_t kCapacity = 1u << 16;
  static_assert((kCapacity & (kCapacity - 1)) == 0,
                "kCapacity must be power of two");

  const size_t capacity_{kCapacity};
  const size_t mask_{kCapacity - 1};
  std::atomic<uint64_t> tail_{0};
  std::unique_ptr<Slot[]> slots_;
};
