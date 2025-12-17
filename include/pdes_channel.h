/**
 * @file pdes_channel.h
 * @brief Conservative PDES channel with CMB (Chandy-Misra-Bryant) protocol.
 *
 * Implements timestamped LP-to-LP communication with:
 * - Real messages (coherence events) appended once per source
 * - Shared lower bounds (replacing null message broadcast)
 * - Safe-time calculation for deadlock-free execution
 */

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

constexpr size_t kRingCapacity = 1ull << 20; // Per-source ring capacity

/// Bus latency in cycles (lookahead > 0 to avoid deadlock)
constexpr uint64_t kBusLatency = 1;

/// Sentinel value for "no timestamp" / infinity
constexpr uint64_t kTimestampInfinity = std::numeric_limits<uint64_t>::max();

/// Per-core lower bound with padding to avoid cache line sharing.
struct alignas(64) ChannelLowerBound {
  static constexpr size_t kPaddingBytes = 64 - sizeof(std::atomic<uint64_t>);
  std::atomic<uint64_t> value{0};
  uint8_t padding[kPaddingBytes]{};

  ChannelLowerBound() = default;
  ChannelLowerBound(const ChannelLowerBound &) = delete;
  ChannelLowerBound &operator=(const ChannelLowerBound &) = delete;
  ChannelLowerBound(ChannelLowerBound &&other) noexcept {
    value.store(other.value.load(std::memory_order_relaxed),
                std::memory_order_relaxed);
  }
  ChannelLowerBound &operator=(ChannelLowerBound &&other) noexcept {
    value.store(other.value.load(std::memory_order_relaxed),
                std::memory_order_relaxed);
    return *this;
  }
};
static_assert(sizeof(ChannelLowerBound) == 64,
              "ChannelLowerBound must fill a cache line");

/**
 * @brief Message kind for PDES communication.
 */
enum class PdesMsgKind {
  Null,         ///< Null message for advancing lower bound
  RealReadMiss, ///< Cache read miss event
  RealWriteMiss ///< Cache write miss event
};

/**
 * @brief PDES message structure.
 */
struct PdesMsg {
  PdesMsgKind kind{PdesMsgKind::Null};
  uint32_t src{0};       ///< Source core ID
  uint32_t dst{0};       ///< Destination core ID
  uint64_t ts{0};        ///< Timestamp (arrival time at destination)
  uint64_t line_addr{0}; ///< Cache line address (for real messages)

  /// Comparison for min-heap (earlier timestamp = higher priority)
  bool operator>(const PdesMsg &other) const {
    if (ts != other.ts)
      return ts > other.ts;
    // Tie-breaker: null messages have lower priority than real messages
    if (kind == PdesMsgKind::Null && other.kind != PdesMsgKind::Null)
      return true;
    if (kind != PdesMsgKind::Null && other.kind == PdesMsgKind::Null)
      return false;
    return src > other.src;
  }
};

/**
 * @brief Thread-safe inbox for receiving PDES messages.
 *
 * Implements a lock-free shared broadcast log:
 * - Each source core owns a single-writer ring buffer.
 * - All destination cores maintain per-source read pointers.
 * - Lower bounds are advertised via a shared atomic array (replacing null
 *   messages).
 */
class PdesRing {
public:
  PdesRing() : capacity_(kRingCapacity), buffer_(capacity_) {}

  void Append(uint64_t ts, PdesMsgKind kind, uint32_t src,
              uint64_t line_addr);
  bool Read(uint64_t seq, PdesMsg &msg) const;
  uint64_t PublishedSeq() const;

private:
  struct Slot {
    std::atomic<uint64_t> seq{kTimestampInfinity};
    PdesMsgKind kind{PdesMsgKind::Null};
    uint32_t src{0};
    uint64_t ts{0};
    uint64_t line_addr{0};
  };

  size_t capacity_;
  std::vector<Slot> buffer_;
  alignas(64) std::atomic<uint64_t> next_seq_{0};
};

class PdesInbox {
public:
  PdesInbox(uint32_t self_core,
            const std::vector<std::unique_ptr<PdesRing>> &rings,
            const std::vector<ChannelLowerBound> &channel_lb);

  /// Try to pop the earliest readable message (non-blocking)
  bool TryPop(PdesMsg &msg, uint64_t safe_time);

  /// Try to pop from a specific source (non-blocking, assumes caller picked
  /// earliest source)
  bool TryPopFrom(uint32_t src, uint64_t safe_time, PdesMsg &msg);

  struct ScanResult {
    uint64_t min_ts{kTimestampInfinity};
    uint32_t min_src{std::numeric_limits<uint32_t>::max()};
    uint64_t safe_time{kTimestampInfinity};
  };

  /// Single pass to get earliest inbox ts and safe_time together
  ScanResult Scan(uint32_t self_core) const;

  /// Peek at the earliest unread remote message timestamp
  uint64_t PeekMinTimestamp() const;

  /// Get the lower bound from a specific source channel
  uint64_t GetChannelLowerBound(uint32_t src) const;

  /// Compute the CMB safe time: min over all channels of min(qmin, lb)
  uint64_t ComputeSafeTime(uint32_t self_core) const;

  /// Get the minimum unread real message timestamp from a specific source
  uint64_t GetMinRealMsgTimestamp(uint32_t src) const;

  /// Lightweight wait to avoid busy spinning
  void WaitForMessage();
  /// Reset adaptive backoff after progress
  void ResetWaitBackoff();

private:
  uint64_t NextTimestampFrom(uint32_t src) const;

  const std::vector<std::unique_ptr<PdesRing>> *rings_{nullptr};
  const std::vector<ChannelLowerBound> *channel_lb_{nullptr};
  std::vector<uint64_t> read_seq_;
  uint32_t num_cores_{0};
  uint32_t self_core_{0};

  static thread_local uint32_t wait_backoff_;
};

/**
 * @brief PDES channel manager for all LP-to-LP communication.
 *
 * Each LP has its own inbox. Messages are sent by enqueueing to
 * the destination LP's inbox.
 */
class PdesChannelManager {
public:
  explicit PdesChannelManager(uint32_t num_cores);

  /// Get inbox for a specific LP
  PdesInbox &GetInbox(uint32_t core_id);

  /// Append a real coherence event to the source ring (arrival_time already
  /// includes bus latency).
  void AppendReal(uint32_t src_core, uint64_t arrival_time, PdesMsgKind kind,
                  uint64_t line_addr);

  /// Update the shared lower bound (replaces null messages)
  void UpdateLowerBound(uint32_t src_core, uint64_t lb);

  uint32_t num_cores() const { return num_cores_; }

private:
  uint32_t num_cores_{0};
  std::vector<std::unique_ptr<PdesInbox>> inboxes_;
  std::vector<std::unique_ptr<PdesRing>> rings_;
  std::vector<ChannelLowerBound> channel_lb_;
};
