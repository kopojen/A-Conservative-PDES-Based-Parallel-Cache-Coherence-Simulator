/**
 * @file pdes_channel.h
 * @brief Conservative PDES channel with CMB (Chandy-Misra-Bryant) protocol.
 *
 * Implements timestamped LP-to-LP communication with:
 * - Real messages (coherence events)
 * - Null messages (for advancing lower bounds)
 * - Safe-time calculation for deadlock-free execution
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <map>
#include <mutex>
#include <queue>
#include <vector>

/// Bus latency in cycles (lookahead > 0 to avoid deadlock)
constexpr uint64_t kBusLatency = 1;

/// Sentinel value for "no timestamp" / infinity
constexpr uint64_t kTimestampInfinity = std::numeric_limits<uint64_t>::max();

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
 * Uses a min-heap ordered by timestamp. Maintains per-source lower bounds
 * for CMB safe-time calculation.
 */
class PdesInbox {
public:
  explicit PdesInbox(uint32_t num_cores);

  /// Enqueue a message (thread-safe)
  void Push(const PdesMsg &msg);

  /// Try to pop the earliest message if safe (non-blocking)
  /// Returns true if a message was popped
  bool TryPop(PdesMsg &msg, uint64_t safe_time);

  /// Peek at the earliest message timestamp (kTimestampInfinity if empty)
  uint64_t PeekMinTimestamp() const;

  /// Get the lower bound from a specific source channel
  uint64_t GetChannelLowerBound(uint32_t src) const;

  /// Update the lower bound from a source (from null message)
  void UpdateChannelLowerBound(uint32_t src, uint64_t lb);

  /// Compute the CMB safe time: min over all channels of min(qmin, lb)
  uint64_t ComputeSafeTime(uint32_t self_core) const;

  /// Check if inbox is empty
  bool Empty() const;

  /// Get the minimum real message timestamp from a specific source
  uint64_t GetMinRealMsgTimestamp(uint32_t src) const;

  /// Block until there's a message or condition changes
  void WaitForMessage();

  /// Wake up any waiting threads
  void NotifyAll();

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;

  /// Min-heap of messages ordered by timestamp
  std::priority_queue<PdesMsg, std::vector<PdesMsg>, std::greater<PdesMsg>>
      heap_;

  /// Lower bound from each source channel (from null messages)
  std::vector<std::atomic<uint64_t>> channel_lb_;

  /// Per-source multimap to efficiently track messages by timestamp
  /// Key: timestamp, Value: message
  /// This allows O(log n) insertion and O(1) minimum lookup per source
  std::vector<std::multimap<uint64_t, PdesMsg>> per_source_msgs_;

  uint32_t num_cores_{0};
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

  /// Send a message to a destination LP
  void Send(const PdesMsg &msg);

  /// Broadcast a real message to all other LPs (with bus latency)
  void BroadcastReal(uint32_t src_core, uint64_t event_time, PdesMsgKind kind,
                     uint64_t line_addr);

  /// Send null messages to all other LPs
  void BroadcastNull(uint32_t src_core, uint64_t null_ts);

  uint32_t num_cores() const { return num_cores_; }

private:
  uint32_t num_cores_{0};
  std::vector<std::unique_ptr<PdesInbox>> inboxes_;
};
