/**
 * @file pdes_worker.h
 * @brief PDES LP (Logical Process) worker for Conservative parallel simulation.
 *
 * Each LP represents a core and processes events in timestamp order,
 * using CMB (Chandy-Misra-Bryant) protocol for deadlock-free execution.
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

#include "msi_cache.h"
#include "pdes_channel.h"
#include "trace_reader.h"

/**
 * @brief Debug info for stuck detection.
 */
struct PdesDebugInfo {
  uint32_t core_id{0};
  uint64_t lvt{0};
  uint64_t t_local{0};
  uint64_t t_inbox{0};
  uint64_t t_next{0};
  uint64_t safe_time{0};
  uint64_t last_lb{0};
  uint64_t last_msg_received{0};
  uint64_t stuck_count{0};
};

/**
 * @brief PDES LP Worker for a single core.
 *
 * Implements Conservative PDES with:
 * - LVT (Local Virtual Time) tracking
 * - CMB safe-time calculation
 * - Null message generation for lookahead
 */
class PdesWorker {
public:
  PdesWorker(uint32_t core_id, std::unique_ptr<TraceReader> reader,
             PdesChannelManager &channel_mgr, CoreStats &stats,
             std::atomic<bool> &stop_flag, std::atomic<uint32_t> &done_count);

  /// Start the worker thread
  void Start();

  /// Join the worker thread
  void Join();

  /// Get debug info for stuck detection
  PdesDebugInfo GetDebugInfo() const;

  uint32_t core_id() const { return core_id_; }

private:
  /// Main event loop
  void Run();

  /// Initialize: peek first event and send initial null messages
  void Initialize();

  /// Process a local trace event
  void ProcessLocalEvent(const TraceEvent &ev);

  /// Process an inbox message (coherence event from another core)
  void ProcessInboxEvent(const PdesMsg &msg);

  /// Handle a read access
  void HandleRead(uint64_t line_addr);

  /// Handle a write access
  void HandleWrite(uint64_t line_addr);

  /// Publish lower bound for this source (replaces null messages)
  void MaybePublishLowerBound(bool force = false);
  void UpdateLowerBoundCandidate();

  /// Normalize address to cache line
  static uint64_t NormalizeAddress(uint64_t addr);

  /// Check for stuck condition and log debug info
  void CheckStuck();

  // Configuration
  static constexpr uint64_t kLineSizeBytes = 64;
  static constexpr uint64_t kCacheSizeBytes = 32 * 1024;
  static constexpr uint32_t kAssociativity = 8;
  static constexpr uint64_t kStuckThreshold =
      100000; // iterations before warning

  uint32_t core_id_{0};
  std::unique_ptr<TraceReader> reader_;
  PdesChannelManager *channel_mgr_{nullptr};
  MSICache cache_;
  CoreStats *stats_{nullptr};
  std::atomic<bool> *stop_flag_{nullptr};
  std::atomic<uint32_t> *done_count_{nullptr};

  std::thread thread_;

  // PDES state
  uint64_t lvt_{0};               // Local Virtual Time
  uint64_t next_local_ts_{0};     // Timestamp of next local event
  bool has_next_local_{false};    // Whether there's a next local event
  uint64_t last_lb_published_{0}; // Last lower bound published
  uint64_t pending_lb_{0};        // Candidate lower bound to publish
  bool lb_dirty_{false};          // Whether pending_lb_ needs publishing
  uint64_t last_msg_received_{0}; // Last message received timestamp
  uint64_t stuck_iterations_{0};  // Counter for stuck detection
  bool trace_exhausted_{false};   // Whether trace has been fully processed
  bool done_signaled_{false};     // Whether done_count was incremented
  uint64_t next_outgoing_msg_id_{0}; // Per-source deterministic msg id
};
