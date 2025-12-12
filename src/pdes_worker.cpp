/**
 * @file pdes_worker.cpp
 * @brief Implementation of PDES LP Worker with CMB protocol.
 */

#include "pdes_worker.h"

#include <algorithm>
#include <iostream>

PdesWorker::PdesWorker(uint32_t core_id, std::unique_ptr<TraceReader> reader,
                       PdesChannelManager &channel_mgr, CoreStats &stats,
                       std::atomic<bool> &stop_flag,
                       std::atomic<uint32_t> &done_count)
    : core_id_(core_id), reader_(std::move(reader)), channel_mgr_(&channel_mgr),
      cache_(core_id, kCacheSizeBytes, kLineSizeBytes, kAssociativity),
      stats_(&stats), stop_flag_(&stop_flag), done_count_(&done_count) {}

void PdesWorker::Start() { thread_ = std::thread(&PdesWorker::Run, this); }

void PdesWorker::Join() {
  if (thread_.joinable()) {
    thread_.join();
  }
}

uint64_t PdesWorker::NormalizeAddress(uint64_t addr) {
  return addr & ~(kLineSizeBytes - 1);
}

void PdesWorker::Initialize() {
  // Peek the first trace event
  auto first_ev = reader_->Peek();
  if (first_ev) {
    next_local_ts_ = first_ev->timestamp;
    has_next_local_ = true;

    // Send initial null messages: "I won't send anything before t0 +
    // bus_latency"
    uint64_t initial_null_ts = next_local_ts_ + kBusLatency;
    channel_mgr_->BroadcastNull(core_id_, initial_null_ts);
    last_null_sent_ = initial_null_ts;
  } else {
    // No events in trace - send infinity null
    has_next_local_ = false;
    trace_exhausted_ = true;
    channel_mgr_->BroadcastNull(core_id_, kTimestampInfinity);
    last_null_sent_ = kTimestampInfinity;
  }
}

void PdesWorker::SendNullMessages() {
  uint64_t null_ts;

  if (has_next_local_) {
    // Next real message can only be sent when we process next_local_ts
    // So others won't see anything before next_local_ts + bus_latency
    null_ts = next_local_ts_ + kBusLatency;
  } else {
    // Trace exhausted - we won't send any more real messages
    null_ts = kTimestampInfinity;
  }

  // Only send if it advances the lower bound
  if (null_ts > last_null_sent_) {
    channel_mgr_->BroadcastNull(core_id_, null_ts);
    last_null_sent_ = null_ts;
  }
}

void PdesWorker::HandleRead(uint64_t line_addr) {
  const auto state = cache_.GetState(line_addr);

  if (state == MSICache::State::Shared || state == MSICache::State::Modified) {
    stats_->hits++;
    cache_.Touch(line_addr);
    return;
  }

  // Cache miss - broadcast read miss
  stats_->misses++;
  channel_mgr_->BroadcastReal(core_id_, lvt_, PdesMsgKind::RealReadMiss,
                              line_addr);
  cache_.InsertOrUpdate(line_addr, MSICache::State::Shared);
}

void PdesWorker::HandleWrite(uint64_t line_addr) {
  const auto state = cache_.GetState(line_addr);

  if (state == MSICache::State::Modified) {
    stats_->hits++;
    cache_.Touch(line_addr);
    return;
  }

  // Cache miss or upgrade - broadcast write miss
  stats_->misses++;
  channel_mgr_->BroadcastReal(core_id_, lvt_, PdesMsgKind::RealWriteMiss,
                              line_addr);

  if (state == MSICache::State::Shared) {
    cache_.SetState(line_addr, MSICache::State::Modified);
    cache_.Touch(line_addr);
  } else {
    cache_.InsertOrUpdate(line_addr, MSICache::State::Modified);
  }
}

void PdesWorker::ProcessLocalEvent(const TraceEvent &ev) {
  const uint64_t line_addr = NormalizeAddress(ev.address);

  if (ev.is_write) {
    stats_->writes++;
    HandleWrite(line_addr);
  } else {
    stats_->reads++;
    HandleRead(line_addr);
  }
}

void PdesWorker::ProcessInboxEvent(const PdesMsg &msg) {
  last_msg_received_ = msg.ts;

  if (msg.kind == PdesMsgKind::RealReadMiss) {
    cache_.HandleExternalReadMiss(msg.line_addr);
  } else if (msg.kind == PdesMsgKind::RealWriteMiss) {
    cache_.HandleExternalWriteMiss(msg.line_addr);
  }
  // Null messages are handled in PdesInbox::Push
}

void PdesWorker::CheckStuck() {
  if (stuck_iterations_ > 0 && stuck_iterations_ % kStuckThreshold == 0) {
    auto &inbox = channel_mgr_->GetInbox(core_id_);
    uint64_t safe_time = inbox.ComputeSafeTime(core_id_);
    uint64_t t_inbox = inbox.PeekMinTimestamp();
    uint64_t t_local = has_next_local_ ? next_local_ts_ : kTimestampInfinity;
    uint64_t t_next = std::min(t_local, t_inbox);

    std::cerr << "[PDES STUCK] Core " << core_id_ << ":\n"
              << "  LVT: " << lvt_ << "\n"
              << "  t_local: "
              << (t_local == kTimestampInfinity ? -1 : (int64_t)t_local) << "\n"
              << "  t_inbox: "
              << (t_inbox == kTimestampInfinity ? -1 : (int64_t)t_inbox) << "\n"
              << "  t_next: "
              << (t_next == kTimestampInfinity ? -1 : (int64_t)t_next) << "\n"
              << "  safe_time: "
              << (safe_time == kTimestampInfinity ? -1 : (int64_t)safe_time)
              << "\n"
              << "  last_null_sent: " << last_null_sent_ << "\n"
              << "  last_msg_received: " << last_msg_received_ << "\n"
              << "  Channel lower bounds:\n";

    for (uint32_t src = 0; src < channel_mgr_->num_cores(); ++src) {
      if (src != core_id_) {
        uint64_t lb = inbox.GetChannelLowerBound(src);
        uint64_t qmin = inbox.GetMinRealMsgTimestamp(src);
        std::cerr << "    src " << src << ": lb=" << lb << ", qmin="
                  << (qmin == kTimestampInfinity ? -1 : (int64_t)qmin) << "\n";
      }
    }
  }
}

PdesDebugInfo PdesWorker::GetDebugInfo() const {
  PdesDebugInfo info;
  info.core_id = core_id_;
  info.lvt = lvt_;
  info.t_local = has_next_local_ ? next_local_ts_ : kTimestampInfinity;
  info.last_null_sent = last_null_sent_;
  info.last_msg_received = last_msg_received_;
  info.stuck_count = stuck_iterations_;
  return info;
}

void PdesWorker::Run() {
  Initialize();

  auto &inbox = channel_mgr_->GetInbox(core_id_);
  uint32_t num_cores = channel_mgr_->num_cores();

  while (!stop_flag_->load(std::memory_order_relaxed)) {
    // Check if all cores are done
    if (done_count_->load(std::memory_order_acquire) >= num_cores) {
      break;
    }

    // Get timestamps for next events
    uint64_t t_local = has_next_local_ ? next_local_ts_ : kTimestampInfinity;
    uint64_t t_inbox = inbox.PeekMinTimestamp();
    uint64_t t_next = std::min(t_local, t_inbox);

    // If no more events to process
    if (t_next == kTimestampInfinity) {
      if (!trace_exhausted_) {
        trace_exhausted_ = true;
        // Send final null message
        channel_mgr_->BroadcastNull(core_id_, kTimestampInfinity);
        last_null_sent_ = kTimestampInfinity;
        done_count_->fetch_add(1, std::memory_order_release);
      }

      // Wait a bit and check again (other cores might still send messages)
      inbox.WaitForMessage();
      stuck_iterations_ = 0;
      continue;
    }

    // Compute CMB safe time
    uint64_t safe_time = inbox.ComputeSafeTime(core_id_);

    // Can we process the next event?
    if (t_next <= safe_time) {
      stuck_iterations_ = 0;

      if (t_inbox <= t_local && t_inbox != kTimestampInfinity) {
        // Process inbox event first (if tie, prefer inbox for consistency)
        PdesMsg msg;
        if (inbox.TryPop(msg, safe_time)) {
          lvt_ = msg.ts;
          ProcessInboxEvent(msg);
        }
      } else if (t_local != kTimestampInfinity) {
        // Process local event
        auto ev = reader_->Next();
        if (ev) {
          lvt_ = ev->timestamp;
          ProcessLocalEvent(*ev);

          // Update next local event
          auto next_ev = reader_->Peek();
          if (next_ev) {
            next_local_ts_ = next_ev->timestamp;
            has_next_local_ = true;
          } else {
            has_next_local_ = false;
          }

          // Send null messages to advance lower bounds
          SendNullMessages();
        }
      }
    } else {
      // Cannot process yet - wait for more null messages
      stuck_iterations_++;
      CheckStuck();

      // Send null messages to help other cores make progress
      SendNullMessages();

      // Brief wait to avoid busy spinning
      inbox.WaitForMessage();
    }
  }

  // Mark as done if not already
  if (!trace_exhausted_) {
    trace_exhausted_ = true;
    channel_mgr_->BroadcastNull(core_id_, kTimestampInfinity);
    done_count_->fetch_add(1, std::memory_order_release);
  }
}
