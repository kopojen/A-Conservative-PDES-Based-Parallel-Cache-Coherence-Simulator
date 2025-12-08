/**
 * @file main.cpp
 * @brief Phase 1: Baseline Sequential Cache Coherence Simulator
 *
 * Processes timestamped multi-core memory traces in strict order.
 * Uses global priority queue to ensure correctness.
 * Implements MSI cache coherence protocol.
 */

#include <algorithm>
#include <atomic>
#include <csignal>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "bus.h"
#include "msi_cache.h"
#include "trace_reader.h"

namespace {

struct CoreStream {
  uint32_t core_id{};
  std::unique_ptr<TraceReader> reader;
};

std::atomic<bool> g_stop_requested{false};
volatile std::sig_atomic_t g_signal_flag = 0;

constexpr uint64_t kLineSizeBytes = 64;
constexpr uint64_t kCacheSizeBytes = 32 * 1024;
constexpr uint32_t kAssociativity = 8;

uint64_t NormalizeAddress(uint64_t addr) {
  return addr & ~(kLineSizeBytes - 1);
}

struct CoreWorker {
  CoreWorker(uint32_t id, std::unique_ptr<TraceReader> reader_ptr, Bus& bus_ref,
             CoreStats& stats_ref)
      : core_id(id),
        reader(std::move(reader_ptr)),
        cache(id, kCacheSizeBytes, kLineSizeBytes, kAssociativity),
        stats(&stats_ref),
        bus(&bus_ref) {
    mailbox = bus->RegisterMailbox(core_id);
  }

  uint32_t core_id{0};
  std::unique_ptr<TraceReader> reader;
  MSICache cache;
  CoreStats* stats{nullptr};
  Bus* bus{nullptr};
  std::shared_ptr<BusMailbox> mailbox;
  std::thread thread;
};

bool StopRequested() {
  if (g_signal_flag != 0) {
    g_stop_requested.store(true, std::memory_order_relaxed);
  }
  return g_stop_requested.load(std::memory_order_relaxed);
}

void HandleSignal(int) {
  g_signal_flag = 1;
}

void InstallSignalHandlers() {
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);
}

void DrainBusMessages(CoreWorker& worker) {
  BusMessage msg;
  while (worker.mailbox->TryPop(msg)) {
    if (msg.type == BusMessageType::ReadMiss) {
      worker.cache.HandleExternalReadMiss(msg.line_addr);
    } else {
      worker.cache.HandleExternalWriteMiss(msg.line_addr);
    }
  }
}

void HandleRead(CoreWorker& worker, uint64_t line_addr) {
  auto& stats = *worker.stats;
  const auto state = worker.cache.GetState(line_addr);

  if (state == MSICache::State::Shared ||
      state == MSICache::State::Modified) {
    stats.hits++;
    worker.cache.Touch(line_addr);
    return;
  }

  stats.misses++;
  BusMessage msg{BusMessageType::ReadMiss, worker.core_id, line_addr};
  worker.bus->Broadcast(msg);
  worker.cache.InsertOrUpdate(line_addr, MSICache::State::Shared);
}

void HandleWrite(CoreWorker& worker, uint64_t line_addr) {
  auto& stats = *worker.stats;
  const auto state = worker.cache.GetState(line_addr);

  if (state == MSICache::State::Modified) {
    stats.hits++;
    worker.cache.Touch(line_addr);
    return;
  }

  stats.misses++;
  BusMessage msg{BusMessageType::WriteMiss, worker.core_id, line_addr};
  worker.bus->Broadcast(msg);

  if (state == MSICache::State::Shared) {
    worker.cache.SetState(line_addr, MSICache::State::Modified);
    worker.cache.Touch(line_addr);
  } else {
    worker.cache.InsertOrUpdate(line_addr, MSICache::State::Modified);
  }
}

void RunCore(CoreWorker* worker) {
  if (!worker) return;
  while (!StopRequested()) {
    DrainBusMessages(*worker);
    auto ev = worker->reader->Next();
    if (!ev) break;

    ev->core_id = worker->core_id;
    const uint64_t line_addr = NormalizeAddress(ev->address);

    if (ev->is_write) {
      worker->stats->writes++;
      HandleWrite(*worker, line_addr);
    } else {
      worker->stats->reads++;
      HandleRead(*worker, line_addr);
    }
  }
  DrainBusMessages(*worker);
}

void PrintUsage() {
  std::cout << "Usage: baseline [--trace CORE_ID:PATH | PATH]...\n"
            << "Examples:\n"
            << "  baseline --trace 0:trace0.ts --trace 1:trace1.ts\n"
            << "  baseline trace0.ts trace1.ts\n";
}

std::vector<std::string> CollectTraceSpecs(int argc, char* argv[]) {
  std::vector<std::string> specs;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--trace" && i + 1 < argc) {
      specs.emplace_back(argv[++i]);
    } else if (arg == "-h" || arg == "--help") {
      PrintUsage();
      std::exit(0);
    } else {
      specs.emplace_back(arg);
    }
  }
  return specs;
}

std::vector<CoreStream> BuildStreams(const std::vector<std::string>& specs) {
  if (specs.empty()) {
    throw std::runtime_error("No trace paths provided.");
  }

  std::vector<CoreStream> streams;
  uint32_t next_core_id = 0;

  for (const auto& spec : specs) {
    uint32_t core_id = next_core_id;
    std::string path_str = spec;

    const auto colon = spec.find(':');
    if (colon != std::string::npos) {
      core_id = static_cast<uint32_t>(std::stoul(spec.substr(0, colon)));
      path_str = spec.substr(colon + 1);
    } else {
      next_core_id++;
    }

    std::filesystem::path p(path_str);
    streams.emplace_back();
    streams.back().core_id = core_id;
    streams.back().reader = std::make_unique<TraceReader>(p, core_id);
    if (colon != std::string::npos) {
      next_core_id = std::max(next_core_id, core_id + 1);
    }
  }

  return streams;
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    const auto specs = CollectTraceSpecs(argc, argv);
    auto streams = BuildStreams(specs);

    InstallSignalHandlers();

    uint32_t max_core_id = 0;
    for (const auto& s : streams) {
      max_core_id = std::max(max_core_id, s.core_id);
    }
    const uint32_t num_cores = max_core_id + 1;

    std::cout << "=== Baseline Sequential Simulator ===\n";
    std::cout << "Cores: " << num_cores << "\n";
    std::cout << "Trace files: " << streams.size() << "\n";
    std::cout << "Starting simulation...\n\n";

    Bus bus(num_cores);
    std::vector<CoreStats> stats(num_cores);
    std::vector<std::unique_ptr<TraceReader>> readers(num_cores);

    for (auto& stream : streams) {
      if (stream.core_id >= num_cores) {
        throw std::runtime_error("core_id out of range when assigning readers");
      }
      if (readers[stream.core_id]) {
        throw std::runtime_error("Multiple traces assigned to core " +
                                 std::to_string(stream.core_id));
      }
      readers[stream.core_id] = std::move(stream.reader);
    }

    std::vector<std::unique_ptr<CoreWorker>> workers;
    workers.reserve(num_cores);

    for (uint32_t core = 0; core < num_cores; ++core) {
      if (!readers[core]) {
        throw std::runtime_error("Missing trace for core " +
                                 std::to_string(core));
      }

      auto worker = std::make_unique<CoreWorker>(core, std::move(readers[core]),
                                                 bus, stats[core]);
      worker->thread = std::thread(RunCore, worker.get());
      workers.push_back(std::move(worker));
    }

    for (auto& worker : workers) {
      if (worker->thread.joinable()) {
        worker->thread.join();
      }
    }

    uint64_t processed = 0;
    for (const auto& s : stats) {
      processed += s.reads + s.writes;
    }

    std::cout << "=== Simulation Complete ===\n";
    std::cout << "Total events processed: " << processed << "\n\n";

    uint64_t total_accesses = 0;
    uint64_t total_hits = 0;

    for (size_t core = 0; core < stats.size(); ++core) {
      const auto& s = stats[core];
      uint64_t accesses = s.reads + s.writes;
      total_accesses += accesses;
      total_hits += s.hits;

      std::cout << "Core " << core << ":\n"
                << "  Reads  : " << s.reads << "\n"
                << "  Writes : " << s.writes << "\n"
                << "  Hits   : " << s.hits << "\n"
                << "  Misses : " << s.misses << "\n";

      if (accesses > 0) {
        double hit_rate = 100.0 * s.hits / accesses;
        std::cout << "  Hit Rate: " << std::fixed << std::setprecision(2)
                  << hit_rate << "%\n";
      }
      std::cout << "\n";
    }

    if (total_accesses > 0) {
      double overall_hit_rate = 100.0 * total_hits / total_accesses;
      std::cout << "Overall Hit Rate: " << std::fixed << std::setprecision(2)
                << overall_hit_rate << "%\n";
    }

    return 0;
  } catch (const std::exception& exc) {
    std::cerr << "Error: " << exc.what() << "\n\n";
    PrintUsage();
    return 1;
  }
}
