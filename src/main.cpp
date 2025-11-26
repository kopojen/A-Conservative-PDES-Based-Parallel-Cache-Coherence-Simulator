/**
 * @file main.cpp
 * @brief Phase 1: Baseline Sequential Cache Coherence Simulator
 *
 * Processes timestamped multi-core memory traces in strict order.
 * Uses global priority queue to ensure correctness.
 * Implements MSI cache coherence protocol.
 */

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "msi_cache.h"
#include "trace_reader.h"

namespace {

struct ScheduledEvent {
  TraceEvent ev;
  size_t stream_index{};
};

struct EventEarlier {
  bool operator()(const ScheduledEvent& a, const ScheduledEvent& b) const {
    if (a.ev.timestamp != b.ev.timestamp) {
      return a.ev.timestamp > b.ev.timestamp;
    }
    return a.ev.core_id > b.ev.core_id;
  }
};

struct CoreStream {
  uint32_t core_id{};
  std::unique_ptr<TraceReader> reader;
};

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

    uint32_t max_core_id = 0;
    for (const auto& s : streams) {
      max_core_id = std::max(max_core_id, s.core_id);
    }
    const uint32_t num_cores = max_core_id + 1;

    std::cout << "=== Baseline Sequential Simulator ===\n";
    std::cout << "Cores: " << num_cores << "\n";
    std::cout << "Trace files: " << streams.size() << "\n";
    std::cout << "Starting simulation...\n\n";

    MSICache cache(num_cores);

    std::priority_queue<ScheduledEvent, std::vector<ScheduledEvent>,
                        EventEarlier>
        queue;

    // Seed queue with first event from each stream
    for (size_t i = 0; i < streams.size(); ++i) {
      if (auto ev = streams[i].reader->Next()) {
        ev->core_id = streams[i].core_id;
        ScheduledEvent se;
        se.ev = *ev;
        se.stream_index = i;
        queue.push(se);
      }
    }

    // Process events in timestamp order
    uint64_t processed = 0;
    while (!queue.empty()) {
      ScheduledEvent cur = queue.top();
      queue.pop();
      cache.Process(cur.ev);
      processed++;

      if (auto next = streams[cur.stream_index].reader->Next()) {
        next->core_id = streams[cur.stream_index].core_id;
        ScheduledEvent se;
        se.ev = *next;
        se.stream_index = cur.stream_index;
        queue.push(se);
      }
    }

    // Output results
    std::cout << "=== Simulation Complete ===\n";
    std::cout << "Total events processed: " << processed << "\n\n";

    const auto& stats = cache.stats();
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
