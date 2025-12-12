/**
 * @file main.cpp
 * @brief Conservative PDES Cache Coherence Simulator with CMB protocol.
 *
 * Processes timestamped multi-core memory traces using Conservative
 * Parallel Discrete Event Simulation with Chandy-Misra-Bryant protocol.
 * Implements MSI cache coherence protocol.
 */

#include <algorithm>
#include <atomic>
#include <csignal>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "msi_cache.h"
#include "pdes_channel.h"
#include "pdes_worker.h"
#include "trace_reader.h"

namespace {

struct CoreStream {
  uint32_t core_id{};
  std::unique_ptr<TraceReader> reader;
};

volatile std::sig_atomic_t g_signal_flag = 0;

void HandleSignal(int) { g_signal_flag = 1; }

void InstallSignalHandlers() {
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);
}

void PrintUsage() {
  std::cout << "Usage: simulator [OPTIONS] [--trace CORE_ID:PATH | PATH]...\n"
            << "\nOptions:\n"
            << "  --trace PATH     Specify trace file (can use CORE_ID:PATH "
               "format)\n"
            << "  -h, --help       Show this help message\n"
            << "\nExamples:\n"
            << "  simulator --trace 0:trace0.ts --trace 1:trace1.ts\n"
            << "  simulator trace0.ts trace1.ts\n";
}

std::vector<std::string> ParseCommandLine(int argc, char *argv[]) {
  std::vector<std::string> trace_specs;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--trace" && i + 1 < argc) {
      trace_specs.emplace_back(argv[++i]);
    } else if (arg == "-h" || arg == "--help") {
      PrintUsage();
      std::exit(0);
    } else if (arg[0] != '-') {
      trace_specs.emplace_back(arg);
    } else {
      throw std::runtime_error("Unknown option: " + arg);
    }
  }
  return trace_specs;
}

std::vector<CoreStream> BuildStreams(const std::vector<std::string> &specs) {
  if (specs.empty()) {
    throw std::runtime_error("No trace paths provided.");
  }

  std::vector<CoreStream> streams;
  uint32_t next_core_id = 0;

  for (const auto &spec : specs) {
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

void RunSimulation(std::vector<CoreStream> &streams, uint32_t num_cores) {
  PdesChannelManager channel_mgr(num_cores);
  std::vector<CoreStats> stats(num_cores);
  std::vector<std::unique_ptr<TraceReader>> readers(num_cores);

  for (auto &stream : streams) {
    if (stream.core_id >= num_cores) {
      throw std::runtime_error("core_id out of range when assigning readers");
    }
    if (readers[stream.core_id]) {
      throw std::runtime_error("Multiple traces assigned to core " +
                               std::to_string(stream.core_id));
    }
    readers[stream.core_id] = std::move(stream.reader);
  }

  std::atomic<bool> stop_flag{false};
  std::atomic<uint32_t> done_count{0};

  std::vector<std::unique_ptr<PdesWorker>> workers;
  workers.reserve(num_cores);

  for (uint32_t core = 0; core < num_cores; ++core) {
    if (!readers[core]) {
      throw std::runtime_error("Missing trace for core " +
                               std::to_string(core));
    }

    auto worker = std::make_unique<PdesWorker>(core, std::move(readers[core]),
                                               channel_mgr, stats[core],
                                               stop_flag, done_count);
    workers.push_back(std::move(worker));
  }

  // Start all workers
  for (auto &worker : workers) {
    worker->Start();
  }

  // Wait for all workers to complete
  for (auto &worker : workers) {
    worker->Join();
  }

  // Print results
  uint64_t processed = 0;
  for (const auto &s : stats) {
    processed += s.reads + s.writes;
  }

  std::cout << "=== Simulation Complete ===\n";
  std::cout << "Total events processed: " << processed << "\n\n";

  uint64_t total_accesses = 0;
  uint64_t total_hits = 0;

  for (size_t core = 0; core < stats.size(); ++core) {
    const auto &s = stats[core];
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
}

} // namespace

int main(int argc, char *argv[]) {
  try {
    const auto trace_specs = ParseCommandLine(argc, argv);
    auto streams = BuildStreams(trace_specs);

    InstallSignalHandlers();

    uint32_t max_core_id = 0;
    for (const auto &s : streams) {
      max_core_id = std::max(max_core_id, s.core_id);
    }
    const uint32_t num_cores = max_core_id + 1;

    std::cout << "=== Conservative PDES Cache Coherence Simulator ===\n";
    std::cout << "Cores: " << num_cores << "\n";
    std::cout << "Trace files: " << streams.size() << "\n";
    std::cout << "Bus Latency: " << kBusLatency << " cycle(s)\n";
    std::cout << "Starting simulation...\n\n";

    RunSimulation(streams, num_cores);

    return 0;
  } catch (const std::exception &exc) {
    std::cerr << "Error: " << exc.what() << "\n\n";
    PrintUsage();
    return 1;
  }
}
