# Cache Coherence Simulator

Sequential simulator for multi-core memory traces using MSI cache coherence protocol.

## Quick Start

### 1. Prepare Traces

```bash
cd dataset
./convert_all_traces.sh <non-memory latency> <memory latency>
cd ..
```

Example:
```bash
./convert_all_traces.sh 1 20
```

This converts raw USIMM traces to timestamped format. See `dataset/README.md` for details and custom parameters.

### 2. Compile

```bash
g++ -std=c++17 -O2 -Wall -Wextra -pedantic \
    -I./include src/main.cpp src/trace_reader.cpp src/msi_cache.cpp \
    -o baseline
```

### 3. Run

Single core:
```bash
./baseline dataset/timestamped_traces/MT0-canneal.ts
```

Multiple cores:
```bash
./baseline dataset/timestamped_traces/MT{0,1,2,3}-canneal.ts
```

## Code Structure

### `src/main.cpp`
- Parses command-line arguments
- Creates streaming trace readers for each core
- Merges all events in timestamp order using global priority queue
- Processes events sequentially through cache simulator

### `src/trace_reader.cpp`
- Parses timestamped trace files
- Auto-detects and handles different address formats (hex/decimal)
- Yields events one-by-one

### `src/msi_cache.cpp`
- Implements MSI cache coherence protocol
- Maintains cache line states: Invalid, Shared, Modified
- Tracks per-core hit/miss statistics
- Processes read/write operations and state transitions