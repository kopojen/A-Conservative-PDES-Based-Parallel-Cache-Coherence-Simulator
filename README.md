# Cache Coherence Simulator

Sequential simulator for multi-core memory traces using MSI cache coherence protocol.

## Quick Start

### 1. Prepare Traces

Either by using the traces we have processed or by downloading the original traces.

1. Use the dataset we have processed
    ```bash
    cd dataset
    ./upzip-all.sh
    ```
    You will get all traces under dataset/data

2. Download the original dataset
    Please follow the instruction in [dataset/README.md](dataset/README.md#raw-traces) to download the raw traces and process them.

### 2. Compile

```bash
g++ -std=c++17 -O2 -Wall -Wextra -pedantic -pthread \
    -I./include src/main.cpp src/trace_reader.cpp src/msi_cache.cpp src/bus.cpp \
    -o baseline
```

### 3. Run

Single core:
```bash
./baseline dataset/data/MT0-canneal
```

Multiple cores:
```bash
./baseline dataset/data/MT{0,1,2,3}-canneal
```

## Cache Configuration

The simulator models private L1 caches with the following parameters:
- 64-byte cache lines (addresses normalized to line granularity)
- 32 KB capacity per core, implemented as 64 sets with 8-way associativity
- True LRU replacement within each set
- MSI coherence maintained via a snoopy bus protocol
- All caches are L1 only; there is no shared lower-level cache

## Code Structure

### `src/main.cpp`
- Parses command-line arguments and builds a trace reader per requested core
- Registers every core with the global snoopy bus and spawns one worker thread per core
- Each worker thread processes its own trace stream, drives its private cache, and consumes bus events
- Global hit/miss statistics are aggregated after all workers finish

### `src/trace_reader.cpp`
- Memory-maps trace files for zero-copy parsing and guarantees safe unmapping
- Auto-detects and handles different address formats (hex/decimal)
- Yields events one-by-one

### `src/msi_cache.cpp`
- Implements MSI cache coherence protocol
- Maintains cache line states: Invalid, Shared, Modified
- Tracks per-core hit/miss statistics

### `src/bus.cpp`
- Implements the global lock-free ring buffer used to broadcast snoopy events
- Each broadcast reserves a slot via an atomic tail pointer; subscribers track their own read indices
