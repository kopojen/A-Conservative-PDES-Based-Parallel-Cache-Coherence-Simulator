# Cache Coherence Simulator

Multi-core cache simulator driven by timestamped traces.

- Each core is modeled as a Logical Process (LP) running in its own thread.
- Coherence uses an MSI-like snoopy broadcast model.
- Parallel execution uses **Conservative PDES** with **CMB (Chandy–Misra–Bryant)** safe-time.
- Inter-core coherence events are communicated via a **lock-free per-source ring buffer**
  plus a shared **atomic lower-bound** array (replacing null-message broadcast).

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

g++ -std=c++17 -O2 -Wall -Wextra -pedantic -pthread \
    -I./include src/main.cpp src/trace_reader.cpp src/msi_cache.cpp src/pdes_channel.cpp src/pdes_worker.cpp \
    -o pdes
```

### 3. Run

Single core:
```bash
./baseline dataset/core0.trace
```

Multiple cores:
```bash
./pdes dataset/core*.trace
```

## Cache Configuration

The simulator models private L1 caches with the following parameters:
- 64-byte cache lines (addresses normalized to line granularity)
- 32 KB capacity per core, implemented as 64 sets with 8-way associativity
- True LRU replacement within each set
- MSI coherence maintained via a snoopy broadcast model
- All caches are L1 only; there is no shared lower-level cache

## PDES / Coherence Model (Current Implementation)

### Event types
- **Local events**: timestamped `R/W` accesses from each core's trace.
- **Remote events**: coherence snoop events generated on local misses:
  - `RealReadMiss`
  - `RealWriteMiss`

### Timing
- A coherence event generated at local virtual time `lvt` is observed by other cores at
  `arrival_time = lvt + kBusLatency` (see `include/pdes_channel.h`).
- Each core continuously publishes a **lower bound** `lb[core]`:
  `lb = next_local_ts + kBusLatency` (or `∞` when the trace is exhausted).

### Safe-time (CMB)
Each core computes:
\[
safe\_time = \min_{src \neq self} \min(lb[src], qmin[src])
\]
where `qmin[src]` is the timestamp of the next unread remote event from source `src`.

### Determinism note
To make results deterministic with lock-free rings, the scheduler enforces:
- **Remote events are processed before local events on timestamp ties**, and
- **Local events at the boundary are only allowed when `t_local < safe_time` (strict)**.

This avoids run-to-run variation when a remote event at the same timestamp exists but
is not yet visible during the current scan.

## Code Structure

### `src/main.cpp`
- Parses command-line arguments and builds a trace reader per requested core
- Creates the PDES channel manager and spawns one worker thread per core
- Each worker thread processes its own trace stream, drives its private cache, and consumes coherence events
- Global hit/miss statistics are aggregated after all workers finish

### `src/trace_reader.cpp`
- Memory-maps trace files for zero-copy parsing and guarantees safe unmapping
- Auto-detects and handles different address formats (hex/decimal)
- Yields events one-by-one

### `src/msi_cache.cpp`
- Implements MSI cache coherence protocol
- Maintains cache line states: Invalid, Shared, Modified
- Tracks per-core hit/miss statistics

### `src/pdes_channel.cpp`
- Implements the lock-free **per-source ring buffers** for coherence events
- Implements the shared **lower bound** array used by CMB safe-time

### `src/pdes_worker.cpp`
- One worker per core (LP)
- Conservative PDES main loop: chooses the next safe event (local vs remote) and updates the private cache
