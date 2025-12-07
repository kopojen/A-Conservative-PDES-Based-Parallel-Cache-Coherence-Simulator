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
g++ -std=c++17 -O2 -Wall -Wextra -pedantic \
    -I./include src/main.cpp src/trace_reader.cpp src/msi_cache.cpp \
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