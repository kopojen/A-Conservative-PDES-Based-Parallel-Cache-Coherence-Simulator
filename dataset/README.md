# Dataset

This directory contains raw USIMM trace files and tools for preprocessing.

## Raw Traces

- `MT0-canneal.zip`, `MT1-canneal.zip`, `MT2-canneal.zip`, `MT3-canneal.zip`: Compressed raw USIMM traces
- `data/`: Uncompressed traces (after running `./unzip-all.sh`)

Raw trace format: `count op address [pc]`

## Preprocessing

Convert raw traces to timestamped format:

```bash
./convert_all_traces.sh [NON_MEM_LATENCY] [MEM_LATENCY]
```

Examples:
```bash
# Use defaults (non-mem-latency=1, mem-latency=20)
./convert_all_traces.sh

# Custom parameters
./convert_all_traces.sh 1 20
./convert_all_traces.sh 2 30
```

Timestamped traces are saved to `timestamped_traces/` directory.

Timestamped trace format: `timestamp op address [pc]`

## Files

- `count_to_timestamp.py`: Convert raw traces to timestamped format
- `convert_all_traces.sh`: Batch convert all traces
- `unzip-all.sh`: Decompress all trace files

## Reference

Chatterjee, Niladrish, et al. "Usimm: the utah simulated memory module." University of Utah, Tech. Rep (2012).
