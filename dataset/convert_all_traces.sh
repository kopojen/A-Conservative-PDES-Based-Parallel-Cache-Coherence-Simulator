#!/bin/bash
# Usage:
#   ./convert_all_traces.sh [NON_MEM_LATENCY] [MEM_LATENCY]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA_DIR="${SCRIPT_DIR}/data"

# Default latency values
NON_MEM_LATENCY="${1:-1}"
MEM_LATENCY="${2:-20}"

echo "Converting traces with non-mem-latency=${NON_MEM_LATENCY}, mem-latency=${MEM_LATENCY}"

# Find all trace files in data directory
TRACE_FILES=()
for dir in "${DATA_DIR}"/*; do
    if [[ -d "$dir" ]]; then
        for trace in "$dir"/*; do
            if [[ -f "$trace" ]]; then
                TRACE_FILES+=("$trace")
            fi
        done
    elif [[ -f "$dir" ]]; then
        TRACE_FILES+=("$dir")
    fi
done

if [[ ${#TRACE_FILES[@]} -eq 0 ]]; then
    echo "Error: No trace files found in ${DATA_DIR}/" >&2
    echo "Please run ./unzip-all.sh first to extract the traces." >&2
    exit 1
fi

echo "Found ${#TRACE_FILES[@]} trace file(s)"

# Run the conversion
python3 "${SCRIPT_DIR}/count_to_timestamp.py" \
    --non-mem-latency "${NON_MEM_LATENCY}" \
    --mem-latency "${MEM_LATENCY}" \
    "${TRACE_FILES[@]}"

echo "Done! Timestamped traces saved to timestamped_traces/"
