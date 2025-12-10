#!/usr/bin/env python3
"""
Analyze timestamped memory traces and report locality characteristics.

The script understands the same formats as the simulator:
    timestamp op address [pc]
    timestamp core_id op address [pc]

For each trace it reports aggregate counts, unique working-set sizes, and (by
default) the hit/miss rate of an idealized single-core cache model with
configurable size, line length, and associativity.  This makes it easy to
justify extremely low hit rates in workloads whose working sets dwarf a
32 KiB private cache.
"""

from __future__ import annotations

import argparse
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional


OPS = {"R", "r", "W", "w"}
DEFAULT_LINE_SIZE = 64
DEFAULT_PAGE_SIZE = 4096
DEFAULT_CACHE_SIZE = 32 * 1024
DEFAULT_ASSOCIATIVITY = 8


def non_negative_int(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{value!r} is not an integer") from exc
    if parsed < 0:
        raise argparse.ArgumentTypeError(f"{value!r} must be >= 0")
    return parsed


def fix_hex_token(token: str) -> str:
    if token.startswith("Ox"):
        return "0x" + token[2:]
    return token


@dataclass
class ParsedEvent:
    timestamp: int
    is_write: bool
    address: int


def parse_event(raw: str, path: Path, lineno: int) -> Optional[ParsedEvent]:
    stripped = raw.strip()
    if not stripped or stripped.startswith("#"):
        return None

    tokens = stripped.split()
    if len(tokens) < 3:
        raise ValueError(f"{path}:{lineno} malformed trace line: {raw!r}")

    try:
        timestamp = int(tokens[0], 0)
    except ValueError as exc:
        raise ValueError(f"{path}:{lineno} invalid timestamp {tokens[0]!r}") from exc

    second = tokens[1]
    op_token_idx: int
    addr_idx: int
    if second in OPS:
        op_token_idx = 1
        addr_idx = 2
    else:
        op_token_idx = 2
        addr_idx = 3
        if len(tokens) <= addr_idx:
            raise ValueError(f"{path}:{lineno} missing address field: {raw!r}")

    op_token = tokens[op_token_idx]
    if op_token not in OPS:
        raise ValueError(f"{path}:{lineno} unknown op {op_token!r} (expected R/W)")
    is_write = op_token in {"W", "w"}

    try:
        address = int(fix_hex_token(tokens[addr_idx]), 0)
    except ValueError as exc:
        raise ValueError(f"{path}:{lineno} invalid address {tokens[addr_idx]!r}") from exc

    return ParsedEvent(timestamp=timestamp, is_write=is_write, address=address)


class CacheModel:
    """Simple set-associative LRU cache used for locality estimation."""

    def __init__(self, cache_size: int, line_size: int, associativity: int) -> None:
        if line_size & (line_size - 1):
            raise ValueError("line_size must be a power of two")
        if cache_size % line_size != 0:
            raise ValueError("cache_size must be a multiple of line_size")
        if associativity <= 0:
            raise ValueError("associativity must be > 0")

        self.num_lines = cache_size // line_size
        if self.num_lines == 0:
            raise ValueError("cache must contain at least one line")
        if self.num_lines % associativity != 0:
            raise ValueError("number of lines must be divisible by associativity")

        self.line_mask = ~(line_size - 1)
        self.line_shift = (line_size.bit_length() - 1)
        self.associativity = associativity
        self.num_sets = self.num_lines // associativity
        self.sets = [OrderedDict() for _ in range(self.num_sets)]
        self.hits = 0
        self.misses = 0

    def access(self, addr: int) -> None:
        line_addr = addr & self.line_mask
        set_idx = ((line_addr >> self.line_shift) % self.num_sets)
        lru = self.sets[set_idx]
        if line_addr in lru:
            self.hits += 1
            lru.move_to_end(line_addr, last=False)
            return

        self.misses += 1
        if len(lru) >= self.associativity:
            lru.popitem(last=True)
        lru[line_addr] = None
        lru.move_to_end(line_addr, last=False)


@dataclass
class TraceStats:
    path: Path
    total_events: int = 0
    reads: int = 0
    writes: int = 0
    first_timestamp: Optional[int] = None
    last_timestamp: Optional[int] = None
    unique_addresses: int = 0
    unique_lines: int = 0
    unique_pages: int = 0
    cache_hits: Optional[int] = None
    cache_misses: Optional[int] = None


def analyze_trace(
    trace_path: Path,
    cache_model: Optional[CacheModel],
    limit: Optional[int],
) -> TraceStats:
    stats = TraceStats(path=trace_path)
    unique_addresses: set[int] = set()
    unique_lines: set[int] = set()
    unique_pages: set[int] = set()

    line_mask = ~(DEFAULT_LINE_SIZE - 1)
    page_mask = ~(DEFAULT_PAGE_SIZE - 1)

    with trace_path.open("r", encoding="ascii", errors="replace") as fh:
        for lineno, raw in enumerate(fh, 1):
            if limit is not None and stats.total_events >= limit:
                break

            event = parse_event(raw, trace_path, lineno)
            if event is None:
                continue

            stats.total_events += 1
            if stats.first_timestamp is None:
                stats.first_timestamp = event.timestamp
            stats.last_timestamp = event.timestamp

            if event.is_write:
                stats.writes += 1
            else:
                stats.reads += 1

            unique_addresses.add(event.address)
            unique_lines.add(event.address & line_mask)
            unique_pages.add(event.address & page_mask)

            if cache_model is not None:
                cache_model.access(event.address)

    stats.unique_addresses = len(unique_addresses)
    stats.unique_lines = len(unique_lines)
    stats.unique_pages = len(unique_pages)

    if cache_model is not None:
        stats.cache_hits = cache_model.hits
        stats.cache_misses = cache_model.misses

    return stats


def format_rate(part: int, whole: int) -> str:
    if whole == 0:
        return "n/a"
    return f"{100.0 * part / whole:.4f}%"


def print_stats(stats: TraceStats) -> None:
    print(f"Trace: {stats.path}")
    print(f"  Events      : {stats.total_events:,}")
    print(f"    Reads     : {stats.reads:,}")
    print(f"    Writes    : {stats.writes:,}")
    if stats.first_timestamp is not None and stats.last_timestamp is not None:
        span = stats.last_timestamp - stats.first_timestamp
        print(f"  Timestamp span: {span:,} cycles")
    print(f"  Unique addresses : {stats.unique_addresses:,}")
    if stats.total_events:
        ratio = stats.unique_lines / stats.total_events * 100.0
        print(
            f"  Unique {DEFAULT_LINE_SIZE}-byte lines : {stats.unique_lines:,} ({ratio:.4f}% of accesses)"
        )
    else:
        print(f"  Unique {DEFAULT_LINE_SIZE}-byte lines : {stats.unique_lines:,}")
    print(f"  Unique 4 KiB pages : {stats.unique_pages:,}")

    if stats.cache_hits is not None and stats.cache_misses is not None:
        accesses = stats.cache_hits + stats.cache_misses
        hit_rate = format_rate(stats.cache_hits, accesses)
        print(
            "  Cache model ({} KiB, line {}, {}-way)".format(
                DEFAULT_CACHE_SIZE // 1024,
                DEFAULT_LINE_SIZE,
                DEFAULT_ASSOCIATIVITY,
            )
        )
        print(f"    Hits   : {stats.cache_hits:,}")
        print(f"    Misses : {stats.cache_misses:,}")
        print(f"    Hit rate: {hit_rate}")
    print()


def summarize_overall(all_stats: list[TraceStats]) -> None:
    if len(all_stats) <= 1:
        return

    total_events = sum(s.total_events for s in all_stats)
    if total_events == 0:
        return
    reads = sum(s.reads for s in all_stats)
    writes = sum(s.writes for s in all_stats)
    cache_hits = sum(s.cache_hits or 0 for s in all_stats)
    cache_misses = sum(s.cache_misses or 0 for s in all_stats)
    accesses = cache_hits + cache_misses

    print("=== Aggregate across all traces ===")
    print(f"  Events : {total_events:,}")
    print(f"    Reads : {reads:,}")
    print(f"    Writes: {writes:,}")
    if accesses:
        print(f"  Cache-model hit rate: {format_rate(cache_hits, accesses)}")
    print()


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze timestamped memory traces used by the simulator."
    )
    parser.add_argument(
        "traces",
        nargs="+",
        type=Path,
        help="Path(s) to timestamped trace files.",
    )
    parser.add_argument(
        "--max-events",
        type=non_negative_int,
        help="Limit the number of events processed per trace (default: entire file).",
    )
    parser.add_argument(
        "--cache-model",
        action="store_true",
        help="Enable the cache-model simulation step (disabled by default).",
    )
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> None:
    args = parse_args(argv)

    cache_model_required = bool(args.cache_model)

    all_results: list[TraceStats] = []
    for trace_path in args.traces:
        if not trace_path.is_file():
            raise FileNotFoundError(f"Trace file {trace_path} does not exist")

        cache_model = None
        if cache_model_required:
            cache_model = CacheModel(
                DEFAULT_CACHE_SIZE, DEFAULT_LINE_SIZE, DEFAULT_ASSOCIATIVITY
            )

        stats = analyze_trace(
            trace_path=trace_path,
            cache_model=cache_model,
            limit=args.max_events,
        )
        all_results.append(stats)

    for stats in all_results:
        print_stats(stats)
    summarize_overall(all_results)


if __name__ == "__main__":
    main()
