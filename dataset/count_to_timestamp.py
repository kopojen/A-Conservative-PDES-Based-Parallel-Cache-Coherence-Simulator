#!/usr/bin/env python3
"""
Convert the leading count column in USIMM traces into per-thread timestamps.

Each line in a trace has the format:
    <count> <op> <addr> [pc]
where `count` is the number of consecutive non-memory instructions that
completed since the previous memory instruction.  We treat every non-memory
instruction as taking `non_mem_latency` cycles, and every memory instruction as
occupying `mem_latency` cycles in the local timeline.  The resulting timestamp
is:

    timestamp = current_time + count * non_mem_latency

After emitting the timestamped event we advance `current_time` by the memory
latency to prepare for the next instruction from the same trace.
"""
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable


def positive_int(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{value!r} is not an integer") from exc
    if parsed < 0:
        raise argparse.ArgumentTypeError(f"{value!r} must be non-negative")
    return parsed


def process_trace(
    src_path: Path,
    dst_path: Path,
    non_mem_latency: int,
    mem_latency: int,
) -> None:
    current_time = 0

    with src_path.open("r", encoding="ascii") as src, dst_path.open(
        "w", encoding="ascii"
    ) as dst:
        for lineno, raw in enumerate(src, 1):
            line = raw.strip()
            if not line:
                continue

            parts = line.split()
            if len(parts) < 3:
                raise ValueError(
                    f"{src_path}:{lineno} malformed trace line: {raw!r}"
                )

            count_str, op = parts[0], parts[1]
            if op not in {"R", "W"}:
                raise ValueError(
                    f"{src_path}:{lineno} unknown op {op!r} (expected R/W)"
                )

            try:
                non_mem_count = int(count_str)
            except ValueError as exc:
                raise ValueError(
                    f"{src_path}:{lineno} invalid count {count_str!r}"
                ) from exc
            if non_mem_count < 0:
                raise ValueError(
                    f"{src_path}:{lineno} negative count {non_mem_count}"
                )

            timestamp = current_time + non_mem_count * non_mem_latency
            current_time = timestamp + mem_latency

            rest = " ".join(parts[2:])
            dst.write(f"{timestamp} {op} {rest}\n")


def resolve_output_path(src: Path, output_dir: Path) -> Path:
    suffix = src.suffix
    stem = src.stem
    target_name = f"{stem}.ts{suffix}" if suffix else f"{stem}.ts"
    return output_dir / target_name


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert USIMM trace counts into timestamps."
    )
    parser.add_argument(
        "traces",
        nargs="+",
        type=Path,
        help="Path(s) to USIMM trace files.",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        default=Path("timestamped_traces"),
        help="Directory to store timestamped traces (default: %(default)s).",
    )
    parser.add_argument(
        "--non-mem-latency",
        type=positive_int,
        default=1,
        help="Latency (cycles) per non-memory instruction (default: %(default)s).",
    )
    parser.add_argument(
        "--mem-latency",
        type=positive_int,
        default=1,
        help="Latency (cycles) per memory instruction (default: %(default)s).",
    )
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> None:
    args = parse_args(argv)
    output_dir: Path = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    for trace_path in args.traces:
        if not trace_path.is_file():
            raise FileNotFoundError(f"Trace file {trace_path} does not exist")
        output_path = resolve_output_path(trace_path, output_dir)
        process_trace(
            trace_path,
            output_path,
            args.non_mem_latency,
            args.mem_latency,
        )
        print(f"Wrote {output_path}")


if __name__ == "__main__":
    main()
