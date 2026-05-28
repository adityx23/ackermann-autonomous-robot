#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

STATUS_BYTE_INDEX = 21


@dataclass(frozen=True)
class StatusByteAnalysis:
    path: Path
    frame_count: int
    invalid_checksum_count: int
    minimum: int | None
    maximum: int | None
    unique_values: tuple[int, ...]
    transitions: int
    counts: dict[int, int]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyze C30D feedback byte 21 from saved passive .bin captures."
    )
    parser.add_argument("captures", nargs="+", type=Path, help="Saved passive C30D .bin captures.")
    return parser


def analyze_status_byte(path: Path) -> StatusByteAnalysis:
    from ackermann_robot.drivers.c30d_frames import extract_frames, filter_frames_by_checksum

    frames = extract_frames(path.read_bytes())
    valid_frames = filter_frames_by_checksum(frames, require_valid=True)
    invalid_checksum_count = len(frames) - len(valid_frames)
    values = [frame[STATUS_BYTE_INDEX] for frame in valid_frames if len(frame) > STATUS_BYTE_INDEX]
    counts = dict(sorted(Counter(values).items()))
    transitions = sum(1 for previous, current in zip(values, values[1:]) if previous != current)
    return StatusByteAnalysis(
        path=path,
        frame_count=len(valid_frames),
        invalid_checksum_count=invalid_checksum_count,
        minimum=min(values) if values else None,
        maximum=max(values) if values else None,
        unique_values=tuple(sorted(counts)),
        transitions=transitions,
        counts=counts,
    )


def format_values(values: tuple[int, ...]) -> str:
    if not values:
        return "none"
    return ", ".join(f"0x{value:02x}" for value in values)


def format_counts(counts: dict[int, int]) -> str:
    if not counts:
        return "none"
    return ", ".join(f"0x{value:02x}:{count}" for value, count in counts.items())


def print_status_analysis(analysis: StatusByteAnalysis) -> None:
    print(f"file: {analysis.path}")
    print(f"valid_frame_count: {analysis.frame_count}")
    print(f"invalid_checksum_count: {analysis.invalid_checksum_count}")
    print(f"byte_index: {STATUS_BYTE_INDEX}")
    print(f"min: {'none' if analysis.minimum is None else f'0x{analysis.minimum:02x}'}")
    print(f"max: {'none' if analysis.maximum is None else f'0x{analysis.maximum:02x}'}")
    print(f"unique_values: {format_values(analysis.unique_values)}")
    print(f"transitions: {analysis.transitions}")
    print(f"counts: {format_counts(analysis.counts)}")
    print("meaning: unassigned_status_counter_or_mode_candidate")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    print("Read-only C30D byte 21 analysis from saved captures only.")
    try:
        analyses = [analyze_status_byte(path) for path in args.captures]
    except OSError as exc:
        print(f"failed to read capture: {exc}", file=sys.stderr)
        return 1

    for index, analysis in enumerate(analyses):
        if index:
            print()
        print_status_analysis(analysis)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
