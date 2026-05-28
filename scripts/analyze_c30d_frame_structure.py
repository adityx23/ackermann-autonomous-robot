#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
import statistics
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))


@dataclass(frozen=True)
class FrameStructureAnalysis:
    path: Path
    extracted_frame_count: int
    valid_checksum_count: int
    invalid_checksum_count: int
    candidate_battery_mv_values: list[int]
    unknown_byte_positions: tuple[int, ...]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyze stable C30D feedback frame structure from saved passive .bin files."
    )
    parser.add_argument("captures", nargs="+", type=Path, help="Saved passive C30D .bin captures.")
    return parser


def decode_candidate_battery_mv(frame: bytes) -> int:
    return int.from_bytes(frame[20:22], "big", signed=False)


def analyze_frame_structure(path: Path) -> FrameStructureAnalysis:
    from ackermann_robot.drivers.c30d_frames import extract_frames, filter_frames_by_checksum

    frames = extract_frames(path.read_bytes())
    valid_frames = filter_frames_by_checksum(frames, require_valid=True)
    return FrameStructureAnalysis(
        path=path,
        extracted_frame_count=len(frames),
        valid_checksum_count=len(valid_frames),
        invalid_checksum_count=len(frames) - len(valid_frames),
        candidate_battery_mv_values=[decode_candidate_battery_mv(frame) for frame in valid_frames],
        unknown_byte_positions=tuple(range(1, 20)) + (21,),
    )


def format_mv_stats(values: list[int]) -> str:
    if not values:
        return "count=0 min=none max=none mean=none"
    return (
        f"count={len(values)} "
        f"min={min(values)} "
        f"max={max(values)} "
        f"mean={statistics.fmean(values):.0f}"
    )


def format_top_values(values: list[int], limit: int = 8) -> str:
    if not values:
        return "none"
    counts = Counter(values)
    return ", ".join(f"{value}:{count}" for value, count in counts.most_common(limit))


def print_analysis(analysis: FrameStructureAnalysis) -> None:
    print(f"file: {analysis.path}")
    print(f"extracted_frame_count: {analysis.extracted_frame_count}")
    print(f"valid_checksum_count: {analysis.valid_checksum_count}")
    print(f"invalid_checksum_count: {analysis.invalid_checksum_count}")
    print("stable_frame_template:")
    print("  byte_0_start: 0x7b")
    print("  bytes_1_to_21: payload_or_status_unknown")
    print("  byte_22_checksum: xor_bytes_0_through_21")
    print("  byte_23_end: 0x7d")
    print("candidate_fields:")
    print("  uint16_be_20_21: candidate_battery_mV")
    print(f"  candidate_battery_mV_stats: {format_mv_stats(analysis.candidate_battery_mv_values)}")
    print(
        f"  candidate_battery_mV_top_values: {format_top_values(analysis.candidate_battery_mv_values)}"
    )
    unknown = ", ".join(str(position) for position in analysis.unknown_byte_positions)
    print(f"unknown_bytes: {unknown}")
    print("interpretation: feedback_structure_only_command_protocol_unknown")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    print("Read-only C30D feedback frame structure analysis from saved captures only.")
    try:
        analyses = [analyze_frame_structure(path) for path in args.captures]
    except OSError as exc:
        print(f"failed to read capture: {exc}", file=sys.stderr)
        return 1

    for index, analysis in enumerate(analyses):
        if index:
            print()
        print_analysis(analysis)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
