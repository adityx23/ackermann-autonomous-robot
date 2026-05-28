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

BYTE_20_INDEX = 20
BYTE_21_INDEX = 21


@dataclass(frozen=True)
class NumericStats:
    count: int
    minimum: float | None
    maximum: float | None
    mean: float | None


@dataclass(frozen=True)
class PayloadFieldAnalysis:
    path: Path
    extracted_frame_count: int
    valid_checksum_count: int
    invalid_checksum_count: int
    uint16_be_20_21: NumericStats
    candidate_battery_voltage_V: NumericStats
    byte_20_counts: dict[int, int]
    byte_21_counts: dict[int, int]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyze C30D feedback payload bytes 20-21 from saved passive .bin captures."
    )
    parser.add_argument("captures", nargs="+", type=Path, help="Saved passive C30D .bin captures.")
    return parser


def numeric_stats(values: list[float]) -> NumericStats:
    if not values:
        return NumericStats(count=0, minimum=None, maximum=None, mean=None)
    return NumericStats(
        count=len(values),
        minimum=min(values),
        maximum=max(values),
        mean=statistics.fmean(values),
    )


def decode_uint16_be_20_21(frame: bytes) -> int:
    return int.from_bytes(frame[BYTE_20_INDEX : BYTE_21_INDEX + 1], "big", signed=False)


def analyze_payload_fields(path: Path) -> PayloadFieldAnalysis:
    from ackermann_robot.drivers.c30d_frames import extract_frames, filter_frames_by_checksum

    frames = extract_frames(path.read_bytes())
    valid_frames = filter_frames_by_checksum(frames, require_valid=True)
    uint16_values = [decode_uint16_be_20_21(frame) for frame in valid_frames]
    voltage_values = [value / 1000.0 for value in uint16_values]
    byte_20_counts = dict(sorted(Counter(frame[BYTE_20_INDEX] for frame in valid_frames).items()))
    byte_21_counts = dict(sorted(Counter(frame[BYTE_21_INDEX] for frame in valid_frames).items()))

    return PayloadFieldAnalysis(
        path=path,
        extracted_frame_count=len(frames),
        valid_checksum_count=len(valid_frames),
        invalid_checksum_count=len(frames) - len(valid_frames),
        uint16_be_20_21=numeric_stats([float(value) for value in uint16_values]),
        candidate_battery_voltage_V=numeric_stats(voltage_values),
        byte_20_counts=byte_20_counts,
        byte_21_counts=byte_21_counts,
    )


def format_optional_float(value: float | None, decimals: int = 3) -> str:
    if value is None:
        return "none"
    return f"{value:.{decimals}f}"


def format_stats(stats: NumericStats, decimals: int = 3) -> str:
    return (
        f"count={stats.count} "
        f"min={format_optional_float(stats.minimum, decimals)} "
        f"max={format_optional_float(stats.maximum, decimals)} "
        f"mean={format_optional_float(stats.mean, decimals)}"
    )


def format_counts(counts: dict[int, int]) -> str:
    if not counts:
        return "none"
    return ", ".join(f"0x{value:02x}:{count}" for value, count in counts.items())


def print_payload_analysis(analysis: PayloadFieldAnalysis) -> None:
    print(f"file: {analysis.path}")
    print(f"extracted_frame_count: {analysis.extracted_frame_count}")
    print(f"valid_checksum_count: {analysis.valid_checksum_count}")
    print(f"invalid_checksum_count: {analysis.invalid_checksum_count}")
    print(f"uint16_be_20_21: {format_stats(analysis.uint16_be_20_21, decimals=0)}")
    print(f"byte_20_unique_counts: {format_counts(analysis.byte_20_counts)}")
    print(f"byte_21_unique_counts: {format_counts(analysis.byte_21_counts)}")
    print(
        "candidate_battery_voltage_V: "
        f"{format_stats(analysis.candidate_battery_voltage_V, decimals=3)}"
    )
    print("interpretation: candidate_only_not_confirmed_battery_voltage")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    print("Read-only C30D payload field analysis from saved captures only.")
    try:
        analyses = [analyze_payload_fields(path) for path in args.captures]
    except OSError as exc:
        print(f"failed to read capture: {exc}", file=sys.stderr)
        return 1

    for index, analysis in enumerate(analyses):
        if index:
            print()
        print_payload_analysis(analysis)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
