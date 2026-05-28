#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import statistics
import sys
from typing import Callable

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

FrameDecoder = Callable[[bytes], int]


@dataclass(frozen=True)
class CandidateFieldDefinition:
    field_id: str
    label: str
    byte_positions: tuple[int, int]
    decoder: FrameDecoder


@dataclass(frozen=True)
class NumericStats:
    count: int
    minimum: float | None
    maximum: float | None
    mean: float | None


@dataclass(frozen=True)
class FrameStructureAnalysis:
    path: Path
    extracted_frame_count: int
    valid_checksum_count: int
    invalid_checksum_count: int
    candidate_field_stats: dict[str, NumericStats]
    unknown_byte_positions: tuple[int, ...]


def decode_int16_be(frame: bytes, first_byte: int) -> int:
    return int.from_bytes(frame[first_byte : first_byte + 2], "big", signed=True)


def decode_uint16_be(frame: bytes, first_byte: int) -> int:
    return int.from_bytes(frame[first_byte : first_byte + 2], "big", signed=False)


CANDIDATE_FIELDS: tuple[CandidateFieldDefinition, ...] = (
    CandidateFieldDefinition(
        "int16_be_02_03",
        "candidate_forward_motion",
        (2, 3),
        lambda frame: decode_int16_be(frame, 2),
    ),
    CandidateFieldDefinition(
        "int16_be_06_07",
        "candidate_yaw_motion",
        (6, 7),
        lambda frame: decode_int16_be(frame, 6),
    ),
    CandidateFieldDefinition(
        "int16_be_12_13",
        "candidate_imu_12_13",
        (12, 13),
        lambda frame: decode_int16_be(frame, 12),
    ),
    CandidateFieldDefinition(
        "int16_be_14_15",
        "candidate_imu_14_15",
        (14, 15),
        lambda frame: decode_int16_be(frame, 14),
    ),
    CandidateFieldDefinition(
        "int16_be_16_17",
        "candidate_imu_16_17",
        (16, 17),
        lambda frame: decode_int16_be(frame, 16),
    ),
    CandidateFieldDefinition(
        "int16_be_18_19",
        "candidate_imu_18_19",
        (18, 19),
        lambda frame: decode_int16_be(frame, 18),
    ),
    CandidateFieldDefinition(
        "uint16_be_20_21",
        "candidate_battery_mV",
        (20, 21),
        lambda frame: decode_uint16_be(frame, 20),
    ),
)

KNOWN_TEMPLATE_BYTES = {0, 22, 23}
KNOWN_CANDIDATE_BYTES = {
    position for field in CANDIDATE_FIELDS for position in field.byte_positions
}
UNKNOWN_PAYLOAD_BYTES = tuple(
    position
    for position in range(1, 22)
    if position not in KNOWN_CANDIDATE_BYTES and position not in KNOWN_TEMPLATE_BYTES
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyze stable C30D feedback frame structure from saved passive .bin files."
    )
    parser.add_argument("captures", nargs="+", type=Path, help="Saved passive C30D .bin captures.")
    return parser


def numeric_stats(values: list[int]) -> NumericStats:
    if not values:
        return NumericStats(count=0, minimum=None, maximum=None, mean=None)
    return NumericStats(
        count=len(values),
        minimum=float(min(values)),
        maximum=float(max(values)),
        mean=statistics.fmean(values),
    )


def analyze_frame_structure(path: Path) -> FrameStructureAnalysis:
    from ackermann_robot.drivers.c30d_frames import extract_frames, filter_frames_by_checksum

    frames = extract_frames(path.read_bytes())
    valid_frames = filter_frames_by_checksum(frames, require_valid=True)
    return FrameStructureAnalysis(
        path=path,
        extracted_frame_count=len(frames),
        valid_checksum_count=len(valid_frames),
        invalid_checksum_count=len(frames) - len(valid_frames),
        candidate_field_stats={
            field.field_id: numeric_stats([field.decoder(frame) for frame in valid_frames])
            for field in CANDIDATE_FIELDS
        },
        unknown_byte_positions=UNKNOWN_PAYLOAD_BYTES,
    )


def format_optional_number(value: float | None) -> str:
    if value is None:
        return "none"
    return f"{value:g}"


def format_stats(stats: NumericStats) -> str:
    return (
        f"count={stats.count} "
        f"min={format_optional_number(stats.minimum)} "
        f"max={format_optional_number(stats.maximum)} "
        f"mean={format_optional_number(stats.mean)}"
    )


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
    for field in CANDIDATE_FIELDS:
        stats = analysis.candidate_field_stats[field.field_id]
        print(f"  {field.field_id}: {field.label} {format_stats(stats)}")
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
