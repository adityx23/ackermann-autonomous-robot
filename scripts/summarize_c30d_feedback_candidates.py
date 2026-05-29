#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

SUMMARY_FIELDS = [
    "candidate_forward_motion",
    "candidate_yaw_motion",
    "candidate_imu_12_13",
    "candidate_imu_14_15",
    "candidate_imu_16_17",
    "candidate_imu_18_19",
]


@dataclass(frozen=True)
class CandidateStats:
    minimum: int | float
    maximum: int | float
    mean: float
    stdev: float
    total: int
    sum_abs: int
    count_nonzero: int


@dataclass(frozen=True)
class FeedbackCandidateSummary:
    csv_path: Path
    row_count: int
    stats_by_field: dict[str, CandidateStats]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Summarize read-only C30D candidate feedback CSV exports."
    )
    parser.add_argument(
        "csv_files",
        nargs="+",
        type=Path,
        help="Feedback candidate CSV files exported from saved passive captures.",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        help="Known capture duration in seconds; used to estimate sample rate.",
    )
    parser.add_argument(
        "--known-distance-m",
        type=float,
        help="Known traveled distance in meters; used with candidate_forward_motion sum.",
    )
    parser.add_argument(
        "--known-yaw-deg",
        type=float,
        help="Known yaw rotation in degrees; used with candidate_yaw_motion sum.",
    )
    return parser


def numeric_stats(values: list[int]) -> CandidateStats:
    if not values:
        return CandidateStats(
            minimum=math.nan,
            maximum=math.nan,
            mean=math.nan,
            stdev=math.nan,
            total=0,
            sum_abs=0,
            count_nonzero=0,
        )
    return CandidateStats(
        minimum=min(values),
        maximum=max(values),
        mean=statistics.fmean(values),
        stdev=statistics.pstdev(values) if len(values) > 1 else 0.0,
        total=sum(values),
        sum_abs=sum(abs(value) for value in values),
        count_nonzero=sum(1 for value in values if value != 0),
    )


def read_candidate_values(csv_path: Path) -> tuple[int, dict[str, list[int]]]:
    values_by_field: dict[str, list[int]] = {field: [] for field in SUMMARY_FIELDS}
    with csv_path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            raise ValueError(f"{csv_path} is missing a CSV header")
        missing_fields = [field for field in SUMMARY_FIELDS if field not in reader.fieldnames]
        if missing_fields:
            missing_text = ", ".join(missing_fields)
            raise ValueError(f"{csv_path} is missing required columns: {missing_text}")

        row_count = 0
        for row_number, row in enumerate(reader, start=2):
            row_count += 1
            for field in SUMMARY_FIELDS:
                try:
                    values_by_field[field].append(int(row[field]))
                except (TypeError, ValueError) as exc:
                    raise ValueError(
                        f"{csv_path}:{row_number} has invalid integer for {field}: "
                        f"{row[field]!r}"
                    ) from exc
    return row_count, values_by_field


def summarize_csv(csv_path: Path) -> FeedbackCandidateSummary:
    row_count, values_by_field = read_candidate_values(csv_path)
    return FeedbackCandidateSummary(
        csv_path=csv_path,
        row_count=row_count,
        stats_by_field={field: numeric_stats(values) for field, values in values_by_field.items()},
    )


def ratio_or_nan(numerator: float, denominator: int) -> float:
    if denominator == 0:
        return math.nan
    return numerator / denominator


def format_number(value: int | float) -> str:
    if isinstance(value, int):
        return str(value)
    if math.isnan(value):
        return "nan"
    return f"{value:.6g}"


def print_summary(
    summary: FeedbackCandidateSummary,
    duration_s: float | None,
    known_distance_m: float | None,
    known_yaw_deg: float | None,
) -> None:
    print(f"csv_path: {summary.csv_path}")
    print(f"row_count: {summary.row_count}")
    if duration_s is not None:
        print(f"sample_rate_hz: {summary.row_count / duration_s:.6g}")

    for field in SUMMARY_FIELDS:
        stats = summary.stats_by_field[field]
        print(
            f"{field}: min={format_number(stats.minimum)} max={format_number(stats.maximum)} "
            f"mean={format_number(stats.mean)} stdev={format_number(stats.stdev)} "
            f"sum={stats.total} sum_abs={stats.sum_abs} count_nonzero={stats.count_nonzero}"
        )

    if known_distance_m is not None:
        forward_sum = summary.stats_by_field["candidate_forward_motion"].total
        meters_per_forward_sum = ratio_or_nan(known_distance_m, forward_sum)
        print(f"meters_per_forward_sum: {format_number(meters_per_forward_sum)}")
    if known_yaw_deg is not None:
        yaw_sum = summary.stats_by_field["candidate_yaw_motion"].total
        radians_per_yaw_sum = ratio_or_nan(math.radians(known_yaw_deg), yaw_sum)
        print(f"radians_per_yaw_sum: {format_number(radians_per_yaw_sum)}")


def validate_args(args: argparse.Namespace) -> None:
    if args.duration_s is not None and args.duration_s <= 0.0:
        raise ValueError("--duration-s must be greater than zero")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        validate_args(args)
        summaries = [summarize_csv(csv_path) for csv_path in args.csv_files]
    except (OSError, ValueError) as exc:
        print(f"failed to summarize C30D feedback candidate CSV: {exc}", file=sys.stderr)
        return 1

    print("Read-only C30D candidate feedback CSV summary.")
    for index, summary in enumerate(summaries):
        if index:
            print()
        print_summary(summary, args.duration_s, args.known_distance_m, args.known_yaw_deg)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
