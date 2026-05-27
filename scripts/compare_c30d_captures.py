#!/usr/bin/env python3

import argparse
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

PAYLOAD_START = 1
PAYLOAD_END_EXCLUSIVE = 23
DEFAULT_TOP_LIMIT = 8


@dataclass(frozen=True)
class NumericStats:
    minimum: float
    maximum: float
    mean: float
    stdev: float
    unique_count: int


@dataclass(frozen=True)
class CaptureAnalysis:
    path: Path
    total_bytes: int
    frame_count: int
    rejected_resync_count: int
    partial_frame_count: int
    byte_stats: dict[int, NumericStats]
    candidate_int16_stats: dict[str, NumericStats]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compare fixed-length C30D frames from saved passive .bin captures."
    )
    parser.add_argument("captures", nargs="+", type=Path, help="Saved passive .bin capture files.")
    parser.add_argument(
        "--baseline-index",
        type=int,
        default=0,
        help="Zero-based capture index to use as the comparison baseline.",
    )
    parser.add_argument(
        "--top",
        type=int,
        default=DEFAULT_TOP_LIMIT,
        help="Number of top changing byte positions and candidate fields to print per capture.",
    )
    return parser


def numeric_stats(values: list[int]) -> NumericStats:
    if not values:
        return NumericStats(
            minimum=math.nan,
            maximum=math.nan,
            mean=math.nan,
            stdev=math.nan,
            unique_count=0,
        )
    stdev = statistics.pstdev(values) if len(values) > 1 else 0.0
    return NumericStats(
        minimum=min(values),
        maximum=max(values),
        mean=statistics.fmean(values),
        stdev=stdev,
        unique_count=len(set(values)),
    )


def byte_position_stats(frames: list[bytes]) -> dict[int, NumericStats]:
    if not frames:
        return {}
    frame_length = len(frames[0])
    return {
        position: numeric_stats([frame[position] for frame in frames])
        for position in range(frame_length)
    }


def candidate_int16_values(frames: list[bytes], first_position: int, endian: str) -> list[int]:
    return [
        int.from_bytes(frame[first_position : first_position + 2], endian, signed=True)
        for frame in frames
    ]


def candidate_int16_stats(frames: list[bytes]) -> dict[str, NumericStats]:
    stats: dict[str, NumericStats] = {}
    if not frames:
        return stats

    for first_position in range(PAYLOAD_START, PAYLOAD_END_EXCLUSIVE - 1):
        second_position = first_position + 1
        pair = f"{first_position:02d}_{second_position:02d}"
        stats[f"candidate_int16_be_{pair}"] = numeric_stats(
            candidate_int16_values(frames, first_position, "big")
        )
        stats[f"candidate_int16_le_{pair}"] = numeric_stats(
            candidate_int16_values(frames, first_position, "little")
        )
    return stats


def analyze_capture(path: Path) -> CaptureAnalysis:
    from ackermann_robot.drivers.c30d_frames import extract_frames_with_stats

    data = path.read_bytes()
    extraction = extract_frames_with_stats(data)
    frames = extraction.frames
    return CaptureAnalysis(
        path=path,
        total_bytes=len(data),
        frame_count=len(frames),
        rejected_resync_count=extraction.rejected_resync_count,
        partial_frame_count=extraction.partial_frame_count,
        byte_stats=byte_position_stats(frames),
        candidate_int16_stats=candidate_int16_stats(frames),
    )


def stdev_delta(stats: NumericStats, baseline: NumericStats | None) -> float:
    if baseline is None:
        return stats.stdev
    return stats.stdev - baseline.stdev


def changes_much_more(stats: NumericStats, baseline: NumericStats | None) -> bool:
    if baseline is None or stats.unique_count <= baseline.unique_count:
        return False
    if baseline.stdev == 0.0:
        return stats.stdev > 0.0
    return stats.stdev >= baseline.stdev * 3.0 and stats.stdev - baseline.stdev >= 1.0


def top_by_stdev_delta(
    stats_by_name: dict[int, NumericStats] | dict[str, NumericStats],
    baseline_by_name: dict[int, NumericStats] | dict[str, NumericStats],
    limit: int,
) -> list[tuple[int | str, NumericStats, float]]:
    rows: list[tuple[int | str, NumericStats, float]] = []
    for name, stats in stats_by_name.items():
        baseline = baseline_by_name.get(name)  # type: ignore[arg-type]
        delta = stdev_delta(stats, baseline)
        rows.append((name, stats, delta))
    return sorted(rows, key=lambda row: (row[2], row[1].stdev, row[1].unique_count), reverse=True)[
        :limit
    ]


def highlighted_byte_positions(
    analysis: CaptureAnalysis, baseline: CaptureAnalysis
) -> list[tuple[int, NumericStats, NumericStats, float]]:
    highlighted: list[tuple[int, NumericStats, NumericStats, float]] = []
    for position, stats in analysis.byte_stats.items():
        baseline_stats = baseline.byte_stats.get(position)
        if baseline_stats is None:
            continue
        if changes_much_more(stats, baseline_stats):
            highlighted.append(
                (position, stats, baseline_stats, stats.stdev - baseline_stats.stdev)
            )
    return sorted(highlighted, key=lambda row: row[3], reverse=True)


def format_stats(stats: NumericStats) -> str:
    return (
        f"min={stats.minimum:.0f} max={stats.maximum:.0f} mean={stats.mean:.2f} "
        f"stdev={stats.stdev:.2f} unique={stats.unique_count}"
    )


def print_frame_counts(analyses: list[CaptureAnalysis]) -> None:
    print("Frame Counts Per File")
    for index, analysis in enumerate(analyses):
        print(
            f"[{index}] {analysis.path}: total_bytes={analysis.total_bytes} "
            f"valid_frames={analysis.frame_count} "
            f"rejected_resync={analysis.rejected_resync_count} "
            f"partial_frames={analysis.partial_frame_count}"
        )


def print_byte_level_comparison(analyses: list[CaptureAnalysis], baseline: CaptureAnalysis) -> None:
    print()
    print(f"Byte-Level Comparison Against Baseline: {baseline.path}")
    for analysis in analyses:
        print(f"{analysis.path}:")
        if not analysis.byte_stats:
            print("  no valid frames")
            continue
        for position, stats in analysis.byte_stats.items():
            baseline_stats = baseline.byte_stats.get(position)
            delta = stdev_delta(stats, baseline_stats)
            highlight = "yes" if changes_much_more(stats, baseline_stats) else "no"
            print(
                f"  byte_{position:02d}: {format_stats(stats)} "
                f"stdev_delta={delta:.2f} highlight={highlight}"
            )


def print_top_byte_positions(
    analyses: list[CaptureAnalysis], baseline: CaptureAnalysis, limit: int
) -> None:
    print()
    print("Top Changing Byte Positions Per Capture")
    for analysis in analyses:
        print(f"{analysis.path}:")
        rows = top_by_stdev_delta(analysis.byte_stats, baseline.byte_stats, limit)
        if not rows:
            print("  none")
            continue
        for position, stats, delta in rows:
            print(f"  byte_{position:02d}: {format_stats(stats)} stdev_delta={delta:.2f}")


def print_top_candidate_fields(
    analyses: list[CaptureAnalysis], baseline: CaptureAnalysis, limit: int
) -> None:
    print()
    print("Top Changing Candidate Int16 Fields Per Capture")
    for analysis in analyses:
        print(f"{analysis.path}:")
        rows = top_by_stdev_delta(
            analysis.candidate_int16_stats, baseline.candidate_int16_stats, limit
        )
        if not rows:
            print("  none")
            continue
        for name, stats, delta in rows:
            print(f"  {name}: {format_stats(stats)} stdev_delta={delta:.2f}")


def print_comparison(analyses: list[CaptureAnalysis], baseline_index: int, top_limit: int) -> None:
    baseline = analyses[baseline_index]
    print("Read-only C30D comparative analysis from saved captures only.")
    print_frame_counts(analyses)
    print_byte_level_comparison(analyses, baseline)
    print_top_byte_positions(analyses, baseline, top_limit)
    print_top_candidate_fields(analyses, baseline, top_limit)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.baseline_index < 0 or args.baseline_index >= len(args.captures):
        print("--baseline-index must refer to one of the input captures", file=sys.stderr)
        return 2
    if args.top < 1:
        print("--top must be positive", file=sys.stderr)
        return 2

    try:
        analyses = [analyze_capture(path) for path in args.captures]
    except OSError as exc:
        print(f"failed to read capture: {exc}", file=sys.stderr)
        return 1

    print_comparison(analyses, args.baseline_index, args.top)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
