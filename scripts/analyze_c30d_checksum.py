#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

DEFAULT_TOP_LIMIT = 8


@dataclass(frozen=True)
class CaptureChecksumAnalysis:
    path: Path
    frame_count: int
    top_results: list


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyze read-only C30D checksum hypotheses from saved .bin captures."
    )
    parser.add_argument("captures", nargs="+", type=Path, help="Saved passive C30D .bin captures.")
    parser.add_argument("--top", type=int, default=DEFAULT_TOP_LIMIT, help="Top hypotheses to print.")
    return parser


def analyze_capture(path: Path) -> CaptureChecksumAnalysis:
    from ackermann_robot.drivers.c30d_checksum import test_checksum_hypotheses
    from ackermann_robot.drivers.c30d_frames import extract_frames

    frames = extract_frames(path.read_bytes())
    return CaptureChecksumAnalysis(
        path=path,
        frame_count=len(frames),
        top_results=test_checksum_hypotheses(frames),
    )


def format_percentage(value: float) -> str:
    return f"{value:.2f}%"


def print_capture_analysis(analysis: CaptureChecksumAnalysis, top_limit: int) -> None:
    print(f"file: {analysis.path}")
    print(f"frame_count: {analysis.frame_count}")
    if not analysis.top_results:
        print("top_checksum_hypotheses: none")
        print("any_hypothesis_100_percent: false")
        return

    print("top_checksum_hypotheses:")
    for result in analysis.top_results[:top_limit]:
        print(
            f"  {result.name} {result.byte_range.label}: "
            f"matches={result.match_count}/{result.frame_count} "
            f"match_percentage={format_percentage(result.match_percentage)} "
            f"range={result.byte_range.start}:{result.byte_range.end_exclusive} "
            f"notes={result.byte_range.notes}"
        )

    any_100 = any(result.reached_100_percent for result in analysis.top_results)
    print(f"any_hypothesis_100_percent: {str(any_100).lower()}")


def hypotheses_100_percent_in_all_captures(
    analyses: list[CaptureChecksumAnalysis],
) -> set[tuple[str, str]]:
    if len(analyses) < 2:
        return set()

    common: set[tuple[str, str]] | None = None
    for analysis in analyses:
        reached = {
            (result.name, result.byte_range.label)
            for result in analysis.top_results
            if result.reached_100_percent
        }
        common = reached if common is None else common & reached
    return common or set()


def print_cross_capture_summary(analyses: list[CaptureChecksumAnalysis]) -> None:
    confirmed_candidates = hypotheses_100_percent_in_all_captures(analyses)
    if len(analyses) < 2:
        print("confirmed_across_multiple_captures: no_single_capture_only")
        return
    if not confirmed_candidates:
        print("confirmed_across_multiple_captures: no")
        return

    print("confirmed_across_multiple_captures: candidate_requires_review")
    for name, range_label in sorted(confirmed_candidates):
        print(f"  {name} {range_label}")
    print("note: 100% across captures is evidence for review, not protocol confirmation by itself")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.top < 1:
        print("--top must be positive", file=sys.stderr)
        return 2

    try:
        analyses = [analyze_capture(path) for path in args.captures]
    except OSError as exc:
        print(f"failed to read capture: {exc}", file=sys.stderr)
        return 1

    print("Read-only C30D checksum hypothesis analysis from saved captures only.")
    print("checksum_candidate_index: 22")
    for index, analysis in enumerate(analyses):
        if index:
            print()
        print_capture_analysis(analysis, args.top)
    print()
    print_cross_capture_summary(analyses)
    print("real_motor_command_path: disabled")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
