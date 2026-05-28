#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import asdict, fields
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

DEFAULT_OUTPUT_DIR = Path("data/c30d_analysis")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Export read-only C30D candidate feedback fields from a saved .bin capture."
    )
    parser.add_argument("capture", type=Path, help="Path to a saved passive .bin capture file.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Directory for the exported CSV file.",
    )
    return parser


def output_path_for(capture_path: Path, output_dir: Path) -> Path:
    return output_dir / f"{capture_path.stem}_feedback_candidates.csv"


def export_feedback_candidates(capture_path: Path, output_dir: Path) -> tuple[Path, int]:
    from ackermann_robot.drivers.c30d_feedback import (
        C30DFeedbackCandidate,
        parse_feedback_candidates,
    )
    from ackermann_robot.drivers.c30d_frames import extract_frames

    data = capture_path.read_bytes()
    frames = extract_frames(data)
    candidates = parse_feedback_candidates(frames)

    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_path_for(capture_path, output_dir)
    field_names = [field.name for field in fields(C30DFeedbackCandidate)]
    with output_path.open("w", newline="") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=field_names)
        writer.writeheader()
        writer.writerows(asdict(candidate) for candidate in candidates)

    return output_path, len(candidates)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        output_path, row_count = export_feedback_candidates(args.capture, args.output_dir)
    except OSError as exc:
        print(f"failed to read or write C30D feedback candidate CSV: {exc}", file=sys.stderr)
        return 1
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    print("Read-only C30D candidate feedback export from saved capture only.")
    print(f"output_path: {output_path}")
    print(f"row_count: {row_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
