#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

DEFAULT_OUTPUT_DIR = Path("data/slam_tests/split_scans")
CSV_COLUMNS = ["timestamp_s", "angle_deg", "distance_mm", "quality"]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Split a recorded RPLIDAR CSV into individual angle-wrapped scans."
    )
    parser.add_argument(
        "input_csv", type=Path, help="Recorded RPLIDAR CSV from rplidar_scan_sample.py."
    )
    parser.add_argument(
        "--wrap-high-deg",
        type=float,
        default=300.0,
        help="Previous angle threshold for detecting revolution wraparound.",
    )
    parser.add_argument(
        "--wrap-low-deg",
        type=float,
        default=60.0,
        help="Current angle threshold for detecting revolution wraparound.",
    )
    parser.add_argument(
        "--save-scans",
        action="store_true",
        help="Save each split scan as a separate CSV.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Directory for split scan CSVs when --save-scans is set.",
    )
    return parser


def scan_summary_rows(sequence) -> list[dict[str, float | int]]:
    rows: list[dict[str, float | int]] = []
    for index, scan in enumerate(sequence.scans):
        rows.append(
            {
                "scan_index": index,
                "point_count": len(scan.points),
                "duration_s": scan.end_time_s - scan.start_time_s,
            }
        )
    return rows


def save_split_scans(sequence, output_dir: Path, input_stem: str) -> list[Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    paths: list[Path] = []
    for index, scan in enumerate(sequence.scans):
        output_path = output_dir / f"{input_stem}_scan_{index:03d}.csv"
        with output_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS)
            writer.writeheader()
            for point in scan.points:
                writer.writerow(
                    {
                        "timestamp_s": f"{point.timestamp_s:.6f}",
                        "angle_deg": f"{point.angle_deg:.3f}",
                        "distance_mm": f"{point.distance_mm:.3f}",
                        "quality": "" if point.quality is None else str(point.quality),
                    }
                )
        paths.append(output_path)
    return paths


def main(argv: list[str] | None = None) -> int:
    from ackermann_robot.slam.lidar_loader import load_lidar_scan_sequence

    args = build_parser().parse_args(argv)
    try:
        sequence = load_lidar_scan_sequence(
            args.input_csv,
            wrap_high_deg=args.wrap_high_deg,
            wrap_low_deg=args.wrap_low_deg,
        )
    except Exception as exc:
        print(f"Failed to split lidar scans: {exc!r}", file=sys.stderr)
        return 1

    print(f"scan_count: {len(sequence.scans)}")
    for row in scan_summary_rows(sequence):
        print(
            f"scan {row['scan_index']}: points={row['point_count']} "
            f"duration_s={row['duration_s']:.6f}"
        )

    if args.save_scans:
        paths = save_split_scans(sequence, args.output_dir, args.input_csv.stem)
        print(f"saved_scans: {len(paths)}")
        print(f"output_dir: {args.output_dir}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
