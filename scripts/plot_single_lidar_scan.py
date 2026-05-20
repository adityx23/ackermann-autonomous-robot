#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

DEFAULT_OUTPUT_DIR = Path("data/slam_tests")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plot one segmented scan from a recorded RPLIDAR CSV."
    )
    parser.add_argument(
        "input_csv", type=Path, help="Recorded RPLIDAR CSV from rplidar_scan_sample.py."
    )
    parser.add_argument(
        "--scan-index", type=int, default=0, help="Zero-based segmented scan index."
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
    parser.add_argument("--output", type=Path, default=None, help="Output PNG path.")
    return parser


def default_output_path(scan_index: int, now: datetime | None = None) -> Path:
    timestamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
    return DEFAULT_OUTPUT_DIR / f"lidar_scan_{scan_index:03d}_{timestamp}.png"


def save_single_scan_plot(scan, output_path: Path) -> int:
    import matplotlib.pyplot as plt

    points_xy = scan.to_xy_m()
    if not points_xy:
        raise ValueError("selected scan has no valid lidar points")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    xs = [point[0] for point in points_xy]
    ys = [point[1] for point in points_xy]

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.scatter(xs, ys, s=5)
    ax.scatter([0.0], [0.0], s=30, marker="+", color="red")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x_m")
    ax.set_ylabel("y_m")
    ax.grid(True, linewidth=0.4)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return len(points_xy)


def select_scan(sequence, scan_index: int):
    if scan_index < 0 or scan_index >= len(sequence.scans):
        raise IndexError(f"scan index {scan_index} out of range for {len(sequence.scans)} scans")
    return sequence.scans[scan_index]


def main(argv: list[str] | None = None) -> int:
    from ackermann_robot.slam.lidar_loader import load_lidar_scan_sequence

    args = build_parser().parse_args(argv)
    output_path = args.output or default_output_path(args.scan_index)
    try:
        sequence = load_lidar_scan_sequence(
            args.input_csv,
            wrap_high_deg=args.wrap_high_deg,
            wrap_low_deg=args.wrap_low_deg,
        )
        scan = select_scan(sequence, args.scan_index)
        point_count = save_single_scan_plot(scan, output_path)
    except Exception as exc:
        print(f"Failed to plot lidar scan: {exc!r}", file=sys.stderr)
        return 1

    print(f"scan_count: {len(sequence.scans)}")
    print(f"scan_index: {args.scan_index}")
    print(f"points: {point_count}")
    print(f"output: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
