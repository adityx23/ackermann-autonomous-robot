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
        description="Plot a recorded RPLIDAR CSV as top-down XY points."
    )
    parser.add_argument(
        "input", type=Path, help="Recorded RPLIDAR CSV from rplidar_scan_sample.py."
    )
    parser.add_argument("--output", type=Path, default=None, help="Output image path.")
    return parser


def default_output_path(now: datetime | None = None) -> Path:
    timestamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
    return DEFAULT_OUTPUT_DIR / f"lidar_scan_xy_{timestamp}.png"


def save_lidar_plot(input_path: Path, output_path: Path) -> int:
    import matplotlib.pyplot as plt

    from ackermann_robot.slam.lidar_loader import load_lidar_csv

    scan = load_lidar_csv(input_path)
    points_xy = scan.to_xy_m()
    if not points_xy:
        raise ValueError(f"recorded scan has no valid lidar points: {input_path}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    xs = [point[0] for point in points_xy]
    ys = [point[1] for point in points_xy]

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.scatter(xs, ys, s=4)
    ax.scatter([0.0], [0.0], s=30, marker="+", color="red")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x_m")
    ax.set_ylabel("y_m")
    ax.grid(True, linewidth=0.4)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return len(points_xy)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    output_path = args.output or default_output_path()
    point_count = save_lidar_plot(args.input, output_path)
    print(f"points: {point_count}")
    print(f"output: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
