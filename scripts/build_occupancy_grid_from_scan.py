#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import sys
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

DEFAULT_OUTPUT_DIR = Path("data/slam_tests")
DEFAULT_WIDTH_M = 8.0
DEFAULT_HEIGHT_M = 8.0
DEFAULT_RESOLUTION_M = 0.05


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build a simple offline occupancy-grid PNG from a recorded RPLIDAR CSV."
    )
    parser.add_argument(
        "input_csv", type=Path, help="Recorded RPLIDAR CSV from rplidar_scan_sample.py."
    )
    parser.add_argument(
        "--width-m", type=float, default=DEFAULT_WIDTH_M, help="Grid width in meters."
    )
    parser.add_argument(
        "--height-m", type=float, default=DEFAULT_HEIGHT_M, help="Grid height in meters."
    )
    parser.add_argument(
        "--resolution-m",
        type=float,
        default=DEFAULT_RESOLUTION_M,
        help="Grid cell resolution in meters.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Directory for the generated occupancy-grid PNG.",
    )
    return parser


def default_output_path(output_dir: Path, now: datetime | None = None) -> Path:
    timestamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
    return output_dir / f"occupancy_grid_{timestamp}.png"


def grid_shape(width_m: float, height_m: float, resolution_m: float) -> tuple[int, int]:
    if width_m <= 0.0:
        raise ValueError("--width-m must be positive")
    if height_m <= 0.0:
        raise ValueError("--height-m must be positive")
    if resolution_m <= 0.0:
        raise ValueError("--resolution-m must be positive")
    return math.ceil(width_m / resolution_m), math.ceil(height_m / resolution_m)


def build_occupancy_grid(input_csv: Path, width_m: float, height_m: float, resolution_m: float):
    from ackermann_robot.slam.lidar_loader import load_lidar_csv
    from ackermann_robot.slam.occupancy_grid import OccupancyGrid

    width_cells, height_cells = grid_shape(width_m, height_m, resolution_m)
    grid = OccupancyGrid(
        width=width_cells,
        height=height_cells,
        resolution_m=resolution_m,
        origin_x_m=-width_m / 2.0,
        origin_y_m=-height_m / 2.0,
    )
    scan = load_lidar_csv(input_csv)
    valid_points = len(scan.valid_points())
    grid.update_from_lidar_scan(scan)
    occupied_cells, free_cells = count_grid_cells(grid)
    return grid, valid_points, occupied_cells, free_cells


def count_grid_cells(grid) -> tuple[int, int]:
    import numpy as np

    from ackermann_robot.slam.occupancy_grid import FREE, OCCUPIED

    occupied_cells = int(np.count_nonzero(grid.data == OCCUPIED))
    free_cells = int(np.count_nonzero(grid.data == FREE))
    return occupied_cells, free_cells


def save_grid_png(grid, output_path: Path) -> None:
    import matplotlib.pyplot as plt
    import numpy as np

    from ackermann_robot.slam.occupancy_grid import FREE, OCCUPIED, UNKNOWN

    output_path.parent.mkdir(parents=True, exist_ok=True)
    image = np.full(grid.data.shape, 0.75, dtype=float)
    image[grid.data == UNKNOWN] = 0.55
    image[grid.data == FREE] = 1.0
    image[grid.data == OCCUPIED] = 0.0

    extent = [
        grid.origin_x_m,
        grid.origin_x_m + grid.width * grid.resolution_m,
        grid.origin_y_m,
        grid.origin_y_m + grid.height * grid.resolution_m,
    ]

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.imshow(image, cmap="gray", origin="lower", extent=extent, vmin=0.0, vmax=1.0)
    ax.scatter([0.0], [0.0], s=30, marker="+", color="red")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x_m")
    ax.set_ylabel("y_m")
    ax.grid(True, linewidth=0.3)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    output_path = default_output_path(args.output_dir)
    try:
        grid, valid_points, occupied_cells, free_cells = build_occupancy_grid(
            args.input_csv, args.width_m, args.height_m, args.resolution_m
        )
        save_grid_png(grid, output_path)
    except Exception as exc:
        print(f"Failed to build occupancy grid: {exc!r}", file=sys.stderr)
        return 1

    print(f"grid_width: {grid.width}")
    print(f"grid_height: {grid.height}")
    print(f"resolution_m: {grid.resolution_m}")
    print(f"valid_points: {valid_points}")
    print(f"occupied_cells: {occupied_cells}")
    print(f"free_cells: {free_cells}")
    print(f"output: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
