#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
SCRIPT_ROOT = REPO_ROOT / "scripts"
for path in (SRC_ROOT, SCRIPT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

REPLAY_OUTPUT_DIR_NAME = "replay_outputs"
LIDAR_XY_PLOT_NAME = "rplidar_scan_xy.png"
LIDAR_GRID_PLOT_NAME = "rplidar_occupancy_grid.png"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Replay and summarize a saved read-only robot sensor run folder."
    )
    parser.add_argument("run_folder", type=Path, help="Saved data/runs/run_* folder.")
    parser.add_argument("--grid-width-m", type=float, default=8.0, help="Occupancy grid width.")
    parser.add_argument("--grid-height-m", type=float, default=8.0, help="Occupancy grid height.")
    parser.add_argument(
        "--grid-resolution-m",
        type=float,
        default=0.05,
        help="Occupancy grid cell resolution.",
    )
    return parser


def validate_saved_run(run_folder: Path) -> tuple[dict[str, Any], dict[str, Any], Any]:
    from validate_readonly_sensor_run import (
        load_metadata,
        missing_required_data,
        summarize_csv,
        summarize_images,
    )

    metadata = load_metadata(run_folder)
    csv_summaries = {
        filename: summarize_csv(run_folder / filename)
        for filename in ("c30d_feedback.csv", "c30d_odometry.csv", "rplidar_scan.csv")
    }
    image_summary = summarize_images(run_folder / "oak_rgb")
    missing = missing_required_data(metadata, csv_summaries, image_summary)
    if missing:
        raise ValueError(f"missing required enabled sensor data: {', '.join(missing)}")
    return metadata, csv_summaries, image_summary


def replay_c30d_odometry(run_folder: Path, output_dir: Path) -> tuple[Path, Path] | None:
    from plot_c30d_odometry import load_odometry_csv, save_plots

    odometry_path = run_folder / "c30d_odometry.csv"
    if not odometry_path.is_file():
        return None

    series = load_odometry_csv(odometry_path)
    return save_plots([series], output_dir)


def load_lidar_counts(lidar_path: Path) -> tuple[int, int]:
    from ackermann_robot.slam.lidar_loader import load_lidar_csv

    scan = load_lidar_csv(lidar_path)
    return len(scan.points), len(scan.valid_points())


def replay_lidar(
    run_folder: Path,
    output_dir: Path,
    grid_width_m: float,
    grid_height_m: float,
    grid_resolution_m: float,
) -> tuple[Path, Path] | None:
    from build_occupancy_grid_from_scan import build_occupancy_grid, save_grid_png
    from plot_lidar_scan import save_lidar_plot

    lidar_path = run_folder / "rplidar_scan.csv"
    if not lidar_path.is_file():
        return None

    _, valid_point_count = load_lidar_counts(lidar_path)
    if valid_point_count == 0:
        print("rplidar: no valid distance_mm > 0 points; skipping lidar plots")
        return None

    xy_plot_path = output_dir / LIDAR_XY_PLOT_NAME
    grid_plot_path = output_dir / LIDAR_GRID_PLOT_NAME
    save_lidar_plot(lidar_path, xy_plot_path)
    grid, _, _, _ = build_occupancy_grid(
        lidar_path,
        width_m=grid_width_m,
        height_m=grid_height_m,
        resolution_m=grid_resolution_m,
    )
    save_grid_png(grid, grid_plot_path)
    return xy_plot_path, grid_plot_path


def print_oak_images(image_summary: Any) -> None:
    print(f"oak_image_count: {image_summary.image_count}")
    for filename, _size_bytes in image_summary.files:
        print(f"oak_image: {image_summary.path / filename}")


def print_replay_summary(
    metadata: dict[str, Any],
    csv_summaries: dict[str, Any],
    image_summary: Any,
    lidar_valid_point_count: int | None,
    output_dir: Path,
) -> None:
    c30d_summary = csv_summaries["c30d_odometry.csv"]
    lidar_summary = csv_summaries["rplidar_scan.csv"]

    print("Replay Summary")
    print(f"enabled_sensors: {metadata.get('enabled_sensors', [])}")
    print(f"c30d_row_count: {c30d_summary.row_count if c30d_summary.exists else 0}")
    print(f"final_odometry: {c30d_summary.final_odometry}")
    print(f"lidar_point_count: {lidar_summary.lidar_point_count or 0}")
    print(f"lidar_valid_point_count: {lidar_valid_point_count or 0}")
    print(f"zero_distance_point_count: {lidar_summary.lidar_zero_distance_count or 0}")
    print(f"oak_image_count: {image_summary.image_count}")
    print(f"output_dir: {output_dir}")


def replay_run_folder(
    run_folder: Path,
    grid_width_m: float = 8.0,
    grid_height_m: float = 8.0,
    grid_resolution_m: float = 0.05,
) -> int:
    metadata, csv_summaries, image_summary = validate_saved_run(run_folder)
    output_dir = run_folder / REPLAY_OUTPUT_DIR_NAME
    output_dir.mkdir(parents=True, exist_ok=True)

    c30d_outputs = replay_c30d_odometry(run_folder, output_dir)
    if c30d_outputs is not None:
        xy_path, x_over_frame_path = c30d_outputs
        print(f"c30d_xy_plot: {xy_path}")
        print(f"c30d_x_over_frame_plot: {x_over_frame_path}")

    lidar_valid_point_count = None
    lidar_path = run_folder / "rplidar_scan.csv"
    if lidar_path.is_file():
        _, lidar_valid_point_count = load_lidar_counts(lidar_path)
        lidar_outputs = replay_lidar(
            run_folder,
            output_dir,
            grid_width_m=grid_width_m,
            grid_height_m=grid_height_m,
            grid_resolution_m=grid_resolution_m,
        )
        if lidar_outputs is not None:
            lidar_xy_path, lidar_grid_path = lidar_outputs
            print(f"lidar_xy_plot: {lidar_xy_path}")
            print(f"lidar_occupancy_grid: {lidar_grid_path}")

    print_oak_images(image_summary)
    print_replay_summary(
        metadata=metadata,
        csv_summaries=csv_summaries,
        image_summary=image_summary,
        lidar_valid_point_count=lidar_valid_point_count,
        output_dir=output_dir,
    )
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return replay_run_folder(
            args.run_folder,
            grid_width_m=args.grid_width_m,
            grid_height_m=args.grid_height_m,
            grid_resolution_m=args.grid_resolution_m,
        )
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"replay failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
