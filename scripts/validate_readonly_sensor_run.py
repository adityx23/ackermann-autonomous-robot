#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

TIMESTAMP_COLUMNS = ("timestamp_s", "timestamp", "time_s", "capture_time_s")
FRAME_INDEX_COLUMNS = ("frame_index", "frame", "scan_id", "image_index")
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp"}


@dataclass(frozen=True)
class CsvSummary:
    path: Path
    exists: bool
    row_count: int = 0
    columns: tuple[str, ...] = ()
    first_timestamp: str | None = None
    last_timestamp: str | None = None
    timestamp_duration_s: float | None = None
    constant_timestamp_warning: bool = False
    first_frame_index: str | None = None
    last_frame_index: str | None = None
    final_odometry: tuple[str, str, str] | None = None
    lidar_point_count: int | None = None
    lidar_min_distance: float | None = None
    lidar_max_distance: float | None = None
    lidar_zero_distance_count: int | None = None
    lidar_nonpositive_distance_count: int | None = None


@dataclass(frozen=True)
class ImageSummary:
    path: Path
    exists: bool
    image_count: int = 0
    files: tuple[tuple[str, int], ...] = ()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate and summarize a saved read-only sensor run folder."
    )
    parser.add_argument("run_folder", type=Path, help="Saved data/runs/run_* folder.")
    return parser


def load_metadata(run_folder: Path) -> dict[str, Any]:
    metadata_path = run_folder / "metadata.yaml"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"missing metadata.yaml: {metadata_path}")
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    if not isinstance(metadata, dict):
        raise ValueError(f"metadata.yaml must contain a mapping: {metadata_path}")
    return metadata


def sensor_enabled(metadata: dict[str, Any], sensor: str) -> bool:
    sensor_metadata = metadata.get(sensor)
    if isinstance(sensor_metadata, dict) and "enabled" in sensor_metadata:
        return bool(sensor_metadata["enabled"])

    enabled_sensors = metadata.get("enabled_sensors", [])
    if isinstance(enabled_sensors, list):
        return sensor in enabled_sensors
    return False


def first_existing_column(columns: tuple[str, ...], candidates: tuple[str, ...]) -> str | None:
    for column in candidates:
        if column in columns:
            return column
    return None


def optional_float(value: str | None) -> float | None:
    if value is None or value == "":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def summarize_csv(path: Path) -> CsvSummary:
    if not path.is_file():
        return CsvSummary(path=path, exists=False)

    with path.open(newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        columns = tuple(reader.fieldnames or ())
        timestamp_column = first_existing_column(columns, TIMESTAMP_COLUMNS)
        frame_index_column = first_existing_column(columns, FRAME_INDEX_COLUMNS)
        distance_column = first_existing_column(columns, ("distance_mm", "distance_m"))

        row_count = 0
        first_row: dict[str, str] | None = None
        last_row: dict[str, str] | None = None
        min_distance: float | None = None
        max_distance: float | None = None
        zero_distance_count = 0
        nonpositive_distance_count = 0

        for row in reader:
            if first_row is None:
                first_row = row
            last_row = row
            row_count += 1

            if distance_column is not None:
                distance = optional_float(row.get(distance_column))
                if distance is not None:
                    min_distance = distance if min_distance is None else min(min_distance, distance)
                    max_distance = distance if max_distance is None else max(max_distance, distance)
                    if distance == 0.0:
                        zero_distance_count += 1
                    if distance <= 0.0:
                        nonpositive_distance_count += 1

    final_odometry = None
    if last_row is not None and {"x_m", "y_m", "theta_rad"}.issubset(columns):
        final_odometry = (
            last_row.get("x_m", ""),
            last_row.get("y_m", ""),
            last_row.get("theta_rad", ""),
        )

    lidar_point_count = None
    if distance_column is not None:
        lidar_point_count = row_count

    first_timestamp = first_row.get(timestamp_column) if first_row and timestamp_column else None
    last_timestamp = last_row.get(timestamp_column) if last_row and timestamp_column else None
    first_timestamp_float = optional_float(first_timestamp)
    last_timestamp_float = optional_float(last_timestamp)
    timestamp_duration_s = None
    if first_timestamp_float is not None and last_timestamp_float is not None:
        timestamp_duration_s = last_timestamp_float - first_timestamp_float

    return CsvSummary(
        path=path,
        exists=True,
        row_count=row_count,
        columns=columns,
        first_timestamp=first_timestamp,
        last_timestamp=last_timestamp,
        timestamp_duration_s=timestamp_duration_s,
        constant_timestamp_warning=(
            row_count > 1 and first_timestamp is not None and first_timestamp == last_timestamp
        ),
        first_frame_index=first_row.get(frame_index_column) if first_row and frame_index_column else None,
        last_frame_index=last_row.get(frame_index_column) if last_row and frame_index_column else None,
        final_odometry=final_odometry,
        lidar_point_count=lidar_point_count,
        lidar_min_distance=min_distance,
        lidar_max_distance=max_distance,
        lidar_zero_distance_count=zero_distance_count if distance_column is not None else None,
        lidar_nonpositive_distance_count=(
            nonpositive_distance_count if distance_column is not None else None
        ),
    )


def summarize_images(path: Path) -> ImageSummary:
    if not path.is_dir():
        return ImageSummary(path=path, exists=False)

    files = tuple(
        (image_path.name, image_path.stat().st_size)
        for image_path in sorted(path.iterdir())
        if image_path.is_file() and image_path.suffix.lower() in IMAGE_EXTENSIONS
    )
    return ImageSummary(path=path, exists=True, image_count=len(files), files=files)


def print_csv_summary(summary: CsvSummary) -> None:
    print(f"{summary.path.name}:")
    if not summary.exists:
        print("  missing")
        return

    print(f"  rows: {summary.row_count}")
    print(f"  columns: {', '.join(summary.columns) if summary.columns else '(none)'}")
    if summary.first_timestamp is not None or summary.last_timestamp is not None:
        print(f"  first_timestamp: {summary.first_timestamp}")
        print(f"  last_timestamp: {summary.last_timestamp}")
        if summary.timestamp_duration_s is not None:
            print(f"  timestamp_duration_s: {summary.timestamp_duration_s:g}")
        if summary.constant_timestamp_warning:
            print("  warning: constant timestamps across multiple rows")
    if summary.first_frame_index is not None or summary.last_frame_index is not None:
        print(f"  first_frame_index: {summary.first_frame_index}")
        print(f"  last_frame_index: {summary.last_frame_index}")
    if summary.final_odometry is not None:
        x_m, y_m, theta_rad = summary.final_odometry
        print(f"  final_odometry: x_m={x_m}, y_m={y_m}, theta_rad={theta_rad}")
    if summary.lidar_point_count is not None:
        print(f"  point_count: {summary.lidar_point_count}")
        if summary.lidar_min_distance is not None and summary.lidar_max_distance is not None:
            print(
                "  distance_range: "
                f"min={summary.lidar_min_distance:g}, max={summary.lidar_max_distance:g}"
            )
        print(f"  zero_distance_points: {summary.lidar_zero_distance_count}")
        print(f"  nonpositive_distance_points: {summary.lidar_nonpositive_distance_count}")


def print_image_summary(summary: ImageSummary) -> None:
    print(f"{summary.path.name}/:")
    if not summary.exists:
        print("  missing")
        return
    print(f"  images: {summary.image_count}")
    for filename, size_bytes in summary.files:
        print(f"  {filename}: {size_bytes} bytes")


def missing_required_data(
    metadata: dict[str, Any],
    csv_summaries: dict[str, CsvSummary],
    image_summary: ImageSummary,
) -> list[str]:
    missing: list[str] = []
    if sensor_enabled(metadata, "c30d"):
        for filename in ("c30d_feedback.csv", "c30d_odometry.csv"):
            if not csv_summaries[filename].exists:
                missing.append(filename)
    if sensor_enabled(metadata, "rplidar") and not csv_summaries["rplidar_scan.csv"].exists:
        missing.append("rplidar_scan.csv")
    if sensor_enabled(metadata, "oak") and image_summary.image_count == 0:
        missing.append("oak_rgb images")
    return missing


def validate_run_folder(run_folder: Path) -> int:
    metadata = load_metadata(run_folder)
    csv_summaries = {
        filename: summarize_csv(run_folder / filename)
        for filename in ("c30d_feedback.csv", "c30d_odometry.csv", "rplidar_scan.csv")
    }
    image_summary = summarize_images(run_folder / "oak_rgb")

    print(f"run_folder: {run_folder}")
    print(f"metadata: {run_folder / 'metadata.yaml'}")
    print(f"enabled_sensors: {metadata.get('enabled_sensors', [])}")
    for filename in ("c30d_feedback.csv", "c30d_odometry.csv", "rplidar_scan.csv"):
        print_csv_summary(csv_summaries[filename])
    print_image_summary(image_summary)

    missing = missing_required_data(metadata, csv_summaries, image_summary)
    if missing:
        print("missing_required_data:")
        for item in missing:
            print(f"  {item}")
        return 1

    print("validation: ok")
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return validate_run_folder(args.run_folder)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"validation failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
