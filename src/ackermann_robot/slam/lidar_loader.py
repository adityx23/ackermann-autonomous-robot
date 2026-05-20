from __future__ import annotations

import csv
from pathlib import Path

from ackermann_robot.slam.lidar_types import LidarPoint, LidarScan

REQUIRED_COLUMNS = ("timestamp_s", "angle_deg", "distance_mm", "quality")


class LidarLoadError(ValueError):
    """Raised when a recorded RPLIDAR CSV cannot be loaded as scan data."""


def load_lidar_csv(path: str | Path) -> LidarScan:
    csv_path = Path(path)
    points: list[LidarPoint] = []

    try:
        with csv_path.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            if reader.fieldnames is None:
                raise LidarLoadError(f"empty lidar CSV: {csv_path}")

            missing = [column for column in REQUIRED_COLUMNS if column not in reader.fieldnames]
            if missing:
                raise LidarLoadError(
                    f"lidar CSV {csv_path} is missing columns: {', '.join(missing)}"
                )

            for row_number, row in enumerate(reader, start=2):
                points.append(_row_to_point(row, row_number, csv_path))
    except FileNotFoundError:
        raise
    except LidarLoadError:
        raise
    except csv.Error as exc:
        raise LidarLoadError(f"failed to parse lidar CSV {csv_path}: {exc}") from exc

    if not points:
        raise LidarLoadError(f"lidar CSV contains no points: {csv_path}")

    timestamps = [point.timestamp_s for point in points]
    return LidarScan(points=points, start_time_s=min(timestamps), end_time_s=max(timestamps))


def _row_to_point(row: dict[str, str], row_number: int, csv_path: Path) -> LidarPoint:
    try:
        timestamp_s = float(row["timestamp_s"])
        angle_deg = float(row["angle_deg"])
        distance_mm = float(row["distance_mm"])
        quality = _parse_quality(row["quality"])
    except (TypeError, ValueError) as exc:
        raise LidarLoadError(f"malformed lidar row {row_number} in {csv_path}: {row}") from exc

    return LidarPoint(
        timestamp_s=timestamp_s,
        angle_deg=angle_deg,
        distance_mm=distance_mm,
        quality=quality,
    )


def _parse_quality(value: str | None) -> int | None:
    if value is None or value == "":
        return None
    return int(value)
