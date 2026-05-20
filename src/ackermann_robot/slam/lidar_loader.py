from __future__ import annotations

import csv
from pathlib import Path

from ackermann_robot.slam.lidar_types import LidarPoint, LidarScan, LidarScanSequence

REQUIRED_COLUMNS = ("timestamp_s", "angle_deg", "distance_mm", "quality")


class LidarLoadError(ValueError):
    """Raised when a recorded RPLIDAR CSV cannot be loaded as scan data."""


def load_lidar_csv(path: str | Path) -> LidarScan:
    csv_path = Path(path)
    points = _load_lidar_points(csv_path)

    if not points:
        raise LidarLoadError(f"lidar CSV contains no points: {csv_path}")

    timestamps = [point.timestamp_s for point in points]
    return LidarScan(points=points, start_time_s=min(timestamps), end_time_s=max(timestamps))


def load_lidar_scan_sequence(
    path: str | Path,
    wrap_high_deg: float = 300.0,
    wrap_low_deg: float = 60.0,
) -> LidarScanSequence:
    if wrap_low_deg >= wrap_high_deg:
        raise ValueError("wrap_low_deg must be lower than wrap_high_deg")

    csv_path = Path(path)
    valid_points = [point for point in _load_lidar_points(csv_path) if point.is_valid()]
    if not valid_points:
        raise LidarLoadError(f"lidar CSV contains no valid points: {csv_path}")

    scans: list[LidarScan] = []
    current_points: list[LidarPoint] = []
    previous_angle: float | None = None
    scan_id = 0

    for point in valid_points:
        if (
            previous_angle is not None
            and previous_angle >= wrap_high_deg
            and point.angle_deg <= wrap_low_deg
            and current_points
        ):
            scans.append(_scan_from_points(current_points))
            scan_id += 1
            current_points = []

        current_points.append(_with_scan_id(point, scan_id))
        previous_angle = point.angle_deg

    if current_points:
        scans.append(_scan_from_points(current_points))

    return LidarScanSequence(scans=scans)


def _load_lidar_points(csv_path: Path) -> list[LidarPoint]:
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

    return points


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


def _scan_from_points(points: list[LidarPoint]) -> LidarScan:
    timestamps = [point.timestamp_s for point in points]
    return LidarScan(points=points, start_time_s=min(timestamps), end_time_s=max(timestamps))


def _with_scan_id(point: LidarPoint, scan_id: int) -> LidarPoint:
    return LidarPoint(
        timestamp_s=point.timestamp_s,
        angle_deg=point.angle_deg,
        distance_mm=point.distance_mm,
        quality=point.quality,
        scan_id=scan_id,
    )
