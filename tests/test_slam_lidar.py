from __future__ import annotations

from pathlib import Path

import pytest

from ackermann_robot.slam.lidar_loader import LidarLoadError, load_lidar_csv
from ackermann_robot.slam.lidar_types import LidarPoint, LidarScan
from ackermann_robot.slam.occupancy_grid import OCCUPIED, UNKNOWN, OccupancyGrid


def test_lidar_scan_filters_valid_points_and_converts_to_xy():
    scan = LidarScan(
        points=[
            LidarPoint(timestamp_s=1.0, angle_deg=0.0, distance_mm=1000.0, quality=10),
            LidarPoint(timestamp_s=1.1, angle_deg=90.0, distance_mm=2000.0),
            LidarPoint(timestamp_s=1.2, angle_deg=180.0, distance_mm=0.0),
        ],
        start_time_s=1.0,
        end_time_s=1.2,
    )

    assert scan.valid_points() == scan.points[:2]
    xy = scan.to_xy_m()
    assert xy[0] == pytest.approx((1.0, 0.0))
    assert xy[1] == pytest.approx((0.0, 2.0))


def test_load_lidar_csv_reads_rplidar_capture_format(tmp_path: Path):
    csv_path = tmp_path / "scan.csv"
    csv_path.write_text(
        "\n".join(
            [
                "timestamp_s,angle_deg,distance_mm,quality",
                "10.0,0.0,1000.0,15",
                "10.2,90.0,2000.0,",
            ]
        ),
        encoding="utf-8",
    )

    scan = load_lidar_csv(csv_path)

    assert scan.start_time_s == 10.0
    assert scan.end_time_s == 10.2
    assert scan.points == [
        LidarPoint(timestamp_s=10.0, angle_deg=0.0, distance_mm=1000.0, quality=15),
        LidarPoint(timestamp_s=10.2, angle_deg=90.0, distance_mm=2000.0, quality=None),
    ]


def test_load_lidar_csv_rejects_empty_and_malformed_files(tmp_path: Path):
    empty_path = tmp_path / "empty.csv"
    empty_path.write_text("timestamp_s,angle_deg,distance_mm,quality\n", encoding="utf-8")
    malformed_path = tmp_path / "malformed.csv"
    malformed_path.write_text(
        "timestamp_s,angle_deg,distance_mm,quality\n1.0,bad,100.0,4\n",
        encoding="utf-8",
    )

    with pytest.raises(LidarLoadError, match="contains no points"):
        load_lidar_csv(empty_path)
    with pytest.raises(LidarLoadError, match="malformed lidar row"):
        load_lidar_csv(malformed_path)


def test_load_lidar_csv_rejects_missing_columns(tmp_path: Path):
    csv_path = tmp_path / "missing.csv"
    csv_path.write_text("timestamp_s,angle_deg,distance_mm\n1.0,0.0,1000.0\n", encoding="utf-8")

    with pytest.raises(LidarLoadError, match="missing columns"):
        load_lidar_csv(csv_path)


def test_occupancy_grid_converts_coordinates_and_marks_cells():
    grid = OccupancyGrid(width=4, height=3, resolution_m=0.5, origin_x_m=-1.0, origin_y_m=-0.5)

    assert grid.world_to_grid(-0.75, -0.25) == (0, 0)
    assert grid.grid_to_world(0, 0) == pytest.approx((-0.75, -0.25))
    assert grid.world_to_grid(2.0, 0.0) is None
    assert grid.data[0, 0] == UNKNOWN

    assert grid.mark_occupied(-0.75, -0.25) is True
    assert grid.data[0, 0] == OCCUPIED
    assert grid.mark_occupied(2.0, 0.0) is False


def test_occupancy_grid_marks_lidar_scan_points():
    grid = OccupancyGrid(width=6, height=6, resolution_m=0.5, origin_x_m=-1.5, origin_y_m=-1.5)
    scan = LidarScan(
        points=[
            LidarPoint(timestamp_s=1.0, angle_deg=0.0, distance_mm=1000.0),
            LidarPoint(timestamp_s=1.0, angle_deg=90.0, distance_mm=1000.0),
            LidarPoint(timestamp_s=1.0, angle_deg=0.0, distance_mm=0.0),
        ],
        start_time_s=1.0,
        end_time_s=1.0,
    )

    assert grid.mark_lidar_points(scan) == 2
    assert grid.data[3, 5] == OCCUPIED
    assert grid.data[5, 3] == OCCUPIED
