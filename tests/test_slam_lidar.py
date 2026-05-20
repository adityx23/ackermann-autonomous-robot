from __future__ import annotations

import importlib.util
from datetime import datetime
from pathlib import Path

import pytest

from ackermann_robot.slam.lidar_loader import (
    LidarLoadError,
    load_lidar_csv,
    load_lidar_scan_sequence,
)
from ackermann_robot.slam.lidar_types import LidarPoint, LidarScan
from ackermann_robot.slam.occupancy_grid import FREE, OCCUPIED, UNKNOWN, OccupancyGrid


def load_script(name: str):
    script_path = Path(__file__).resolve().parents[1] / "scripts" / name
    spec = importlib.util.spec_from_file_location(script_path.stem, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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


def test_load_lidar_scan_sequence_keeps_one_monotonic_scan(tmp_path: Path):
    csv_path = tmp_path / "one_scan.csv"
    csv_path.write_text(
        "\n".join(
            [
                "timestamp_s,angle_deg,distance_mm,quality",
                "1.0,0.0,1000.0,1",
                "1.1,90.0,1000.0,1",
                "1.2,180.0,1000.0,1",
                "1.3,270.0,1000.0,1",
            ]
        ),
        encoding="utf-8",
    )

    sequence = load_lidar_scan_sequence(csv_path)

    assert len(sequence.scans) == 1
    assert sequence.total_points() == 4
    assert [point.scan_id for point in sequence.scans[0].points] == [0, 0, 0, 0]
    assert sequence.scans[0].start_time_s == 1.0
    assert sequence.scans[0].end_time_s == 1.3


def test_load_lidar_scan_sequence_splits_on_angle_wraparound(tmp_path: Path):
    csv_path = tmp_path / "two_scans.csv"
    csv_path.write_text(
        "\n".join(
            [
                "timestamp_s,angle_deg,distance_mm,quality",
                "1.0,10.0,1000.0,1",
                "1.1,350.0,1000.0,1",
                "1.2,5.0,1000.0,1",
                "1.3,45.0,1000.0,1",
            ]
        ),
        encoding="utf-8",
    )

    sequence = load_lidar_scan_sequence(csv_path)

    assert len(sequence.scans) == 2
    assert [len(scan.points) for scan in sequence.scans] == [2, 2]
    assert [point.scan_id for point in sequence.scans[0].points] == [0, 0]
    assert [point.scan_id for point in sequence.scans[1].points] == [1, 1]


def test_load_lidar_scan_sequence_ignores_invalid_distances(tmp_path: Path):
    csv_path = tmp_path / "invalid_ranges.csv"
    csv_path.write_text(
        "\n".join(
            [
                "timestamp_s,angle_deg,distance_mm,quality",
                "1.0,0.0,1000.0,1",
                "1.1,10.0,0.0,1",
                "1.2,20.0,-5.0,1",
                "1.3,30.0,nan,1",
                "1.4,40.0,2000.0,1",
            ]
        ),
        encoding="utf-8",
    )

    sequence = load_lidar_scan_sequence(csv_path)

    assert len(sequence.scans) == 1
    assert [point.angle_deg for point in sequence.scans[0].points] == [0.0, 40.0]


def test_load_lidar_scan_sequence_rejects_empty_file(tmp_path: Path):
    csv_path = tmp_path / "empty.csv"
    csv_path.write_text("timestamp_s,angle_deg,distance_mm,quality\n", encoding="utf-8")

    with pytest.raises(LidarLoadError, match="contains no valid points"):
        load_lidar_scan_sequence(csv_path)


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


def test_occupancy_grid_ray_tracing_marks_free_cells_and_endpoint():
    grid = OccupancyGrid(width=7, height=5, resolution_m=1.0, origin_x_m=-3.0, origin_y_m=-2.0)
    scan = LidarScan(
        points=[LidarPoint(timestamp_s=1.0, angle_deg=0.0, distance_mm=2000.0)],
        start_time_s=1.0,
        end_time_s=1.0,
    )

    assert grid.update_from_lidar_scan(scan) == 1
    origin_cell = grid.world_to_grid(0.0, 0.0)
    between_cell = grid.world_to_grid(1.0, 0.0)
    endpoint_cell = grid.world_to_grid(2.0, 0.0)

    assert origin_cell == (3, 2)
    assert grid.data[origin_cell[1], origin_cell[0]] == FREE
    assert between_cell == (4, 2)
    assert grid.data[between_cell[1], between_cell[0]] == FREE
    assert endpoint_cell == (5, 2)
    assert grid.data[endpoint_cell[1], endpoint_cell[0]] == OCCUPIED


def test_occupancy_grid_ray_tracing_ignores_out_of_bounds_points():
    grid = OccupancyGrid(width=3, height=3, resolution_m=1.0, origin_x_m=-1.5, origin_y_m=-1.5)
    scan = LidarScan(
        points=[LidarPoint(timestamp_s=1.0, angle_deg=0.0, distance_mm=4000.0)],
        start_time_s=1.0,
        end_time_s=1.0,
    )

    assert grid.world_to_grid(0.0, 0.0) == (1, 1)
    assert grid.update_from_lidar_scan(scan) == 0
    assert (grid.data == UNKNOWN).all()


def test_build_occupancy_grid_script_parser_defaults():
    module = load_script("build_occupancy_grid_from_scan.py")

    args = module.build_parser().parse_args(["scan.csv"])

    assert args.input_csv == Path("scan.csv")
    assert args.width_m == 8.0
    assert args.height_m == 8.0
    assert args.resolution_m == 0.05
    assert args.output_dir == Path("data/slam_tests")


def test_build_occupancy_grid_script_default_output_path():
    module = load_script("build_occupancy_grid_from_scan.py")

    output = module.default_output_path(
        Path("data/slam_tests"), now=datetime(2026, 5, 20, 8, 9, 10)
    )

    assert output == Path("data/slam_tests/occupancy_grid_20260520_080910.png")


def test_build_occupancy_grid_script_uses_loader_and_grid(tmp_path: Path):
    module = load_script("build_occupancy_grid_from_scan.py")
    csv_path = tmp_path / "scan.csv"
    csv_path.write_text(
        "\n".join(
            [
                "timestamp_s,angle_deg,distance_mm,quality",
                "1.0,0.0,1000.0,10",
                "1.0,90.0,1000.0,10",
                "1.0,45.0,0.0,10",
            ]
        ),
        encoding="utf-8",
    )

    grid, valid_points, occupied_cells, free_cells = module.build_occupancy_grid(
        csv_path, width_m=4.0, height_m=4.0, resolution_m=1.0
    )

    assert grid.width == 4
    assert grid.height == 4
    assert grid.resolution_m == 1.0
    assert grid.world_to_grid(0.0, 0.0) == (2, 2)
    assert valid_points == 2
    assert occupied_cells == 2
    assert free_cells >= 1
    assert grid.data[2, 3] == OCCUPIED
    assert grid.data[3, 2] == OCCUPIED


def test_build_occupancy_grid_script_main_prints_summary(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
):
    module = load_script("build_occupancy_grid_from_scan.py")
    csv_path = tmp_path / "scan.csv"
    output_dir = tmp_path / "out"
    csv_path.write_text(
        "timestamp_s,angle_deg,distance_mm,quality\n1.0,0.0,1000.0,10\n",
        encoding="utf-8",
    )
    saved_paths: list[Path] = []

    def fake_save_grid_png(_grid, output_path: Path) -> None:
        saved_paths.append(output_path)

    monkeypatch.setattr(module, "save_grid_png", fake_save_grid_png)

    result = module.main(
        [
            str(csv_path),
            "--width-m",
            "4",
            "--height-m",
            "4",
            "--resolution-m",
            "1",
            "--output-dir",
            str(output_dir),
        ]
    )

    captured = capsys.readouterr()
    assert result == 0
    assert "grid_width: 4" in captured.out
    assert "grid_height: 4" in captured.out
    assert "resolution_m: 1.0" in captured.out
    assert "valid_points: 1" in captured.out
    assert "occupied_cells: 1" in captured.out
    assert "free_cells:" in captured.out
    assert saved_paths
    assert saved_paths[0].parent == output_dir


def test_split_lidar_scans_script_saves_individual_csvs(tmp_path: Path):
    module = load_script("split_lidar_scans.py")
    sequence = load_lidar_scan_sequence(_write_two_scan_csv(tmp_path))

    rows = module.scan_summary_rows(sequence)
    paths = module.save_split_scans(sequence, tmp_path / "split", "capture")

    assert rows == [
        {"scan_index": 0, "point_count": 2, "duration_s": pytest.approx(0.1)},
        {"scan_index": 1, "point_count": 2, "duration_s": pytest.approx(0.1)},
    ]
    assert len(paths) == 2
    assert paths[0].read_text(encoding="utf-8").splitlines()[0] == (
        "timestamp_s,angle_deg,distance_mm,quality"
    )


def test_split_lidar_scans_script_main_prints_summary(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
):
    module = load_script("split_lidar_scans.py")
    csv_path = _write_two_scan_csv(tmp_path)

    result = module.main([str(csv_path)])

    captured = capsys.readouterr()
    assert result == 0
    assert "scan_count: 2" in captured.out
    assert "scan 0: points=2" in captured.out
    assert "scan 1: points=2" in captured.out


def test_plot_single_lidar_scan_script_selects_scan_and_defaults_output(tmp_path: Path):
    module = load_script("plot_single_lidar_scan.py")
    sequence = load_lidar_scan_sequence(_write_two_scan_csv(tmp_path))

    selected = module.select_scan(sequence, 1)
    output = module.default_output_path(1, now=datetime(2026, 5, 20, 8, 9, 10))

    assert [point.scan_id for point in selected.points] == [1, 1]
    assert output == Path("data/slam_tests/lidar_scan_001_20260520_080910.png")
    with pytest.raises(IndexError, match="out of range"):
        module.select_scan(sequence, 2)


def test_plot_single_lidar_scan_script_main_prints_summary(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
):
    module = load_script("plot_single_lidar_scan.py")
    csv_path = _write_two_scan_csv(tmp_path)
    output_path = tmp_path / "scan.png"

    def fake_save_single_scan_plot(scan, _output_path: Path) -> int:
        return len(scan.points)

    monkeypatch.setattr(module, "save_single_scan_plot", fake_save_single_scan_plot)

    result = module.main([str(csv_path), "--scan-index", "1", "--output", str(output_path)])

    captured = capsys.readouterr()
    assert result == 0
    assert "scan_count: 2" in captured.out
    assert "scan_index: 1" in captured.out
    assert "points: 2" in captured.out
    assert f"output: {output_path}" in captured.out


def _write_two_scan_csv(tmp_path: Path) -> Path:
    csv_path = tmp_path / "two_scan_capture.csv"
    csv_path.write_text(
        "\n".join(
            [
                "timestamp_s,angle_deg,distance_mm,quality",
                "1.0,10.0,1000.0,1",
                "1.1,350.0,1000.0,1",
                "1.2,5.0,1000.0,1",
                "1.3,45.0,1000.0,1",
            ]
        ),
        encoding="utf-8",
    )
    return csv_path
