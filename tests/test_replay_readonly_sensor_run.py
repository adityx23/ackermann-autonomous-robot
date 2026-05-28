from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import yaml


def load_replay_script():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "replay_readonly_sensor_run.py"
    spec = importlib.util.spec_from_file_location(script_path.stem, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[script_path.stem] = module
    spec.loader.exec_module(module)
    return module


def write_metadata(run_dir: Path, enabled_sensors: list[str]) -> None:
    metadata = {
        "start_time": "2026-05-28T12:34:56+00:00",
        "duration_s": 5.0,
        "enabled_sensors": enabled_sensors,
        "c30d": {"enabled": "c30d" in enabled_sensors},
        "rplidar": {"enabled": "rplidar" in enabled_sensors},
        "oak": {"enabled": "oak" in enabled_sensors, "rgb_output_dir": "oak_rgb"},
    }
    (run_dir / "metadata.yaml").write_text(yaml.safe_dump(metadata), encoding="utf-8")


def write_odometry(run_dir: Path) -> None:
    (run_dir / "c30d_odometry.csv").write_text(
        "\n".join(
            [
                "frame_index,timestamp_s,x_m,y_m,theta_rad",
                "0,1.0,0.0,0.0,0.0",
                "1,1.1,0.3,0.1,0.02",
            ]
        ),
        encoding="utf-8",
    )


def write_lidar(run_dir: Path) -> None:
    (run_dir / "rplidar_scan.csv").write_text(
        "\n".join(
            [
                "timestamp_s,angle_deg,distance_mm,quality",
                "2.0,0.0,1000.0,10",
                "2.1,90.0,1000.0,11",
                "2.2,180.0,0.0,0",
                "2.3,270.0,-5.0,0",
            ]
        ),
        encoding="utf-8",
    )


def test_replay_run_folder_generates_outputs_and_summary(tmp_path: Path, capsys):
    module = load_replay_script()
    run_dir = tmp_path / "run_20260528_123456"
    run_dir.mkdir()
    write_metadata(run_dir, ["c30d", "rplidar", "oak"])
    (run_dir / "c30d_feedback.csv").write_text("frame_index\n0\n1\n", encoding="utf-8")
    write_odometry(run_dir)
    write_lidar(run_dir)
    image_dir = run_dir / "oak_rgb"
    image_dir.mkdir()
    (image_dir / "oak_rgb_0000.jpg").write_bytes(b"fake image bytes")

    exit_code = module.replay_run_folder(
        run_dir,
        grid_width_m=4.0,
        grid_height_m=4.0,
        grid_resolution_m=0.25,
    )

    output_dir = run_dir / "replay_outputs"
    captured = capsys.readouterr()
    assert exit_code == 0
    assert (output_dir / "c30d_odometry_xy.png").is_file()
    assert (output_dir / "c30d_odometry_x_over_frame.png").is_file()
    assert (output_dir / "rplidar_scan_xy.png").is_file()
    assert (output_dir / "rplidar_occupancy_grid.png").is_file()
    assert "enabled_sensors: ['c30d', 'rplidar', 'oak']" in captured.out
    assert "c30d_row_count: 2" in captured.out
    assert "final_odometry: ('0.3', '0.1', '0.02')" in captured.out
    assert "lidar_point_count: 4" in captured.out
    assert "lidar_valid_point_count: 2" in captured.out
    assert "zero_distance_point_count: 1" in captured.out
    assert "oak_image_count: 1" in captured.out
    assert "oak_image:" in captured.out


def test_replay_run_folder_rejects_missing_enabled_data(tmp_path: Path):
    module = load_replay_script()
    run_dir = tmp_path / "run_20260528_123456"
    run_dir.mkdir()
    write_metadata(run_dir, ["rplidar"])

    try:
        module.replay_run_folder(run_dir)
    except ValueError as exc:
        assert "rplidar_scan.csv" in str(exc)
    else:
        raise AssertionError("expected missing enabled data to fail")


def test_replay_run_folder_skips_lidar_outputs_when_all_distances_invalid(
    tmp_path: Path, capsys
):
    module = load_replay_script()
    run_dir = tmp_path / "run_20260528_123456"
    run_dir.mkdir()
    write_metadata(run_dir, ["rplidar"])
    (run_dir / "rplidar_scan.csv").write_text(
        "\n".join(
            [
                "timestamp_s,angle_deg,distance_mm,quality",
                "2.0,0.0,0.0,0",
                "2.1,90.0,-5.0,0",
            ]
        ),
        encoding="utf-8",
    )

    exit_code = module.replay_run_folder(run_dir)

    output_dir = run_dir / "replay_outputs"
    captured = capsys.readouterr()
    assert exit_code == 0
    assert not (output_dir / "rplidar_scan_xy.png").exists()
    assert not (output_dir / "rplidar_occupancy_grid.png").exists()
    assert "no valid distance_mm > 0 points" in captured.out
    assert "lidar_point_count: 2" in captured.out
    assert "lidar_valid_point_count: 0" in captured.out
    assert "zero_distance_point_count: 1" in captured.out
