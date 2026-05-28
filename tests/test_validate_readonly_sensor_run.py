from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import yaml


def load_validate_script():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "validate_readonly_sensor_run.py"
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


def test_validate_run_folder_summarizes_complete_run(tmp_path: Path, capsys):
    module = load_validate_script()
    run_dir = tmp_path / "run_20260528_123456"
    run_dir.mkdir()
    write_metadata(run_dir, ["c30d", "rplidar", "oak"])
    (run_dir / "c30d_feedback.csv").write_text(
        "\n".join(
            [
                "frame_index,candidate_forward_motion",
                "0,10",
                "1,11",
            ]
        ),
        encoding="utf-8",
    )
    (run_dir / "c30d_odometry.csv").write_text(
        "\n".join(
            [
                "frame_index,timestamp_s,x_m,y_m,theta_rad",
                "0,1.0,0.1,0.0,0.0",
                "1,1.2,0.3,0.1,0.02",
            ]
        ),
        encoding="utf-8",
    )
    (run_dir / "rplidar_scan.csv").write_text(
        "\n".join(
            [
                "timestamp_s,angle_deg,distance_mm,quality",
                "2.0,0.0,1000.0,10",
                "2.1,90.0,1500.0,12",
            ]
        ),
        encoding="utf-8",
    )
    image_dir = run_dir / "oak_rgb"
    image_dir.mkdir()
    (image_dir / "oak_rgb_0000.jpg").write_bytes(b"fake image bytes")

    exit_code = module.validate_run_folder(run_dir)

    captured = capsys.readouterr()
    assert exit_code == 0
    assert "c30d_feedback.csv:" in captured.out
    assert "rows: 2" in captured.out
    assert "first_frame_index: 0" in captured.out
    assert "last_frame_index: 1" in captured.out
    assert "final_odometry: x_m=0.3, y_m=0.1, theta_rad=0.02" in captured.out
    assert "point_count: 2" in captured.out
    assert "timestamp_duration_s: 0.1" in captured.out
    assert "distance_range: min=1000, max=1500" in captured.out
    assert "zero_distance_points: 0" in captured.out
    assert "images: 1" in captured.out
    assert "validation: ok" in captured.out


def test_validate_run_folder_missing_disabled_sensor_data_is_ok(tmp_path: Path, capsys):
    module = load_validate_script()
    run_dir = tmp_path / "run_20260528_123456"
    run_dir.mkdir()
    write_metadata(run_dir, ["rplidar"])
    (run_dir / "rplidar_scan.csv").write_text(
        "timestamp_s,angle_deg,distance_mm,quality\n2.0,0.0,1000.0,10\n",
        encoding="utf-8",
    )

    exit_code = module.validate_run_folder(run_dir)

    captured = capsys.readouterr()
    assert exit_code == 0
    assert "c30d_feedback.csv:\n  missing" in captured.out
    assert "oak_rgb/:\n  missing" in captured.out
    assert "validation: ok" in captured.out


def test_validate_run_folder_missing_enabled_sensor_data_fails(tmp_path: Path, capsys):
    module = load_validate_script()
    run_dir = tmp_path / "run_20260528_123456"
    run_dir.mkdir()
    write_metadata(run_dir, ["c30d", "oak"])
    (run_dir / "c30d_feedback.csv").write_text(
        "frame_index,candidate_forward_motion\n0,10\n",
        encoding="utf-8",
    )
    (run_dir / "oak_rgb").mkdir()

    exit_code = module.validate_run_folder(run_dir)

    captured = capsys.readouterr()
    assert exit_code == 1
    assert "missing_required_data:" in captured.out
    assert "c30d_odometry.csv" in captured.out
    assert "oak_rgb images" in captured.out


def test_summarize_csv_handles_empty_csv_with_headers(tmp_path: Path):
    module = load_validate_script()
    csv_path = tmp_path / "empty.csv"
    csv_path.write_text("timestamp_s,frame_index,x_m,y_m,theta_rad\n", encoding="utf-8")

    summary = module.summarize_csv(csv_path)

    assert summary.exists is True
    assert summary.row_count == 0
    assert summary.columns == ("timestamp_s", "frame_index", "x_m", "y_m", "theta_rad")
    assert summary.first_timestamp is None
    assert summary.final_odometry is None


def test_rplidar_changing_timestamps_do_not_warn(tmp_path: Path, capsys):
    module = load_validate_script()
    run_dir = tmp_path / "run_20260528_123456"
    run_dir.mkdir()
    write_metadata(run_dir, ["rplidar"])
    (run_dir / "rplidar_scan.csv").write_text(
        "\n".join(
            [
                "timestamp_s,angle_deg,distance_mm,quality",
                "10.0,0.0,1000.0,10",
                "10.5,90.0,1200.0,11",
                "11.0,180.0,1300.0,12",
            ]
        ),
        encoding="utf-8",
    )

    exit_code = module.validate_run_folder(run_dir)

    captured = capsys.readouterr()
    assert exit_code == 0
    assert "timestamp_duration_s: 1" in captured.out
    assert "warning: constant timestamps" not in captured.out


def test_rplidar_constant_timestamps_generate_warning(tmp_path: Path, capsys):
    module = load_validate_script()
    run_dir = tmp_path / "run_20260528_123456"
    run_dir.mkdir()
    write_metadata(run_dir, ["rplidar"])
    (run_dir / "rplidar_scan.csv").write_text(
        "\n".join(
            [
                "timestamp_s,angle_deg,distance_mm,quality",
                "10.0,0.0,1000.0,10",
                "10.0,90.0,1200.0,11",
            ]
        ),
        encoding="utf-8",
    )

    exit_code = module.validate_run_folder(run_dir)

    captured = capsys.readouterr()
    assert exit_code == 0
    assert "timestamp_duration_s: 0" in captured.out
    assert "warning: constant timestamps across multiple rows" in captured.out


def test_rplidar_zero_distances_are_counted(tmp_path: Path, capsys):
    module = load_validate_script()
    run_dir = tmp_path / "run_20260528_123456"
    run_dir.mkdir()
    write_metadata(run_dir, ["rplidar"])
    (run_dir / "rplidar_scan.csv").write_text(
        "\n".join(
            [
                "timestamp_s,angle_deg,distance_mm,quality",
                "10.0,0.0,0.0,0",
                "10.1,90.0,-5.0,0",
                "10.2,180.0,1200.0,12",
            ]
        ),
        encoding="utf-8",
    )

    exit_code = module.validate_run_folder(run_dir)

    captured = capsys.readouterr()
    assert exit_code == 0
    assert "point_count: 3" in captured.out
    assert "zero_distance_points: 1" in captured.out
    assert "nonpositive_distance_points: 2" in captured.out
