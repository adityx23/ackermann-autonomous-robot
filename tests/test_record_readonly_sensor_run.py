from __future__ import annotations

import argparse
import importlib.util
import sys
from datetime import datetime, timezone
from pathlib import Path

import pytest
import yaml


def load_record_script():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "record_readonly_sensor_run.py"
    spec = importlib.util.spec_from_file_location(script_path.stem, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[script_path.stem] = module
    spec.loader.exec_module(module)
    return module


def test_settings_from_args_and_enabled_sensors(tmp_path):
    module = load_record_script()
    args = argparse.Namespace(
        duration=3.5,
        enable_c30d=True,
        enable_rplidar=False,
        enable_oak=True,
        output_root=tmp_path,
        c30d_port="/dev/mock-c30d",
        c30d_baud=57600,
        rplidar_port="/dev/mock-lidar",
        rplidar_baud=460800,
        oak_fps=4.0,
        oak_preview_width=320,
        oak_preview_height=180,
        oak_timeout=2.0,
    )

    settings = module.settings_from_args(args)

    assert settings.duration_s == 3.5
    assert settings.c30d_port == "/dev/mock-c30d"
    assert settings.oak.preview_width == 320
    assert module.enabled_sensors(settings) == ["c30d", "oak"]


def test_validate_settings_requires_positive_duration_and_one_sensor(tmp_path):
    module = load_record_script()
    settings = module.RunSettings(
        duration_s=1.0,
        enable_c30d=False,
        enable_rplidar=False,
        enable_oak=False,
        output_root=tmp_path,
    )

    with pytest.raises(ValueError, match="enable at least one sensor"):
        module.validate_settings(settings)

    with pytest.raises(ValueError, match="--duration"):
        module.validate_settings(
            module.RunSettings(
                duration_s=0.0,
                enable_c30d=True,
                enable_rplidar=False,
                enable_oak=False,
                output_root=tmp_path,
            )
        )


def test_create_run_folder_is_timestamped_and_unique(tmp_path):
    module = load_record_script()
    start_time = datetime(2026, 5, 28, 12, 34, 56, tzinfo=timezone.utc)

    first = module.create_run_folder(tmp_path, start_time)
    second = module.create_run_folder(tmp_path, start_time)

    assert first == tmp_path / "run_20260528_123456"
    assert second == tmp_path / "run_20260528_123456_01"
    assert first.is_dir()
    assert second.is_dir()


def test_metadata_for_run_contains_required_read_only_fields(tmp_path):
    module = load_record_script()
    start_time = datetime(2026, 5, 28, 12, 34, 56, tzinfo=timezone.utc)
    settings = module.RunSettings(
        duration_s=5.0,
        enable_c30d=True,
        enable_rplidar=True,
        enable_oak=True,
        output_root=tmp_path,
        c30d_port="/dev/mock-c30d",
        c30d_baud=115200,
        rplidar_port="/dev/mock-rplidar",
        rplidar_baud=460800,
        oak=module.OakCaptureSettings(fps=5.0, preview_width=640, preview_height=360),
    )

    metadata = module.metadata_for_run(settings, start_time)

    assert metadata["start_time"] == "2026-05-28T12:34:56+00:00"
    assert metadata["duration_s"] == 5.0
    assert metadata["enabled_sensors"] == ["c30d", "rplidar", "oak"]
    assert metadata["c30d"] == {
        "enabled": True,
        "port": "/dev/mock-c30d",
        "baud": 115200,
        "access": "read_only",
    }
    assert metadata["rplidar"]["port"] == "/dev/mock-rplidar"
    assert metadata["oak"]["rgb_output_dir"] == "oak_rgb"
    assert metadata["safety"] == {
        "note": "read-only sensor run; sends no motor or steering commands",
        "c30d_write": False,
        "motor_commands": False,
        "ros2": False,
    }


def test_c30d_output_paths_are_run_folder_files(tmp_path):
    module = load_record_script()

    feedback_path, odometry_path = module.c30d_output_paths(tmp_path)

    assert feedback_path == tmp_path / "c30d_feedback.csv"
    assert odometry_path == tmp_path / "c30d_odometry.csv"


def test_write_metadata_round_trips_yaml(tmp_path):
    module = load_record_script()
    metadata = {
        "start_time": "2026-05-28T12:34:56+00:00",
        "duration_s": 2.0,
        "enabled_sensors": ["c30d"],
    }

    path = module.write_metadata(tmp_path, metadata)

    assert path == tmp_path / "metadata.yaml"
    assert yaml.safe_load(path.read_text(encoding="utf-8")) == metadata
