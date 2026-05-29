from __future__ import annotations

import csv
import importlib.util
import sys
from pathlib import Path

import pytest

from ackermann_robot.odometry.c30d_dead_reckoning import (
    C30DCalibration,
    load_c30d_calibration,
    load_feedback_candidate_csv,
    output_path_for,
    replay_dead_reckoning,
    write_odometry_csv,
)


def load_replay_script():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "replay_c30d_odometry.py"
    spec = importlib.util.spec_from_file_location(script_path.stem, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[script_path.stem] = module
    spec.loader.exec_module(module)
    return module


def write_calibration(path: Path, forward_m_per_count: float = 0.01) -> None:
    path.write_text(
        f"""
c30d:
  frame_length: 24
  sample_rate_hz: 20.0
  forward_m_per_count: {forward_m_per_count}
  yaw_rad_per_count: null
""",
        encoding="utf-8",
    )


def write_feedback_csv(path: Path) -> None:
    fieldnames = [
        "frame_index",
        "candidate_forward_motion",
        "candidate_yaw_motion",
        "candidate_imu_12_13",
        "candidate_imu_14_15",
        "candidate_imu_16_17",
        "candidate_imu_18_19",
        "checksum_candidate",
        "raw_frame_hex",
    ]
    rows = [
        {"frame_index": 5, "candidate_forward_motion": 10, "candidate_yaw_motion": 3},
        {"frame_index": 6, "candidate_forward_motion": 20, "candidate_yaw_motion": -4},
        {"frame_index": 7, "candidate_forward_motion": -5, "candidate_yaw_motion": 0},
    ]
    with path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    **row,
                    "candidate_imu_12_13": 0,
                    "candidate_imu_14_15": 0,
                    "candidate_imu_16_17": 0,
                    "candidate_imu_18_19": 0,
                    "checksum_candidate": 0,
                    "raw_frame_hex": "7b 00 7d",
                }
            )


def test_load_c30d_calibration_marks_values_provisional(tmp_path):
    config_path = tmp_path / "c30d_calibration.yaml"
    write_calibration(config_path, forward_m_per_count=0.25)

    calibration = load_c30d_calibration(config_path)

    assert calibration.forward_m_per_count == 0.25
    assert calibration.yaw_rad_per_count is None
    assert calibration.sample_rate_hz == 20.0
    assert calibration.provisional is True


def test_load_feedback_candidate_csv_reads_only_required_columns(tmp_path):
    csv_path = tmp_path / "feedback.csv"
    write_feedback_csv(csv_path)

    rows = load_feedback_candidate_csv(csv_path)

    assert rows == [
        {"frame_index": 5, "candidate_forward_motion": 10, "candidate_yaw_motion": 3},
        {"frame_index": 6, "candidate_forward_motion": 20, "candidate_yaw_motion": -4},
        {"frame_index": 7, "candidate_forward_motion": -5, "candidate_yaw_motion": 0},
    ]


def test_replay_dead_reckoning_straight_only_zeros_yaw_candidate():
    rows = [
        {"frame_index": 0, "candidate_forward_motion": 10, "candidate_yaw_motion": 99},
        {"frame_index": 1, "candidate_forward_motion": -4, "candidate_yaw_motion": -25},
    ]
    calibration = C30DCalibration(
        forward_m_per_count=0.1,
        yaw_rad_per_count=None,
        sample_rate_hz=None,
    )

    samples = replay_dead_reckoning(rows, calibration, "straight_only")

    assert [sample.frame_index for sample in samples] == [0, 1]
    assert [sample.delta_s_m for sample in samples] == [1.0, -0.4]
    assert [sample.yaw_candidate for sample in samples] == [0, 0]
    assert [sample.x_m for sample in samples] == [1.0, 0.6]
    assert [sample.y_m for sample in samples] == [0.0, 0.0]
    assert [sample.theta_rad for sample in samples] == [0.0, 0.0]


def test_replay_dead_reckoning_raw_yaw_candidate_preserves_counts():
    rows = [
        {"frame_index": 0, "candidate_forward_motion": 2, "candidate_yaw_motion": 7},
        {"frame_index": 1, "candidate_forward_motion": 3, "candidate_yaw_motion": -8},
    ]
    calibration = C30DCalibration(
        forward_m_per_count=0.5,
        yaw_rad_per_count=None,
        sample_rate_hz=None,
    )

    samples = replay_dead_reckoning(rows, calibration, "raw_yaw_candidate")

    assert [sample.delta_s_m for sample in samples] == [1.0, 1.5]
    assert [sample.yaw_candidate for sample in samples] == [7, -8]
    assert [sample.theta_rad for sample in samples] == [0.0, 0.0]


def test_replay_dead_reckoning_rejects_calibrated_yaw_until_implemented():
    rows = [{"frame_index": 0, "candidate_forward_motion": 1, "candidate_yaw_motion": 1}]
    calibration = C30DCalibration(
        forward_m_per_count=0.1,
        yaw_rad_per_count=0.01,
        sample_rate_hz=None,
    )

    with pytest.raises(ValueError, match="calibrated C30D yaw odometry is not implemented"):
        replay_dead_reckoning(rows, calibration, "straight_only")


def test_write_odometry_csv_uses_required_columns(tmp_path):
    rows = [{"frame_index": 0, "candidate_forward_motion": 10, "candidate_yaw_motion": 5}]
    calibration = C30DCalibration(
        forward_m_per_count=0.01,
        yaw_rad_per_count=None,
        sample_rate_hz=None,
    )
    samples = replay_dead_reckoning(rows, calibration, "raw_yaw_candidate")
    output_path = tmp_path / "odometry.csv"

    write_odometry_csv(samples, output_path)

    with output_path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        assert reader.fieldnames == [
            "frame_index",
            "delta_s_m",
            "yaw_candidate",
            "x_m",
            "y_m",
            "theta_rad",
        ]
        assert list(reader) == [
            {
                "frame_index": "0",
                "delta_s_m": "0.1",
                "yaw_candidate": "5",
                "x_m": "0.1",
                "y_m": "0",
                "theta_rad": "0",
            }
        ]


def test_output_path_includes_mode():
    assert output_path_for(
        "data/c30d_analysis/run_feedback_candidates.csv", "out", "straight_only"
    ) == Path("out/run_feedback_candidates_odometry_straight_only.csv")


def test_replay_script_writes_offline_odometry_csv(capsys, tmp_path):
    module = load_replay_script()
    input_csv = tmp_path / "feedback.csv"
    config_path = tmp_path / "c30d_calibration.yaml"
    output_dir = tmp_path / "analysis"
    write_feedback_csv(input_csv)
    write_calibration(config_path, forward_m_per_count=0.01)

    result = module.main(
        [
            str(input_csv),
            "--config",
            str(config_path),
            "--mode",
            "raw_yaw_candidate",
            "--output-dir",
            str(output_dir),
        ]
    )

    output = capsys.readouterr().out
    output_path = output_dir / "feedback_odometry_raw_yaw_candidate.csv"
    assert result == 0
    assert "Provisional read-only C30D dead-reckoning replay" in output
    assert "yaw_calibration: unavailable" in output
    assert f"output_path: {output_path}" in output
    assert "row_count: 3" in output
    assert output_path.exists()
