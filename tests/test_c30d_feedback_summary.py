from __future__ import annotations

import csv
import importlib.util
import math
import sys
from pathlib import Path


def load_summary_script():
    script_path = (
        Path(__file__).resolve().parents[1] / "scripts" / "summarize_c30d_feedback_candidates.py"
    )
    spec = importlib.util.spec_from_file_location(script_path.stem, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[script_path.stem] = module
    spec.loader.exec_module(module)
    return module


def write_feedback_csv(path: Path, rows: list[dict[str, int]]) -> None:
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
    with path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for index, row in enumerate(rows):
            writer.writerow(
                {
                    "frame_index": index,
                    "candidate_forward_motion": row["candidate_forward_motion"],
                    "candidate_yaw_motion": row["candidate_yaw_motion"],
                    "candidate_imu_12_13": row["candidate_imu_12_13"],
                    "candidate_imu_14_15": row["candidate_imu_14_15"],
                    "candidate_imu_16_17": row["candidate_imu_16_17"],
                    "candidate_imu_18_19": row["candidate_imu_18_19"],
                    "checksum_candidate": 0,
                    "raw_frame_hex": "7b 00 7d",
                }
            )


def test_summarize_csv_computes_candidate_stats(tmp_path):
    module = load_summary_script()
    csv_path = tmp_path / "feedback.csv"
    write_feedback_csv(
        csv_path,
        [
            {
                "candidate_forward_motion": 10,
                "candidate_yaw_motion": -5,
                "candidate_imu_12_13": 0,
                "candidate_imu_14_15": 1,
                "candidate_imu_16_17": -1,
                "candidate_imu_18_19": 100,
            },
            {
                "candidate_forward_motion": 20,
                "candidate_yaw_motion": 5,
                "candidate_imu_12_13": 3,
                "candidate_imu_14_15": 1,
                "candidate_imu_16_17": -1,
                "candidate_imu_18_19": -100,
            },
            {
                "candidate_forward_motion": -5,
                "candidate_yaw_motion": 0,
                "candidate_imu_12_13": -3,
                "candidate_imu_14_15": 1,
                "candidate_imu_16_17": 0,
                "candidate_imu_18_19": 0,
            },
        ],
    )

    summary = module.summarize_csv(csv_path)

    assert summary.row_count == 3
    forward = summary.stats_by_field["candidate_forward_motion"]
    assert forward.minimum == -5
    assert forward.maximum == 20
    assert forward.mean == 25 / 3
    assert forward.total == 25
    assert forward.sum_abs == 35
    assert forward.count_nonzero == 3
    assert math.isclose(forward.stdev, 10.274023338281626)

    imu_16_17 = summary.stats_by_field["candidate_imu_16_17"]
    assert imu_16_17.total == -2
    assert imu_16_17.sum_abs == 2
    assert imu_16_17.count_nonzero == 2


def test_main_prints_sample_rate_and_calibration_helpers(capsys, tmp_path):
    module = load_summary_script()
    csv_path = tmp_path / "feedback.csv"
    write_feedback_csv(
        csv_path,
        [
            {
                "candidate_forward_motion": 10,
                "candidate_yaw_motion": 5,
                "candidate_imu_12_13": 0,
                "candidate_imu_14_15": 0,
                "candidate_imu_16_17": 0,
                "candidate_imu_18_19": 0,
            },
            {
                "candidate_forward_motion": 30,
                "candidate_yaw_motion": 5,
                "candidate_imu_12_13": 0,
                "candidate_imu_14_15": 0,
                "candidate_imu_16_17": 0,
                "candidate_imu_18_19": 0,
            },
        ],
    )

    result = module.main(
        [
            str(csv_path),
            "--duration-s",
            "0.5",
            "--known-distance-m",
            "2.0",
            "--known-yaw-deg",
            "90",
        ]
    )

    output = capsys.readouterr().out
    assert result == 0
    assert "Read-only C30D candidate feedback CSV summary." in output
    assert f"csv_path: {csv_path}" in output
    assert "row_count: 2" in output
    assert "sample_rate_hz: 4" in output
    assert "candidate_forward_motion: min=10 max=30 mean=20 stdev=10 sum=40" in output
    assert "sum_abs=40 count_nonzero=2" in output
    assert "meters_per_forward_sum: 0.05" in output
    assert "radians_per_yaw_sum: 0.15708" in output


def test_main_rejects_missing_required_column(capsys, tmp_path):
    module = load_summary_script()
    csv_path = tmp_path / "missing.csv"
    csv_path.write_text("candidate_forward_motion\n1\n", encoding="utf-8")

    result = module.main([str(csv_path)])

    captured = capsys.readouterr()
    assert result == 1
    assert "missing required columns" in captured.err
