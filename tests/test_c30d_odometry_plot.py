from __future__ import annotations

import csv
import importlib.util
import math
import sys
from pathlib import Path

import pytest


def load_plot_script():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "plot_c30d_odometry.py"
    spec = importlib.util.spec_from_file_location(script_path.stem, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[script_path.stem] = module
    spec.loader.exec_module(module)
    return module


def write_odometry_csv(path: Path) -> None:
    with path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "frame_index",
                "delta_s_m",
                "yaw_candidate",
                "x_m",
                "y_m",
                "theta_rad",
            ],
        )
        writer.writeheader()
        writer.writerows(
            [
                {
                    "frame_index": 10,
                    "delta_s_m": 0.1,
                    "yaw_candidate": 0,
                    "x_m": 0.1,
                    "y_m": 0.0,
                    "theta_rad": 0.0,
                },
                {
                    "frame_index": 11,
                    "delta_s_m": 0.2,
                    "yaw_candidate": 5,
                    "x_m": 0.3,
                    "y_m": 0.0,
                    "theta_rad": 0.0,
                },
            ]
        )


def test_load_odometry_csv_reads_required_plot_fields(tmp_path):
    module = load_plot_script()
    csv_path = tmp_path / "odometry.csv"
    write_odometry_csv(csv_path)

    series = module.load_odometry_csv(csv_path)

    assert series.csv_path == csv_path
    assert series.samples == [
        module.OdometrySample(frame_index=10, x_m=0.1, y_m=0.0, theta_rad=0.0),
        module.OdometrySample(frame_index=11, x_m=0.3, y_m=0.0, theta_rad=0.0),
    ]
    assert module.final_pose(series) == (0.3, 0.0, 0.0)


def test_load_odometry_csv_rejects_missing_required_column(tmp_path):
    module = load_plot_script()
    csv_path = tmp_path / "missing.csv"
    csv_path.write_text("frame_index,x_m,y_m\n1,0.1,0.0\n", encoding="utf-8")

    with pytest.raises(ValueError, match="missing required columns: theta_rad"):
        module.load_odometry_csv(csv_path)


def test_load_odometry_csv_rejects_invalid_numeric_value(tmp_path):
    module = load_plot_script()
    csv_path = tmp_path / "invalid.csv"
    csv_path.write_text(
        "frame_index,x_m,y_m,theta_rad\n1,not-a-number,0.0,0.0\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="has invalid odometry data"):
        module.load_odometry_csv(csv_path)


def test_final_pose_returns_nan_for_empty_series(tmp_path):
    module = load_plot_script()
    series = module.OdometrySeries(csv_path=tmp_path / "empty.csv", samples=[])

    x_m, y_m, theta_rad = module.final_pose(series)

    assert math.isnan(x_m)
    assert math.isnan(y_m)
    assert math.isnan(theta_rad)


def test_output_paths_use_analysis_plot_names(tmp_path):
    module = load_plot_script()

    xy_path, x_over_frame_path = module.output_paths(tmp_path)

    assert xy_path == tmp_path / "c30d_odometry_xy.png"
    assert x_over_frame_path == tmp_path / "c30d_odometry_x_over_frame.png"
