from __future__ import annotations

import argparse
import importlib.util
import math
import sys
from datetime import datetime
from pathlib import Path

import pytest

from ackermann_robot.drivers.c30d_feedback import C30DFeedbackCandidate


def load_monitor_script():
    script_path = (
        Path(__file__).resolve().parents[1] / "scripts" / "monitor_c30d_odometry_readonly.py"
    )
    spec = importlib.util.spec_from_file_location(script_path.stem, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[script_path.stem] = module
    spec.loader.exec_module(module)
    return module


def make_candidate(frame_index: int, forward: int, yaw: int) -> C30DFeedbackCandidate:
    return C30DFeedbackCandidate(
        frame_index=frame_index,
        candidate_forward_motion=forward,
        candidate_yaw_motion=yaw,
        candidate_imu_12_13=0,
        candidate_imu_14_15=0,
        candidate_imu_16_17=0,
        candidate_imu_18_19=0,
        checksum_candidate=0,
        raw_frame_hex="7b 00 7d",
    )


def test_update_live_odometry_straight_only_accumulates_x_and_zeros_yaw():
    module = load_monitor_script()
    state = module.LiveOdometryState()

    state, first = module.update_live_odometry(
        candidate=make_candidate(frame_index=0, forward=10, yaw=99),
        state=state,
        forward_m_per_count=0.1,
        mode="straight_only",
    )
    state, second = module.update_live_odometry(
        candidate=make_candidate(frame_index=1, forward=-4, yaw=-20),
        state=state,
        forward_m_per_count=0.1,
        mode="straight_only",
    )

    assert first == module.LiveOdometrySample(
        frame_index=0,
        forward_candidate=10,
        yaw_candidate=0,
        delta_s_m=1.0,
        x_m=1.0,
        y_m=0.0,
        theta_rad=0.0,
    )
    assert second == module.LiveOdometrySample(
        frame_index=1,
        forward_candidate=-4,
        yaw_candidate=0,
        delta_s_m=-0.4,
        x_m=0.6,
        y_m=0.0,
        theta_rad=0.0,
    )


def test_update_live_odometry_raw_yaw_candidate_preserves_yaw_counts():
    module = load_monitor_script()
    state = module.LiveOdometryState()

    _, sample = module.update_live_odometry(
        candidate=make_candidate(frame_index=7, forward=3, yaw=-8),
        state=state,
        forward_m_per_count=0.25,
        mode="raw_yaw_candidate",
    )

    assert sample.frame_index == 7
    assert sample.forward_candidate == 3
    assert sample.yaw_candidate == -8
    assert sample.delta_s_m == 0.75
    assert sample.x_m == 0.75
    assert sample.y_m == 0.0
    assert sample.theta_rad == 0.0


def test_update_live_odometry_uses_existing_theta_without_changing_it():
    module = load_monitor_script()
    state = module.LiveOdometryState(x_m=1.0, y_m=2.0, theta_rad=math.pi / 2)

    next_state, sample = module.update_live_odometry(
        candidate=make_candidate(frame_index=2, forward=4, yaw=6),
        state=state,
        forward_m_per_count=0.5,
        mode="raw_yaw_candidate",
    )

    assert math.isclose(sample.x_m, 1.0)
    assert math.isclose(sample.y_m, 4.0)
    assert sample.theta_rad == math.pi / 2
    assert next_state.theta_rad == math.pi / 2


def test_format_live_odometry_line_is_compact_and_provisional_fields_are_named():
    module = load_monitor_script()
    sample = module.LiveOdometrySample(
        frame_index=3,
        forward_candidate=12,
        yaw_candidate=-2,
        delta_s_m=0.00123456,
        x_m=0.25,
        y_m=0.0,
        theta_rad=0.0,
    )

    line = module.format_live_odometry_line(sample)

    assert "frame_index=3" in line
    assert "forward_candidate=12" in line
    assert "yaw_candidate=-2" in line
    assert "delta_s_m=0.00123456" in line
    assert "x_m=0.25" in line
    assert "theta_rad=0" in line


def test_resolve_output_path_places_csv_under_live_data_dir():
    module = load_monitor_script()

    assert module.resolve_output_path(Path("/tmp/custom_name"), datetime(2026, 5, 28, 1, 2, 3)) == Path(
        "data/c30d_live/custom_name.csv"
    )


def test_validate_args_rejects_non_positive_values():
    module = load_monitor_script()

    with pytest.raises(ValueError, match="--duration"):
        module.validate_args(argparse.Namespace(duration=0.0, print_every=1))

    with pytest.raises(ValueError, match="--print-every"):
        module.validate_args(argparse.Namespace(duration=1.0, print_every=0))
