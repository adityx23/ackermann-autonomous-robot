from __future__ import annotations

import importlib.util
import sys
from datetime import datetime
from pathlib import Path

from ackermann_robot.drivers.c30d_checksum import compute_feedback_checksum
from ackermann_robot.drivers.c30d_frames import FRAME_LENGTH


def load_check_script():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "check_robot_sensors.py"
    spec = importlib.util.spec_from_file_location(script_path.stem, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[script_path.stem] = module
    spec.loader.exec_module(module)
    return module


def make_feedback_frame(*, checksum_valid: bool = True, battery_mv: int = 12000) -> bytes:
    frame = bytearray([0x7B, *([0x00] * (FRAME_LENGTH - 2)), 0x7D])
    frame[20:22] = int(battery_mv).to_bytes(2, "big", signed=False)
    frame[22] = compute_feedback_checksum(frame)
    if not checksum_valid:
        frame[22] ^= 0x01
    return bytes(frame)


class StepClock:
    def __init__(self, step_s: float = 0.6) -> None:
        self.value = 0.0
        self.step_s = step_s

    def __call__(self) -> float:
        current = self.value
        self.value += self.step_s
        return current


def run_fake_c30d_check(module, monkeypatch, chunks: list[bytes]):
    import monitor_c30d_feedback_readonly

    read_chunks = list(chunks)
    monkeypatch.setattr(
        monitor_c30d_feedback_readonly, "open_readonly_serial_fd", lambda *_args: 99
    )
    monkeypatch.setattr(module.os, "close", lambda _fd: None)
    monkeypatch.setattr(
        module.os, "read", lambda _fd, _size: read_chunks.pop(0) if read_chunks else b""
    )
    monkeypatch.setattr(module.time, "monotonic", StepClock())
    return module.check_c30d_readonly(
        "/tmp/c30d",
        1.0,
        min_frame_rate_hz=0.1,
        warmup_duration_s=1.0,
        battery_config=module.BatterySafetyConfig(),
    )


def test_parser_defaults_enable_all_checks():
    module = load_check_script()

    args = module.build_parser().parse_args([])

    assert args.check_c30d is True
    assert args.check_rplidar is True
    assert args.check_oak is True
    assert args.duration == 3.0
    assert args.c30d_port == "/dev/c30d"
    assert args.c30d_warmup_duration == 0.0
    assert args.rplidar_port == "/dev/rplidar"


def test_parser_can_disable_individual_checks():
    module = load_check_script()

    args = module.build_parser().parse_args(
        [
            "--no-check-c30d",
            "--no-check-rplidar",
            "--no-check-oak",
            "--duration",
            "1.5",
            "--c30d-port",
            "/tmp/c30d",
            "--c30d-warmup-duration",
            "0.5",
            "--rplidar-port",
            "/tmp/rplidar",
        ]
    )

    assert args.check_c30d is False
    assert args.check_rplidar is False
    assert args.check_oak is False
    assert args.duration == 1.5
    assert args.c30d_port == "/tmp/c30d"
    assert args.c30d_warmup_duration == 0.5
    assert args.rplidar_port == "/tmp/rplidar"


def test_validate_args_rejects_nonpositive_duration():
    module = load_check_script()
    args = module.build_parser().parse_args(["--duration", "0"])

    try:
        module.validate_args(args)
    except ValueError as exc:
        assert "--duration" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_validate_args_rejects_negative_c30d_warmup_duration():
    module = load_check_script()
    args = module.build_parser().parse_args(["--c30d-warmup-duration", "-0.1"])

    try:
        module.validate_args(args)
    except ValueError as exc:
        assert "--c30d-warmup-duration" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_invalid_checksum_during_c30d_warmup_is_not_counted(monkeypatch):
    module = load_check_script()

    result = run_fake_c30d_check(
        module,
        monkeypatch,
        [
            make_feedback_frame(checksum_valid=False),
            make_feedback_frame(checksum_valid=True),
        ],
    )

    assert result.passed is True
    assert result.details["c30d_warmup_duration_s"] == 1.0
    assert result.details["warmup_frame_count"] == 1
    assert result.details["counted_frame_count"] == 1
    assert result.details["invalid_checksum_count"] == 0
    assert result.details["counted_invalid_checksum_count"] == 0


def test_invalid_checksum_during_counted_c30d_window_is_counted(monkeypatch):
    module = load_check_script()

    result = run_fake_c30d_check(
        module,
        monkeypatch,
        [
            make_feedback_frame(checksum_valid=True),
            make_feedback_frame(checksum_valid=False),
        ],
    )

    assert result.passed is False
    assert result.details["warmup_frame_count"] == 1
    assert result.details["counted_frame_count"] == 1
    assert result.details["invalid_checksum_count"] == 1
    assert result.details["counted_invalid_checksum_count"] == 1


def test_frame_rate_handles_elapsed_time():
    module = load_check_script()

    assert module.frame_rate(30, 3.0) == 10.0
    assert module.frame_rate(30, 0.0) == 0.0


def test_invalid_checksum_percentage_handles_zero_frames():
    module = load_check_script()

    assert module.invalid_checksum_percentage(0, 3) == 0.0
    assert module.invalid_checksum_percentage(100, 2) == 2.0


def test_battery_threshold_helpers_warn_and_block():
    module = load_check_script()
    config = module.BatterySafetyConfig(
        warn_battery_mV=10800,
        block_motor_test_battery_mV=10500,
        critical_battery_mV=10200,
    )

    warnings, reasons = module.battery_warnings_and_reasons(10750, config)
    assert warnings == ["candidate_battery_below_warning_threshold"]
    assert reasons == []

    warnings, reasons = module.battery_warnings_and_reasons(10499, config)
    assert warnings == []
    assert reasons == ["candidate_battery_below_motor_test_block_threshold"]

    warnings, reasons = module.battery_warnings_and_reasons(10199, config)
    assert warnings == []
    assert reasons == [
        "candidate_battery_below_critical_threshold",
        "candidate_battery_below_motor_test_block_threshold",
    ]


def test_valid_lidar_point_count_counts_positive_distances_only():
    module = load_check_script()

    class Point:
        def __init__(self, distance_mm: float) -> None:
            self.distance_mm = distance_mm

    points = [Point(100.0), Point(0.0), Point(-1.0), Point(50.0)]

    assert module.valid_lidar_point_count(points) == 2


def test_timestamped_path_uses_expected_name():
    module = load_check_script()

    path = module.timestamped_path(
        Path("data/preflight"),
        "rplidar_preflight",
        ".csv",
        datetime(2026, 5, 28, 12, 34, 56),
    )

    assert path == Path("data/preflight/rplidar_preflight_20260528_123456.csv")


def test_path_and_directory_checks_are_pure_filesystem_checks(tmp_path: Path):
    module = load_check_script()
    existing_file = tmp_path / "device"
    existing_file.write_text("", encoding="utf-8")
    existing_dir = tmp_path / "runs"
    existing_dir.mkdir()

    file_result = module.check_path_exists("device", existing_file)
    dir_result = module.check_directory("runs", existing_dir)
    missing_result = module.check_path_exists("missing", tmp_path / "missing")

    assert file_result.passed is True
    assert dir_result.passed is True
    assert missing_result.passed is False
    assert module.status_text(True) == "PASS"
    assert module.status_text(False) == "FAIL"
