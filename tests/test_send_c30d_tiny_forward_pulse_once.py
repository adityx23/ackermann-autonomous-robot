from __future__ import annotations

import csv
import importlib.util
import inspect
import sys
from dataclasses import dataclass

import pytest
from pathlib import Path

from ackermann_robot.drivers.c30d_checksum import compute_feedback_checksum, xor_checksum


def load_pulse_script():
    script_path = (
        Path(__file__).resolve().parents[1] / "scripts" / "send_c30d_tiny_forward_pulse_once.py"
    )
    spec = importlib.util.spec_from_file_location(script_path.stem, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[script_path.stem] = module
    spec.loader.exec_module(module)
    return module


def all_real_pulse_args() -> list[str]:
    return [
        "--armed",
        "--manual-enable",
        "--wheels-lifted",
        "--robot-restrained",
        "--manual-power-cutoff-ready",
        "--motor-enable-switch-reviewed",
        "--i-understand-this-may-spin-the-wheels",
        "--execute-real-pulse",
    ]


@dataclass(frozen=True)
class FakePreflight:
    invalid_checksum_count: int | None = 0
    candidate_battery_mV: float | int | None = 12000


@dataclass(frozen=True)
class FakeReadinessReport:
    readiness_allowed: bool
    preflight: FakePreflight = FakePreflight()
    warning_battery_mV: int = 10800
    reasons: tuple[str, ...] = ()


class FakeSerial:
    def __init__(self, read_chunks: list[bytes] | None = None) -> None:
        self.write_calls: list[bytes] = []
        self.read_chunks = list(read_chunks or [])
        self.flush_calls = 0
        self.close_calls = 0

    def write(self, data: bytes) -> int:
        self.write_calls.append(data)
        return len(data)

    def read(self, _size: int) -> bytes:
        if not self.read_chunks:
            return b""
        return self.read_chunks.pop(0)

    def flush(self) -> None:
        self.flush_calls += 1

    def close(self) -> None:
        self.close_calls += 1


class StepClock:
    def __init__(self) -> None:
        self.value = 0.0

    def __call__(self) -> float:
        return self.value

    def sleep(self, seconds: float) -> None:
        self.value += max(seconds, 1.0)


def feedback_frame(forward: int, yaw: int = 0, battery_mv: int = 12000) -> bytes:
    frame = bytearray([0x7B] + [0x00] * 22 + [0x7D])
    frame[2:4] = int(forward).to_bytes(2, "big", signed=True)
    frame[6:8] = int(yaw).to_bytes(2, "big", signed=True)
    frame[20:22] = int(battery_mv).to_bytes(2, "big", signed=False)
    frame[22] = compute_feedback_checksum(bytes(frame))
    return bytes(frame)


def test_default_pulse_reserved_bytes_are_zero():
    module = load_pulse_script()

    frames = module.build_pulse_frames(0.03, 0.10)

    assert frames.pulse_reserved_1 == 0x00
    assert frames.pulse_reserved_2 == 0x00
    assert frames.zero_frame.hex(" ") == "7b 00 00 00 00 00 00 00 00 7b 7d"
    assert frames.pulse_frame[1] == 0x00
    assert frames.pulse_frame[2] == 0x00


def test_reserved_2_changes_only_the_pulse_frame(capsys):
    module = load_pulse_script()
    fake_serial = FakeSerial()

    exit_code = module.main(
        ["--reserved-2", "0x01"],
        serial_factory=lambda _port, _baud: fake_serial,
    )

    output = capsys.readouterr().out
    frames = module.build_pulse_frames(0.03, 0.10, reserved_1=0, reserved_2=1)
    assert exit_code == 0
    assert frames.zero_frame.hex(" ") == "7b 00 00 00 00 00 00 00 00 7b 7d"
    assert frames.pulse_frame[1] == 0x00
    assert frames.pulse_frame[2] == 0x01
    assert "pulse_reserved_1: 0x00" in output
    assert "pulse_reserved_2: 0x01" in output
    assert "safe_zero_frame_hex: 7b 00 00 00 00 00 00 00 00 7b 7d" in output
    assert f"pulse_frame_hex: {frames.pulse_frame.hex(' ')}" in output
    assert "safe_zero_frame_hex: 7b 00 01" not in output
    assert fake_serial.write_calls == []


def test_reserved_bytes_above_one_are_rejected():
    module = load_pulse_script()

    with pytest.raises(SystemExit):
        module.main(["--reserved-1", "2"])
    with pytest.raises(SystemExit):
        module.main(["--reserved-2", "0x02"])
    with pytest.raises(ValueError, match="reserved_1_must_be_0x00_or_0x01"):
        module.build_pulse_frames(0.03, 0.10, reserved_1=2)
    with pytest.raises(ValueError, match="reserved_2_must_be_0x00_or_0x01"):
        module.build_pulse_frames(0.03, 0.10, reserved_2=2)


def test_generated_reserved_byte_frame_checksums_are_correct():
    module = load_pulse_script()

    frames = module.build_pulse_frames(0.05, 0.15, reserved_1=1, reserved_2=0)

    assert frames.zero_frame.hex(" ") == "7b 00 00 00 00 00 00 00 00 7b 7d"
    assert frames.zero_frame[9] == xor_checksum(frames.zero_frame[:9])
    assert frames.pulse_frame[9] == xor_checksum(frames.pulse_frame[:9])
    assert frames.zero_frame[5:7] == b"\x00\x00"
    assert frames.zero_frame[7:9] == b"\x00\x00"
    assert frames.pulse_frame[5:7] == b"\x00\x00"
    assert frames.pulse_frame[7:9] == b"\x00\x00"


def test_tiny_forward_pulse_dry_run_writes_nothing(capsys):
    module = load_pulse_script()
    fake_serial = FakeSerial()

    exit_code = module.main([], serial_factory=lambda _port, _baud: fake_serial)

    output = capsys.readouterr().out
    assert exit_code == 0
    assert "dry_run: true" in output
    assert "refused: execute_real_pulse_required_for_real_write" in output
    assert "safe_zero_frame_hex: 7b 00 00 00 00 00 00 00 00 7b 7d" in output
    assert "pulse_target_x_scaled_int16: 30" in output
    assert "real_write_performed: false" in output
    assert "bytes_written_total: 0" in output
    assert fake_serial.write_calls == []


def test_tiny_forward_pulse_refuses_without_execute_real_pulse(capsys):
    module = load_pulse_script()
    fake_serial = FakeSerial()
    args = [flag for flag in all_real_pulse_args() if flag != "--execute-real-pulse"]

    exit_code = module.main(args, serial_factory=lambda _port, _baud: fake_serial)

    output = capsys.readouterr().out
    assert exit_code == 0
    assert "refused: execute_real_pulse_required_for_real_write" in output
    assert "real_write_performed: false" in output
    assert fake_serial.write_calls == []


def test_tiny_forward_pulse_refuses_without_safety_flags(capsys):
    module = load_pulse_script()
    fake_serial = FakeSerial()

    exit_code = module.main(
        ["--execute-real-pulse"], serial_factory=lambda _port, _baud: fake_serial
    )

    output = capsys.readouterr().out
    assert exit_code == 1
    assert "refused: missing_required_safety_flags" in output
    assert "armed" in output
    assert "manual_enable" in output
    assert "real_write_performed: false" in output
    assert fake_serial.write_calls == []


def test_tiny_forward_pulse_dry_run_stream_mode_writes_nothing(capsys):
    module = load_pulse_script()
    fake_serial = FakeSerial()

    exit_code = module.main(
        ["--stream-mode"],
        serial_factory=lambda _port, _baud: fake_serial,
    )

    output = capsys.readouterr().out
    assert exit_code == 0
    assert "stream_mode: true" in output
    assert "stream_rate_hz: 20" in output
    assert "zero_stream_duration_s: 0.2" in output
    assert "pulse_stream_duration_s: 0.1" in output
    assert "stop_stream_duration_s: 0.3" in output
    assert "frames_written_by_phase: none" in output
    assert "bytes_written_total: 0" in output
    assert fake_serial.write_calls == []


def test_tiny_forward_pulse_refuses_target_x_above_limit(capsys):
    module = load_pulse_script()
    fake_serial = FakeSerial()

    exit_code = module.main(
        ["--target-x", "0.051"],
        serial_factory=lambda _port, _baud: fake_serial,
    )

    output = capsys.readouterr().out
    assert exit_code == 1
    assert "target_x_exceeds_0.05_limit" in output
    assert "real_write_performed: false" in output
    assert fake_serial.write_calls == []


def test_tiny_forward_pulse_rejects_extended_duration_without_explicit_stream_flag(capsys):
    module = load_pulse_script()
    fake_serial = FakeSerial()

    exit_code = module.main(
        ["--duration", "0.50"],
        serial_factory=lambda _port, _baud: fake_serial,
    )

    output = capsys.readouterr().out
    assert exit_code == 1
    assert "duration_exceeds_0.15_limit" in output
    assert fake_serial.write_calls == []


def test_tiny_forward_pulse_rejects_extended_duration_without_stream_mode(capsys):
    module = load_pulse_script()
    fake_serial = FakeSerial()

    exit_code = module.main(
        ["--allow-extended-low-speed-stream", "--duration", "0.50"],
        serial_factory=lambda _port, _baud: fake_serial,
    )

    output = capsys.readouterr().out
    assert exit_code == 1
    assert "duration_exceeds_0.15_limit" in output
    assert fake_serial.write_calls == []


def test_tiny_forward_pulse_accepts_extended_duration_only_with_stream_mode_and_flag(capsys):
    module = load_pulse_script()
    fake_serial = FakeSerial()

    exit_code = module.main(
        [
            "--stream-mode",
            "--allow-extended-low-speed-stream",
            "--duration",
            "0.50",
        ],
        serial_factory=lambda _port, _baud: fake_serial,
    )

    output = capsys.readouterr().out
    assert exit_code == 0
    assert "allow_extended_low_speed_stream: true" in output
    assert "extended_duration_limit_s: 0.5" in output
    assert "stream_mode: true" in output
    assert "pulse_stream_duration_s: 0.5" in output
    assert "dry_run: true" in output
    assert fake_serial.write_calls == []


def test_tiny_forward_pulse_rejects_duration_above_extended_limit(capsys):
    module = load_pulse_script()
    fake_serial = FakeSerial()

    exit_code = module.main(
        [
            "--stream-mode",
            "--allow-extended-low-speed-stream",
            "--duration",
            "0.501",
        ],
        serial_factory=lambda _port, _baud: fake_serial,
    )

    output = capsys.readouterr().out
    assert exit_code == 1
    assert "duration_exceeds_0.50_extended_stream_limit" in output
    assert fake_serial.write_calls == []


def test_tiny_forward_pulse_refuses_duration_above_limit(capsys):
    module = load_pulse_script()
    fake_serial = FakeSerial()

    exit_code = module.main(
        ["--duration", "0.151"],
        serial_factory=lambda _port, _baud: fake_serial,
    )

    output = capsys.readouterr().out
    assert exit_code == 1
    assert "duration_exceeds_0.15_limit" in output
    assert "real_write_performed: false" in output
    assert fake_serial.write_calls == []


def test_tiny_forward_pulse_refuses_when_readiness_false(monkeypatch, capsys):
    module = load_pulse_script()
    fake_serial = FakeSerial()
    monkeypatch.setattr(
        module,
        "run_internal_readiness",
        lambda _args: FakeReadinessReport(readiness_allowed=False),
    )

    exit_code = module.main(
        all_real_pulse_args(),
        serial_factory=lambda _port, _baud: fake_serial,
        sleep_fn=lambda _seconds: None,
    )

    output = capsys.readouterr().out
    assert exit_code == 1
    assert "readiness_attempt: 1" in output
    assert "readiness_attempt_allowed: false" in output
    assert "refused: readiness_attempts_exhausted" in output
    assert "real_write_performed: false" in output
    assert "bytes_written_total: 0" in output
    assert fake_serial.write_calls == []


def test_tiny_forward_pulse_default_c30d_warmup_duration_is_passed_to_readiness(
    monkeypatch, capsys
):
    module = load_pulse_script()
    fake_serial = FakeSerial()
    captured = {}

    def fake_readiness(args):
        captured["c30d_warmup_duration"] = args.c30d_warmup_duration
        return FakeReadinessReport(readiness_allowed=True)

    monkeypatch.setattr(module, "run_internal_readiness", fake_readiness)

    exit_code = module.main(
        all_real_pulse_args(),
        serial_factory=lambda _port, _baud: fake_serial,
        sleep_fn=lambda _seconds: None,
    )

    capsys.readouterr()
    assert exit_code == 0
    assert captured == {"c30d_warmup_duration": 1.0}


def test_tiny_forward_pulse_custom_c30d_warmup_duration_is_passed_to_readiness(monkeypatch, capsys):
    module = load_pulse_script()
    fake_serial = FakeSerial()
    captured = {}

    def fake_readiness(args):
        captured["c30d_warmup_duration"] = args.c30d_warmup_duration
        return FakeReadinessReport(readiness_allowed=True)

    monkeypatch.setattr(module, "run_internal_readiness", fake_readiness)

    exit_code = module.main(
        [*all_real_pulse_args(), "--c30d-warmup-duration", "1.5"],
        serial_factory=lambda _port, _baud: fake_serial,
        sleep_fn=lambda _seconds: None,
    )

    capsys.readouterr()
    assert exit_code == 0
    assert captured == {"c30d_warmup_duration": 1.5}


def test_tiny_forward_pulse_retries_readiness_then_writes_on_clean_attempt(monkeypatch, capsys):
    module = load_pulse_script()
    fake_serial = FakeSerial()
    clock = StepClock()
    reports = [
        FakeReadinessReport(
            readiness_allowed=False,
            preflight=FakePreflight(invalid_checksum_count=1, candidate_battery_mV=12000),
            reasons=("invalid_c30d_checksum_frames_observed",),
        ),
        FakeReadinessReport(readiness_allowed=True),
    ]
    readiness_calls = []

    def fake_readiness(_args):
        readiness_calls.append("called")
        return reports.pop(0)

    monkeypatch.setattr(module, "run_internal_readiness", fake_readiness)

    exit_code = module.main(
        [*all_real_pulse_args(), "--readiness-retries", "2", "--retry-delay", "0.25"],
        serial_factory=lambda _port, _baud: fake_serial,
        sleep_fn=clock.sleep,
        clock=clock,
    )

    output = capsys.readouterr().out
    assert exit_code == 0
    assert len(readiness_calls) == 2
    assert "readiness_attempt: 1" in output
    assert "readiness_attempt_invalid_checksum_count: 1" in output
    assert "readiness_attempt_write_allowed: false" in output
    assert "readiness_retry_delay_s: 0.25" in output
    assert "readiness_attempt: 2" in output
    assert "readiness_attempt_invalid_checksum_count: 0" in output
    assert "readiness_attempt_write_allowed: true" in output
    assert len(fake_serial.write_calls) == 4


def test_tiny_forward_pulse_refuses_after_all_readiness_retries_fail(monkeypatch, capsys):
    module = load_pulse_script()
    fake_serial = FakeSerial()
    reports = [
        FakeReadinessReport(
            readiness_allowed=False,
            preflight=FakePreflight(invalid_checksum_count=1, candidate_battery_mV=12000),
            reasons=("invalid_c30d_checksum_frames_observed",),
        ),
        FakeReadinessReport(
            readiness_allowed=False,
            preflight=FakePreflight(invalid_checksum_count=0, candidate_battery_mV=10799),
            reasons=("candidate_battery_below_warning_threshold",),
        ),
    ]
    readiness_calls = []

    def fake_readiness(_args):
        readiness_calls.append("called")
        return reports.pop(0)

    monkeypatch.setattr(module, "run_internal_readiness", fake_readiness)

    exit_code = module.main(
        [*all_real_pulse_args(), "--readiness-retries", "2", "--retry-delay", "0"],
        serial_factory=lambda _port, _baud: fake_serial,
        sleep_fn=lambda _seconds: None,
    )

    output = capsys.readouterr().out
    assert exit_code == 1
    assert len(readiness_calls) == 2
    assert "readiness_attempt: 1" in output
    assert "readiness_attempt_invalid_checksum_count: 1" in output
    assert "readiness_attempt_reasons: invalid_c30d_checksum_frames_observed" in output
    assert "readiness_attempt: 2" in output
    assert "readiness_attempt_battery_candidate_mV: 10799" in output
    assert "readiness_attempt_reasons: candidate_battery_below_warning_threshold" in output
    assert "refused: readiness_attempts_exhausted" in output
    assert "real_write_performed: false" in output
    assert "bytes_written_total: 0" in output
    assert fake_serial.write_calls == []


def test_tiny_forward_pulse_real_stream_mode_writes_zero_pulse_zero_frames(monkeypatch, capsys):
    module = load_pulse_script()
    fake_serial = FakeSerial()
    clock = StepClock()
    monkeypatch.setattr(
        module,
        "run_internal_readiness",
        lambda _args: FakeReadinessReport(readiness_allowed=True),
    )

    exit_code = module.main(
        [*all_real_pulse_args(), "--stream-mode", "--stream-rate-hz", "20"],
        serial_factory=lambda _port, _baud: fake_serial,
        sleep_fn=clock.sleep,
        clock=clock,
    )

    output = capsys.readouterr().out
    frames = module.build_pulse_frames(0.03, 0.10)
    assert exit_code == 0
    assert fake_serial.write_calls == (
        [frames.zero_frame] * 4 + [frames.pulse_frame] * 2 + [frames.zero_frame] * 6
    )
    assert fake_serial.flush_calls == 12
    assert fake_serial.close_calls == 1
    assert "stream_mode: true" in output
    assert (
        "frames_written_by_phase: zero_before_stream=4, pulse_stream=2, zero_after_stream=6"
        in output
    )
    assert "bytes_written_total: 132" in output


def test_tiny_forward_pulse_refuses_stream_rate_above_limit(capsys):
    module = load_pulse_script()
    fake_serial = FakeSerial()

    exit_code = module.main(
        ["--stream-mode", "--stream-rate-hz", "50.1"],
        serial_factory=lambda _port, _baud: fake_serial,
    )

    output = capsys.readouterr().out
    assert exit_code == 1
    assert "stream_rate_hz_exceeds_50_limit" in output
    assert "real_write_performed: false" in output
    assert fake_serial.write_calls == []


def test_tiny_forward_pulse_sends_safe_zero_pulse_safe_zero_safe_zero_when_armed(
    monkeypatch, capsys
):
    module = load_pulse_script()
    fake_serial = FakeSerial()
    factory_calls = []
    clock = StepClock()

    def fake_serial_factory(port: str, baud: int) -> FakeSerial:
        factory_calls.append((port, baud))
        return fake_serial

    monkeypatch.setattr(
        module,
        "run_internal_readiness",
        lambda _args: FakeReadinessReport(readiness_allowed=True),
    )

    exit_code = module.main(
        [
            *all_real_pulse_args(),
            "--reserved-2",
            "0x01",
            "--port",
            "/tmp/c30d",
            "--baud",
            "57600",
        ],
        serial_factory=fake_serial_factory,
        sleep_fn=clock.sleep,
        clock=clock,
    )

    output = capsys.readouterr().out
    zero_frame = module.build_pulse_frames(0.03, 0.10, reserved_2=1).zero_frame
    pulse_frame = module.build_pulse_frames(0.03, 0.10, reserved_2=1).pulse_frame
    assert exit_code == 0
    assert factory_calls == [("/tmp/c30d", 57600)]
    assert fake_serial.write_calls == [zero_frame, pulse_frame, zero_frame, zero_frame]
    assert [len(frame) for frame in fake_serial.write_calls] == [11, 11, 11, 11]
    assert fake_serial.flush_calls == 4
    assert fake_serial.close_calls == 1
    assert zero_frame.hex(" ") == "7b 00 00 00 00 00 00 00 00 7b 7d"
    assert pulse_frame[2] == 0x01
    assert "frame_hex: 7b 00 00 00 00 00 00 00 00 7b 7d" in output
    assert f"frame_hex: {pulse_frame.hex(' ')}" in output
    assert "real_write_performed: true" in output
    assert "bytes_written_total: 44" in output
    assert "pulse_target_x: 0.03" in output
    assert "pulse_duration_s: 0.1" in output
    assert "warning: wheels may spin briefly" in output


def test_tiny_forward_pulse_stream_feedback_csv_rows_include_stream_phase_labels(
    monkeypatch, tmp_path: Path
):
    module = load_pulse_script()
    read_chunks = [
        feedback_frame(0),
        feedback_frame(0),
        feedback_frame(0),
        feedback_frame(7),
        feedback_frame(1),
        feedback_frame(1),
        feedback_frame(1),
        feedback_frame(0),
    ]
    fake_serial = FakeSerial(read_chunks=read_chunks)
    clock = StepClock()
    output_path = tmp_path / "stream_feedback.csv"
    monkeypatch.setattr(
        module,
        "run_internal_readiness",
        lambda _args: FakeReadinessReport(readiness_allowed=True),
    )

    exit_code = module.main(
        [
            *all_real_pulse_args(),
            "--stream-mode",
            "--stream-rate-hz",
            "10",
            "--feedback-output",
            str(output_path),
        ],
        serial_factory=lambda _port, _baud: fake_serial,
        sleep_fn=clock.sleep,
        clock=clock,
    )

    assert exit_code == 0
    rows = list(csv.DictReader(output_path.open(encoding="utf-8")))
    assert [row["phase"] for row in rows] == [
        "baseline",
        "zero_before_stream",
        "zero_before_stream",
        "pulse_stream",
        "zero_after_stream",
        "zero_after_stream",
        "zero_after_stream",
        "post",
    ]


def test_tiny_forward_pulse_feedback_csv_rows_include_phase_labels(monkeypatch, tmp_path: Path):
    module = load_pulse_script()
    read_chunks = [
        feedback_frame(0),
        feedback_frame(0),
        feedback_frame(7),
        feedback_frame(2),
        feedback_frame(1),
    ]
    fake_serial = FakeSerial(read_chunks=read_chunks)
    clock = StepClock()
    output_path = tmp_path / "pulse_feedback.csv"
    monkeypatch.setattr(
        module,
        "run_internal_readiness",
        lambda _args: FakeReadinessReport(readiness_allowed=True),
    )

    exit_code = module.main(
        [
            *all_real_pulse_args(),
            "--reserved-1",
            "1",
            "--reserved-2",
            "0x01",
            "--feedback-output",
            str(output_path),
        ],
        serial_factory=lambda _port, _baud: fake_serial,
        sleep_fn=clock.sleep,
        clock=clock,
    )

    assert exit_code == 0
    rows = list(csv.DictReader(output_path.open(encoding="utf-8")))
    assert [row["phase"] for row in rows] == [
        "baseline",
        "zero_before",
        "pulse",
        "zero_after",
        "post",
    ]
    assert set(rows[0]) == {
        "monotonic_timestamp",
        "phase",
        "frame_index",
        "forward_candidate",
        "yaw_candidate",
        "candidate_battery_mV",
        "checksum_valid",
        "raw_frame_hex",
    }


def test_tiny_forward_pulse_movement_detection_summary_on_synthetic_feedback():
    module = load_pulse_script()
    rows = [
        module.FeedbackLogRow(0.0, "baseline", 0, 1, 0, 12000, True, "baseline"),
        module.FeedbackLogRow(0.1, "pulse", 1, 5, -3, 12000, True, "pulse"),
        module.FeedbackLogRow(0.2, "post", 2, 2, 4, 12000, False, "post"),
    ]

    summary = module.summarize_feedback(rows)

    assert summary.max_abs_forward_candidate_baseline == 1
    assert summary.max_abs_forward_candidate_pulse_post == 5
    assert summary.max_abs_yaw_candidate == 4
    assert summary.invalid_checksum_count == 1
    assert summary.movement_feedback_detected is True


def test_tiny_forward_pulse_has_no_ros_or_ros2_imports():
    module = load_pulse_script()
    source = inspect.getsource(module)

    assert "import rospy" not in source
    assert "import rclpy" not in source
    assert "ros2" not in source.lower()
    assert "roslaunch" not in source
    assert "rostopic" not in source
