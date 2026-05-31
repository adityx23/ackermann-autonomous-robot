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
class FakeReadinessReport:
    readiness_allowed: bool


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


def test_reserved_bytes_default_to_zero():
    module = load_pulse_script()

    frames = module.build_pulse_frames(0.03, 0.10)

    assert frames.reserved_1 == 0x00
    assert frames.reserved_2 == 0x00
    assert frames.zero_frame[1] == 0x00
    assert frames.zero_frame[2] == 0x00
    assert frames.pulse_frame[1] == 0x00
    assert frames.pulse_frame[2] == 0x00


def test_reserved_bytes_zero_and_one_are_accepted(capsys):
    module = load_pulse_script()
    fake_serial = FakeSerial()

    exit_code = module.main(
        ["--reserved-1", "0x01", "--reserved-2", "1"],
        serial_factory=lambda _port, _baud: fake_serial,
    )

    output = capsys.readouterr().out
    frames = module.build_pulse_frames(0.03, 0.10, reserved_1=1, reserved_2=1)
    assert exit_code == 0
    assert "reserved_1: 0x01" in output
    assert "reserved_2: 0x01" in output
    assert f"zero_frame_hex: {frames.zero_frame.hex(' ')}" in output
    assert f"pulse_frame_hex: {frames.pulse_frame.hex(' ')}" in output
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
    assert "zero_frame_hex: 7b 00 00 00 00 00 00 00 00 7b 7d" in output
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
    assert "refused: readiness_allowed_false" in output
    assert "real_write_performed: false" in output
    assert "bytes_written_total: 0" in output
    assert fake_serial.write_calls == []


def test_tiny_forward_pulse_sends_zero_pulse_zero_zero_when_armed(monkeypatch, capsys):
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
        [*all_real_pulse_args(), "--port", "/tmp/c30d", "--baud", "57600"],
        serial_factory=fake_serial_factory,
        sleep_fn=clock.sleep,
        clock=clock,
    )

    output = capsys.readouterr().out
    zero_frame = module.build_pulse_frames(0.03, 0.10).zero_frame
    pulse_frame = module.build_pulse_frames(0.03, 0.10).pulse_frame
    assert exit_code == 0
    assert factory_calls == [("/tmp/c30d", 57600)]
    assert fake_serial.write_calls == [zero_frame, pulse_frame, zero_frame, zero_frame]
    assert [len(frame) for frame in fake_serial.write_calls] == [11, 11, 11, 11]
    assert fake_serial.flush_calls == 4
    assert fake_serial.close_calls == 1
    assert "frame_hex: 7b 00 00 00 00 00 00 00 00 7b 7d" in output
    assert f"frame_hex: {pulse_frame.hex(' ')}" in output
    assert "real_write_performed: true" in output
    assert "bytes_written_total: 44" in output
    assert "pulse_target_x: 0.03" in output
    assert "pulse_duration_s: 0.1" in output
    assert "warning: wheels may spin briefly" in output


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
