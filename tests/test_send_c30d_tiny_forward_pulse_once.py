from __future__ import annotations

import importlib.util
import inspect
import sys
from dataclasses import dataclass
from pathlib import Path


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
    def __init__(self) -> None:
        self.write_calls: list[bytes] = []
        self.flush_calls = 0
        self.close_calls = 0

    def write(self, data: bytes) -> int:
        self.write_calls.append(data)
        return len(data)

    def flush(self) -> None:
        self.flush_calls += 1

    def close(self) -> None:
        self.close_calls += 1


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
    sleep_calls = []

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
        sleep_fn=sleep_calls.append,
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
    assert sleep_calls == [0.05, 0.10, 0.05]
    assert "frame_hex: 7b 00 00 00 00 00 00 00 00 7b 7d" in output
    assert f"frame_hex: {pulse_frame.hex(' ')}" in output
    assert "real_write_performed: true" in output
    assert "bytes_written_total: 44" in output
    assert "pulse_target_x: 0.03" in output
    assert "pulse_duration_s: 0.1" in output
    assert "warning: wheels may spin briefly" in output


def test_tiny_forward_pulse_has_no_ros_or_ros2_imports():
    module = load_pulse_script()
    source = inspect.getsource(module)

    assert "import rospy" not in source
    assert "import rclpy" not in source
    assert "ros2" not in source.lower()
    assert "roslaunch" not in source
    assert "rostopic" not in source
