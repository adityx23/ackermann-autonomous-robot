from __future__ import annotations

import importlib.util
import inspect
import sys
from types import SimpleNamespace
from dataclasses import dataclass
from pathlib import Path


def load_zero_frame_script():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "send_c30d_zero_frame_once.py"
    spec = importlib.util.spec_from_file_location(script_path.stem, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[script_path.stem] = module
    spec.loader.exec_module(module)
    return module


def all_guard_args() -> list[str]:
    return [
        "--armed",
        "--manual-enable",
        "--wheels-lifted",
        "--robot-restrained",
        "--manual-power-cutoff-ready",
        "--motor-enable-switch-reviewed",
        "--i-understand-this-sends-a-real-serial-frame",
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


def test_zero_frame_sender_defaults_to_five_second_c30d_only_preflight():
    module = load_zero_frame_script()

    args = module.build_parser().parse_args([])

    assert args.preflight_duration == 5.0
    assert args.preflight_mode == "c30d_only"


def test_zero_frame_sender_passes_preflight_duration_into_readiness(monkeypatch):
    module = load_zero_frame_script()
    calls = {}
    fake_report = FakeReadinessReport(readiness_allowed=True)

    fake_readiness = SimpleNamespace(
        run_readonly_preflight=lambda duration_s, mode: calls.setdefault(
            "preflight", (duration_s, mode)
        )
        or "preflight",
        preflight_summary_from_json=lambda path, mode, duration_s: calls.setdefault(
            "json", (path, mode, duration_s)
        )
        or "preflight",
        load_warning_battery_threshold=lambda: 10800,
        evaluate_readiness=lambda confirmations, preflight, threshold: fake_report,
        print_report=lambda report: calls.setdefault("printed", report),
    )
    monkeypatch.setitem(sys.modules, "c30d_first_write_readiness", fake_readiness)
    args = module.build_parser().parse_args(
        ["--preflight-duration", "7.5", "--full-sensor-preflight"]
    )

    report = module.run_internal_readiness(args)

    assert report is fake_report
    assert calls["preflight"] == (7.5, "full_sensor")
    assert calls["printed"] is fake_report


def test_zero_frame_sender_refuses_without_flags(capsys):
    module = load_zero_frame_script()
    fake_serial = FakeSerial()

    exit_code = module.main([], serial_factory=lambda **_: fake_serial)

    output = capsys.readouterr().out
    assert exit_code == 1
    assert "refused: missing_required_safety_flags" in output
    assert "armed" in output
    assert "real_write_performed: false" in output
    assert fake_serial.write_calls == []


def test_zero_frame_sender_refuses_if_readiness_false(monkeypatch, capsys):
    module = load_zero_frame_script()
    fake_serial = FakeSerial()
    monkeypatch.setattr(
        module,
        "run_internal_readiness",
        lambda _args: FakeReadinessReport(readiness_allowed=False),
    )

    exit_code = module.main(all_guard_args(), serial_factory=lambda **_: fake_serial)

    output = capsys.readouterr().out
    assert exit_code == 1
    assert "frame_hex: 7b 00 00 00 00 00 00 00 00 7b 7d" in output
    assert "refused: readiness_allowed_false" in output
    assert "real_write_performed: false" in output
    assert fake_serial.write_calls == []


def test_zero_frame_sender_validates_zero_frame():
    module = load_zero_frame_script()

    frame = module.build_zero_frame()
    validation = module.validate_zero_frame(frame)

    assert frame.hex(" ") == "7b 00 00 00 00 00 00 00 00 7b 7d"
    assert validation.valid is True
    assert validation.reasons == ()


def test_zero_frame_sender_rejects_any_nonzero_frame_path(monkeypatch, capsys):
    module = load_zero_frame_script()
    fake_serial = FakeSerial()
    frame = bytearray(module.ZERO_NEUTRAL_FRAME)
    frame[4] = 0x01
    frame[9] = 0x7A
    monkeypatch.setattr(module, "build_zero_frame", lambda: bytes(frame))
    monkeypatch.setattr(
        module,
        "run_internal_readiness",
        lambda _args: FakeReadinessReport(readiness_allowed=True),
    )

    exit_code = module.main(all_guard_args(), serial_factory=lambda **_: fake_serial)

    output = capsys.readouterr().out
    assert exit_code == 1
    assert "refused: zero_frame_validation_failed" in output
    assert "target_values_not_all_zero" in output
    assert "frame_not_hardcoded_zero_neutral" in output
    assert fake_serial.write_calls == []


def test_zero_frame_sender_writes_once_when_all_checks_pass(monkeypatch, capsys):
    module = load_zero_frame_script()
    fake_serial = FakeSerial()
    monkeypatch.setattr(
        module,
        "run_internal_readiness",
        lambda _args: FakeReadinessReport(readiness_allowed=True),
    )

    exit_code = module.main(all_guard_args(), serial_factory=lambda **_: fake_serial)

    output = capsys.readouterr().out
    assert exit_code == 0
    assert fake_serial.write_calls == [module.ZERO_NEUTRAL_FRAME]
    assert len(fake_serial.write_calls[0]) == 11
    assert fake_serial.flush_calls == 1
    assert fake_serial.close_calls == 1
    assert "real_write_performed: true" in output
    assert "bytes_written: 11" in output
    assert "frame_hex: 7b 00 00 00 00 00 00 00 00 7b 7d" in output
    assert "warning: zero/neutral frame only, not a motor pulse" in output


def test_zero_frame_sender_has_no_ros_or_ros2_imports():
    module = load_zero_frame_script()
    source = inspect.getsource(module)

    assert "import rospy" not in source
    assert "import rclpy" not in source
    assert "ros2" not in source.lower()
    assert "roslaunch" not in source
    assert "rostopic" not in source
