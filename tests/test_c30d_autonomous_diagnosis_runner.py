from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

from ackermann_robot.drivers.c30d_checksum import compute_feedback_checksum
from ackermann_robot.drivers.c30d_frames import FRAME_LENGTH


def load_runner():
    script_path = (
        Path(__file__).resolve().parents[1] / "scripts" / "c30d_autonomous_diagnosis_runner.py"
    )
    spec = importlib.util.spec_from_file_location("c30d_autonomous_diagnosis_runner", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def make_feedback_frame(status: int = 0, forward: int = 0, battery_mv: int = 12000) -> bytes:
    frame = bytearray([0x7B, *([0x00] * (FRAME_LENGTH - 2)), 0x7D])
    frame[2:4] = forward.to_bytes(2, "big", signed=True)
    frame[20:22] = battery_mv.to_bytes(2, "big", signed=False)
    frame[21] = status
    frame[22] = compute_feedback_checksum(frame)
    return bytes(frame)


class ExplodingSerial:
    def __call__(self, *_args, **_kwargs):
        raise AssertionError("offline mode must not open serial")


def test_offline_report_generation(tmp_path: Path, monkeypatch):
    module = load_runner()
    capture_dir = tmp_path / "data" / "c30d_captures"
    capture_dir.mkdir(parents=True)
    (capture_dir / "stationary.bin").write_bytes(make_feedback_frame(status=1))
    live_dir = tmp_path / "data" / "c30d_live"
    analysis_dir = tmp_path / "data" / "c30d_analysis"
    diagnostics_dir = tmp_path / "data" / "c30d_diagnostics"
    live_dir.mkdir()
    analysis_dir.mkdir()
    diagnostics_dir.mkdir()

    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(
        module,
        "INSPECTED_DIRS",
        (
            Path("data/c30d_captures"),
            Path("data/c30d_live"),
            Path("data/c30d_analysis"),
            Path("data/c30d_diagnostics"),
        ),
    )
    monkeypatch.setattr(module, "REPORT_ROOT", Path("data/c30d_diagnostics"))

    state = module.run_offline_analysis(Path("data/c30d_diagnostics/report"))
    report_path = module.write_report(state)

    text = report_path.read_text(encoding="utf-8")
    assert "Confirmed facts" in text
    assert "Python host command builder matches the 11-byte layout" in text
    assert "Battery/frame-rate/checksum summaries" in text
    assert "Status/mode byte candidates" in text


def test_byte_difference_detection():
    module = load_runner()
    left = [bytes([0x7B, 0x00, 0x01, 0x7D])]
    right = [bytes([0x7B, 0x00, 0x02, 0x7D])]

    differences = module.detect_byte_differences(left, right)

    assert len(differences) == 1
    assert differences[0].position == 2
    assert differences[0].left_values == (0x01,)
    assert differences[0].right_values == (0x02,)


def test_offline_mode_never_writes_to_dev_c30d(tmp_path: Path, monkeypatch):
    module = load_runner()
    for directory in module.INSPECTED_DIRS:
        (tmp_path / directory).mkdir(parents=True, exist_ok=True)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(module, "REPORT_ROOT", Path("data/c30d_diagnostics"))

    assert module.main(["--offline-only"], serial_factory=ExplodingSerial()) == 0


def test_guided_live_requires_explicit_confirmation_before_zero_frame(monkeypatch):
    module = load_runner()
    calls: list[str] = []

    def fake_input(prompt: str) -> str:
        calls.append(prompt)
        return "NO"

    class FakeReport:
        readiness_allowed = True

        class preflight:
            counted_invalid_checksum_count = 0
            invalid_checksum_count = 0

    monkeypatch.setattr(module, "run_readiness_for_live_zero", lambda: FakeReport())

    sent = module.write_zero_frame_after_confirmation(
        port="/dev/c30d",
        baud=115200,
        input_fn=fake_input,
        serial_factory=ExplodingSerial(),
    )

    assert sent is False
    assert calls


def test_continue_until_exhausted_stops_when_all_branches_are_exhausted(tmp_path: Path):
    module = load_runner()
    state = module.DiagnosisState(report_dir=tmp_path, live_captures_required=False)
    state.captures.append(
        module.CaptureSummary(
            path=tmp_path / "live.csv",
            kind="csv_capture",
            byte_count=10,
            frame_count=2,
            valid_checksum_count=2,
            invalid_checksum_count=0,
            phases=("baseline", "post"),
            apparent_write_effect=False,
        )
    )

    assert module.all_branches_exhausted(state) is True
    assert (
        module.choose_recommendation(state)
        == "bypass C30D actuation using separate MCU/motor drivers"
    )


def test_no_ros_or_ros2_imports_exist():
    source = (
        Path(__file__).resolve().parents[1] / "scripts" / "c30d_autonomous_diagnosis_runner.py"
    ).read_text(encoding="utf-8")
    assert "import rospy" not in source
    assert "import rclpy" not in source
