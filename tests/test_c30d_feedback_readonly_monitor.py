from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

from ackermann_robot.drivers.c30d_frames import FRAME_LENGTH


def load_monitor_script():
    script_path = (
        Path(__file__).resolve().parents[1] / "scripts" / "monitor_c30d_feedback_readonly.py"
    )
    spec = importlib.util.spec_from_file_location(script_path.stem, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[script_path.stem] = module
    spec.loader.exec_module(module)
    return module


def make_frame(fill: int, overrides: dict[int, int] | None = None) -> bytes:
    frame = bytearray([0x7B, *([fill] * (FRAME_LENGTH - 2)), 0x7D])
    for position, value in (overrides or {}).items():
        frame[position] = value
    return bytes(frame)


def test_extract_fixed_frames_from_buffer_handles_split_frame():
    module = load_monitor_script()
    frame = make_frame(0x11)
    buffer = bytearray()

    assert module.extract_fixed_frames_from_buffer(buffer, frame[:8]) == []
    assert bytes(buffer) == frame[:8]
    assert module.extract_fixed_frames_from_buffer(buffer, frame[8:]) == [frame]
    assert buffer == bytearray()


def test_extract_fixed_frames_from_buffer_discards_noise_before_frame():
    module = load_monitor_script()
    frame = make_frame(0x22)
    buffer = bytearray()

    frames = module.extract_fixed_frames_from_buffer(buffer, bytes([0x00, 0x55]) + frame)

    assert frames == [frame]
    assert buffer == bytearray()


def test_extract_fixed_frames_from_buffer_does_not_truncate_on_payload_end_marker():
    module = load_monitor_script()
    frame = make_frame(0x33, {5: 0x7D, 12: 0x7D})
    buffer = bytearray()

    frames = module.extract_fixed_frames_from_buffer(buffer, frame)

    assert frames == [frame]
    assert buffer == bytearray()


def test_extract_fixed_frames_from_buffer_resyncs_after_bad_end_marker():
    module = load_monitor_script()
    invalid = make_frame(0x44, {23: 0x00})
    valid = make_frame(0x55)
    buffer = bytearray()

    frames = module.extract_fixed_frames_from_buffer(buffer, invalid + valid)

    assert frames == [valid]
    assert buffer == bytearray()


def test_extract_fixed_frames_from_buffer_clears_noise_without_start_marker():
    module = load_monitor_script()
    buffer = bytearray(b"\x01\x02\x03")

    frames = module.extract_fixed_frames_from_buffer(buffer, b"\x04\x05")

    assert frames == []
    assert buffer == bytearray()
