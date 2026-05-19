from __future__ import annotations

import importlib.util
import os
from datetime import datetime
from pathlib import Path


def load_script(name: str):
    script_path = Path(__file__).resolve().parents[1] / "scripts" / name
    spec = importlib.util.spec_from_file_location(script_path.stem, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_rplidar_parser_defaults_do_not_open_port():
    module = load_script("test_rplidar_port.py")

    args = module.build_parser().parse_args([])

    assert args.port == "/dev/rplidar"
    assert args.baud == 460800


def test_c30d_capture_parser_defaults_do_not_open_port():
    module = load_script("capture_c30d_passive.py")

    args = module.build_parser().parse_args([])

    assert args.port == "/dev/c30d"
    assert args.baud == 115200
    assert args.duration == 5.0
    assert args.output is None


def test_c30d_capture_default_output_path():
    module = load_script("capture_c30d_passive.py")

    output = module.default_output_path(datetime(2026, 5, 19, 12, 34, 56))

    assert output == Path("data/c30d_captures/c30d_capture_20260519_123456.bin")


def test_analyze_c30d_capture_helpers_do_not_assign_protocol_meanings():
    module = load_script("analyze_c30d_capture.py")
    data = bytes([0x00, 0x7B, 0x01, 0x02, 0x7D, 0x7B, 0x03, 0x7D, 0x04])

    assert module.first_bytes_hex(data, limit=5) == "00 7b 01 02 7d"
    assert module.count_frame_markers(data) == (2, 2)
    assert module.extract_candidate_frames(data) == [
        bytes([0x7B, 0x01, 0x02, 0x7D]),
        bytes([0x7B, 0x03, 0x7D]),
    ]


def test_analyze_c30d_capture_frame_length_distribution():
    module = load_script("analyze_c30d_capture.py")
    frames = [
        bytes([0x7B, 0x01, 0x7D]),
        bytes([0x7B, 0x02, 0x03, 0x7D]),
        bytes([0x7B, 0x04, 0x7D]),
    ]

    distribution = module.frame_length_distribution(frames)

    assert distribution == {3: 2, 4: 1}
    assert module.format_length_distribution(distribution) == "3:2, 4:1"


def test_analyze_c30d_capture_latest_selects_newest_bin(tmp_path):
    module = load_script("analyze_c30d_capture.py")
    older = tmp_path / "older.bin"
    newer = tmp_path / "newer.bin"
    ignored = tmp_path / "newer.txt"
    older.write_bytes(b"old")
    newer.write_bytes(b"new")
    ignored.write_text("ignored", encoding="utf-8")

    os.utime(older, (1_000_000, 1_000_000))
    os.utime(newer, (2_000_000, 2_000_000))

    assert module.newest_capture(tmp_path) == newer


def test_analyze_c30d_capture_resolve_requires_path_or_latest():
    module = load_script("analyze_c30d_capture.py")

    try:
        module.resolve_capture_path(None, latest=False)
    except ValueError as exc:
        assert "capture path" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_oak_camera_feature_formatter_handles_plain_objects():
    module = load_script("test_oak_detect.py")

    class Feature:
        socket = "CAM_A"
        supportedTypes = ["COLOR"]

    assert module.format_camera_feature(Feature()) == "CAM_A: COLOR"
