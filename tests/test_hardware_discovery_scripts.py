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


def test_c30d_frame_stats_parser_defaults_do_not_open_port():
    module = load_script("c30d_frame_stats.py")

    args = module.build_parser().parse_args([])

    assert args.capture is None
    assert args.latest is False


def test_c30d_frame_stats_latest_selects_newest_bin(tmp_path):
    module = load_script("c30d_frame_stats.py")
    older = tmp_path / "older.bin"
    newer = tmp_path / "newer.bin"
    older.write_bytes(b"old")
    newer.write_bytes(b"new")

    os.utime(older, (1_000_000, 1_000_000))
    os.utime(newer, (2_000_000, 2_000_000))

    assert module.newest_capture(tmp_path) == newer


def test_c30d_frame_stats_formats_read_only_summary(capsys, tmp_path):
    module = load_script("c30d_frame_stats.py")
    capture = tmp_path / "capture.bin"
    repeated = bytes([0x7B, 0x10, 0xAA, *([0x10] * 20), 0x7D])
    changed = bytes([0x7B, 0x11, 0xAA, *([0x11] * 20), 0x7D])
    invalid = bytes([0x7B, *([0x33] * 22), 0x00])
    data = invalid + repeated + changed + repeated + repeated[:8]

    module.print_frame_stats(capture, data)

    output = capsys.readouterr().out
    assert "Read-only C30D frame analysis from saved capture only." in output
    assert "total_bytes: 104" in output
    assert "valid_fixed_length_frame_count: 3" in output
    assert "rejected_resync_count: 1" in output
    assert "partial_frame_count: 1" in output
    assert "frame_length_distribution: 24:3" in output
    assert "constant_byte_positions: 0=0x7b, 2=0xaa, 23=0x7d" in output
    assert "changing_byte_positions: 1" in output
    assert f"count=2 hex={repeated.hex(' ')}" in output


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


def test_oak_capture_default_output_stem():
    module = load_script("oak_capture_once.py")

    output = module.default_output_stem(datetime(2026, 5, 20, 8, 9, 10))

    assert output == Path("data/oak_tests/oak_capture_20260520_080910")


def test_rplidar_scan_parser_defaults_do_not_open_port():
    module = load_script("rplidar_scan_sample.py")

    args = module.build_parser().parse_args([])

    assert args.port == "/dev/rplidar"
    assert args.baud == 460800
    assert args.backend == "sdk"
    assert args.duration == 5.0
    assert args.output is None


def test_rplidar_scan_parser_accepts_overrides():
    module = load_script("rplidar_scan_sample.py")

    args = module.build_parser().parse_args(
        [
            "--backend",
            "pyrplidar",
            "--port",
            "/tmp/lidar",
            "--baud",
            "115200",
            "--duration",
            "1.5",
            "--output",
            "scan.csv",
            "--verbose",
        ]
    )

    assert args.backend == "pyrplidar"
    assert args.port == "/tmp/lidar"
    assert args.baud == 115200
    assert args.duration == 1.5
    assert args.output == Path("scan.csv")
    assert args.verbose is True


def test_rplidar_scan_default_output_path():
    module = load_script("rplidar_scan_sample.py")

    output = module.default_output_path(datetime(2026, 5, 20, 8, 9, 10))

    assert output == Path("data/rplidar_tests/rplidar_scan_20260520_080910.csv")


def test_rplidar_scan_raw_log_path():
    module = load_script("rplidar_scan_sample.py")

    output = module.raw_log_path_for(Path("data/rplidar_tests/rplidar_scan_20260520_080910.csv"))

    assert output == Path("data/rplidar_tests/rplidar_scan_20260520_080910.raw.log")


def test_rplidar_scan_parses_sdk_output_lines():
    module = load_script("rplidar_scan_sample.py")
    stdout = "\n".join(
        [
            "Ultra simple LIDAR data grabber for SLAMTEC LIDAR.",
            "S  theta: 012.50 Dist: 00123.25 Q: 47 ",
            "   theta: 359.75 Dist: 04567.00 Q: 12 ",
        ]
    )

    points, unparsed = module.parse_sdk_output(stdout, timestamp_s=42.0)

    assert unparsed == []
    assert points == [
        module.ScanPoint(timestamp_s=42.0, angle_deg=12.5, distance_mm=123.25, quality=47),
        module.ScanPoint(timestamp_s=42.0, angle_deg=359.75, distance_mm=4567.0, quality=12),
    ]


def test_rplidar_scan_reports_unparsed_sdk_scan_like_lines():
    module = load_script("rplidar_scan_sample.py")

    points, unparsed = module.parse_sdk_output("theta: bad Dist: 1 Q: 2", timestamp_s=42.0)

    assert points == []
    assert unparsed == ["theta: bad Dist: 1 Q: 2"]


def test_rplidar_scan_csv_row_formats_optional_quality():
    module = load_script("rplidar_scan_sample.py")

    with_quality = module.csv_row(
        module.ScanPoint(
            timestamp_s=1.23456789, angle_deg=45.6789, distance_mm=1234.5678, quality=12
        )
    )
    without_quality = module.csv_row(
        module.ScanPoint(timestamp_s=1.2, angle_deg=2.0, distance_mm=3.0, quality=None)
    )

    assert with_quality == {
        "timestamp_s": "1.234568",
        "angle_deg": "45.679",
        "distance_mm": "1234.568",
        "quality": "12",
    }
    assert without_quality["quality"] == ""


def test_rplidar_scan_measurement_to_point_and_summary():
    module = load_script("rplidar_scan_sample.py")

    class Measurement:
        angle = 10.5
        distance = 250.25
        quality = 7

    point = module.measurement_to_point(Measurement(), timestamp_s=42.0)
    summary = module.summarize_points(
        [
            point,
            module.ScanPoint(timestamp_s=43.0, angle_deg=20.0, distance_mm=100.0),
        ]
    )

    assert point == module.ScanPoint(
        timestamp_s=42.0,
        angle_deg=10.5,
        distance_mm=250.25,
        quality=7,
    )
    assert summary == {
        "count": 2,
        "min_angle": 10.5,
        "max_angle": 20.0,
        "min_distance": 100.0,
        "max_distance": 250.25,
    }
