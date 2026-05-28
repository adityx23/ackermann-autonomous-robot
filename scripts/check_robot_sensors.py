#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
SCRIPT_ROOT = REPO_ROOT / "scripts"
for path in (SRC_ROOT, SCRIPT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

DEFAULT_C30D_PORT = "/dev/c30d"
DEFAULT_RPLIDAR_PORT = "/dev/rplidar"
DEFAULT_C30D_BAUD = 115200
DEFAULT_RPLIDAR_BAUD = 460800
DEFAULT_DURATION_S = 3.0
DEFAULT_C30D_MIN_FRAME_RATE_HZ = 10.0
DEFAULT_RPLIDAR_MIN_VALID_POINTS = 20
DEFAULT_PREFLIGHT_DIR = Path("data/preflight")


@dataclass(frozen=True)
class CheckResult:
    name: str
    passed: bool
    message: str
    details: dict[str, Any]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Native read-only robot sensor health/preflight checker."
    )
    parser.add_argument("--check-c30d", dest="check_c30d", action="store_true", default=True)
    parser.add_argument("--no-check-c30d", dest="check_c30d", action="store_false")
    parser.add_argument("--check-rplidar", dest="check_rplidar", action="store_true", default=True)
    parser.add_argument("--no-check-rplidar", dest="check_rplidar", action="store_false")
    parser.add_argument("--check-oak", dest="check_oak", action="store_true", default=True)
    parser.add_argument("--no-check-oak", dest="check_oak", action="store_false")
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_S,
        help="Short C30D/RPLIDAR check duration in seconds.",
    )
    parser.add_argument("--c30d-port", default=DEFAULT_C30D_PORT, help="C30D read-only port.")
    parser.add_argument("--rplidar-port", default=DEFAULT_RPLIDAR_PORT, help="RPLIDAR port.")
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if args.duration <= 0.0:
        raise ValueError("--duration must be greater than zero")


def status_text(passed: bool) -> str:
    return "PASS" if passed else "FAIL"


def frame_rate(frame_count: int, elapsed_s: float) -> float:
    if elapsed_s <= 0.0:
        return 0.0
    return frame_count / elapsed_s


def valid_lidar_point_count(points: list[Any]) -> int:
    return sum(1 for point in points if getattr(point, "distance_mm", 0.0) > 0.0)


def timestamped_path(directory: Path, stem: str, suffix: str, now: datetime | None = None) -> Path:
    timestamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
    return directory / f"{stem}_{timestamp}{suffix}"


def check_path_exists(name: str, path: Path) -> CheckResult:
    passed = path.exists()
    kind = "exists" if passed else "missing"
    return CheckResult(name=name, passed=passed, message=f"{path}: {kind}", details={"path": path})


def check_directory(name: str, path: Path) -> CheckResult:
    passed = path.is_dir()
    kind = "directory exists" if passed else "missing directory"
    return CheckResult(name=name, passed=passed, message=f"{path}: {kind}", details={"path": path})


def check_c30d_readonly(
    port: str,
    duration_s: float,
    baud: int = DEFAULT_C30D_BAUD,
    min_frame_rate_hz: float = DEFAULT_C30D_MIN_FRAME_RATE_HZ,
) -> CheckResult:
    from ackermann_robot.drivers.c30d_feedback import parse_feedback_candidates
    from monitor_c30d_feedback_readonly import (
        READ_SIZE,
        extract_fixed_frames_from_buffer,
        open_readonly_serial_fd,
    )

    buffer = bytearray()
    parsed_count = 0
    start_s = time.monotonic()
    deadline_s = start_s + duration_s
    fd: int | None = None

    try:
        fd = open_readonly_serial_fd(port, baud)
        while time.monotonic() < deadline_s:
            chunk = os.read(fd, READ_SIZE)
            if not chunk:
                continue
            for frame in extract_fixed_frames_from_buffer(buffer, chunk):
                parse_feedback_candidates([frame])
                parsed_count += 1
    except Exception as exc:
        elapsed_s = max(time.monotonic() - start_s, 0.0)
        return CheckResult(
            name="c30d",
            passed=False,
            message=f"read-only C30D check failed: {exc}",
            details={"frame_count": parsed_count, "elapsed_s": elapsed_s, "frame_rate_hz": 0.0},
        )
    finally:
        if fd is not None:
            os.close(fd)

    elapsed_s = max(time.monotonic() - start_s, 0.0)
    rate_hz = frame_rate(parsed_count, elapsed_s)
    passed = rate_hz >= min_frame_rate_hz
    return CheckResult(
        name="c30d",
        passed=passed,
        message=(
            f"frames={parsed_count} frame_rate_hz={rate_hz:.2f} "
            f"threshold_hz={min_frame_rate_hz:.2f}"
        ),
        details={
            "frame_count": parsed_count,
            "elapsed_s": elapsed_s,
            "frame_rate_hz": rate_hz,
            "threshold_hz": min_frame_rate_hz,
        },
    )


def check_rplidar_capture(
    port: str,
    duration_s: float,
    baud: int = DEFAULT_RPLIDAR_BAUD,
    output_dir: Path = DEFAULT_PREFLIGHT_DIR,
    min_valid_points: int = DEFAULT_RPLIDAR_MIN_VALID_POINTS,
) -> CheckResult:
    from rplidar_scan_sample import DEFAULT_SDK_BINARY, capture_scan

    output_path = timestamped_path(output_dir, "rplidar_preflight", ".csv")
    try:
        result = capture_scan(
            backend="sdk",
            port=port,
            baud=baud,
            duration_s=duration_s,
            output_path=output_path,
            sdk_binary=DEFAULT_SDK_BINARY,
        )
    except Exception as exc:
        return CheckResult(
            name="rplidar",
            passed=False,
            message=f"RPLIDAR capture failed: {exc}",
            details={"point_count": 0, "valid_point_count": 0, "output_path": output_path},
        )

    point_count = len(result.points)
    valid_count = valid_lidar_point_count(result.points)
    passed = valid_count >= min_valid_points
    return CheckResult(
        name="rplidar",
        passed=passed,
        message=(
            f"points={point_count} valid_points={valid_count} "
            f"threshold={min_valid_points} output={result.output_path}"
        ),
        details={
            "point_count": point_count,
            "valid_point_count": valid_count,
            "threshold": min_valid_points,
            "output_path": result.output_path,
        },
    )


def check_oak_capture(output_dir: Path = DEFAULT_PREFLIGHT_DIR) -> CheckResult:
    from record_readonly_sensor_run import OakCaptureSettings, capture_oak_rgb_once

    try:
        output_path = capture_oak_rgb_once(
            OakCaptureSettings(fps=2.0, preview_width=320, preview_height=180, timeout_s=5.0),
            output_dir / "oak_rgb",
        )
    except Exception as exc:
        return CheckResult(
            name="oak",
            passed=False,
            message=f"OAK-D Lite RGB capture failed: {exc}",
            details={"output_path": None},
        )

    return CheckResult(
        name="oak",
        passed=True,
        message=f"RGB capture saved: {output_path}",
        details={"output_path": output_path},
    )


def print_result(result: CheckResult) -> None:
    print(f"{status_text(result.passed)} {result.name}: {result.message}")


def print_summary(results: list[CheckResult]) -> None:
    print("Preflight Summary")
    for result in results:
        print_result(result)
    overall = all(result.passed for result in results)
    print(f"overall: {status_text(overall)}")


def run_preflight(args: argparse.Namespace) -> int:
    validate_args(args)
    results: list[CheckResult] = [
        check_directory("data", Path("data")),
        check_directory("data/runs", Path("data/runs")),
    ]

    if args.check_c30d:
        results.append(check_path_exists("device:/dev/c30d", Path(args.c30d_port)))
        results.append(check_c30d_readonly(args.c30d_port, args.duration))
    if args.check_rplidar:
        results.append(check_path_exists("device:/dev/rplidar", Path(args.rplidar_port)))
        results.append(check_rplidar_capture(args.rplidar_port, args.duration))
    if args.check_oak:
        results.append(check_oak_capture())

    print_summary(results)
    return 0 if all(result.passed for result in results) else 1


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return run_preflight(args)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
