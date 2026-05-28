#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from dataclasses import asdict, dataclass, fields, replace
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
SCRIPT_ROOT = REPO_ROOT / "scripts"
for path in (SRC_ROOT, SCRIPT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

DEFAULT_OUTPUT_ROOT = Path("data/runs")
DEFAULT_DURATION_S = 5.0
DEFAULT_C30D_PORT = "/dev/c30d"
DEFAULT_C30D_BAUD = 115200
DEFAULT_RPLIDAR_PORT = "/dev/rplidar"
DEFAULT_RPLIDAR_BAUD = 460800
DEFAULT_OAK_FPS = 5.0
DEFAULT_OAK_PREVIEW_WIDTH = 640
DEFAULT_OAK_PREVIEW_HEIGHT = 360
DEFAULT_OAK_TIMEOUT_S = 10.0


@dataclass(frozen=True)
class OakCaptureSettings:
    fps: float = DEFAULT_OAK_FPS
    preview_width: int = DEFAULT_OAK_PREVIEW_WIDTH
    preview_height: int = DEFAULT_OAK_PREVIEW_HEIGHT
    timeout_s: float = DEFAULT_OAK_TIMEOUT_S
    frame_count: int = 1


@dataclass(frozen=True)
class RunSettings:
    duration_s: float
    enable_c30d: bool
    enable_rplidar: bool
    enable_oak: bool
    output_root: Path
    c30d_port: str = DEFAULT_C30D_PORT
    c30d_baud: int = DEFAULT_C30D_BAUD
    rplidar_port: str = DEFAULT_RPLIDAR_PORT
    rplidar_baud: int = DEFAULT_RPLIDAR_BAUD
    oak: OakCaptureSettings = OakCaptureSettings()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Unified read-only robot sensor logger for C30D, RPLIDAR, and OAK-D Lite."
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_S,
        help="Run duration in seconds for streaming sensors.",
    )
    parser.add_argument("--enable-c30d", action="store_true", help="Record read-only C30D feedback.")
    parser.add_argument("--enable-rplidar", action="store_true", help="Record a RPLIDAR scan CSV.")
    parser.add_argument("--enable-oak", action="store_true", help="Capture one OAK-D Lite RGB frame.")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=DEFAULT_OUTPUT_ROOT,
        help="Root directory for timestamped run folders.",
    )
    parser.add_argument("--c30d-port", default=DEFAULT_C30D_PORT, help="C30D read-only serial port.")
    parser.add_argument("--c30d-baud", type=int, default=DEFAULT_C30D_BAUD, help="C30D baud rate.")
    parser.add_argument("--rplidar-port", default=DEFAULT_RPLIDAR_PORT, help="RPLIDAR serial port.")
    parser.add_argument(
        "--rplidar-baud",
        type=int,
        default=DEFAULT_RPLIDAR_BAUD,
        help="RPLIDAR baud rate.",
    )
    parser.add_argument("--oak-fps", type=float, default=DEFAULT_OAK_FPS, help="OAK RGB FPS.")
    parser.add_argument(
        "--oak-preview-width",
        type=int,
        default=DEFAULT_OAK_PREVIEW_WIDTH,
        help="OAK RGB preview width.",
    )
    parser.add_argument(
        "--oak-preview-height",
        type=int,
        default=DEFAULT_OAK_PREVIEW_HEIGHT,
        help="OAK RGB preview height.",
    )
    parser.add_argument(
        "--oak-timeout",
        type=float,
        default=DEFAULT_OAK_TIMEOUT_S,
        help="OAK RGB frame wait timeout in seconds.",
    )
    return parser


def settings_from_args(args: argparse.Namespace) -> RunSettings:
    return RunSettings(
        duration_s=args.duration,
        enable_c30d=args.enable_c30d,
        enable_rplidar=args.enable_rplidar,
        enable_oak=args.enable_oak,
        output_root=args.output_root,
        c30d_port=args.c30d_port,
        c30d_baud=args.c30d_baud,
        rplidar_port=args.rplidar_port,
        rplidar_baud=args.rplidar_baud,
        oak=OakCaptureSettings(
            fps=args.oak_fps,
            preview_width=args.oak_preview_width,
            preview_height=args.oak_preview_height,
            timeout_s=args.oak_timeout,
        ),
    )


def validate_settings(settings: RunSettings) -> None:
    if settings.duration_s <= 0.0:
        raise ValueError("--duration must be greater than zero")
    if not (settings.enable_c30d or settings.enable_rplidar or settings.enable_oak):
        raise ValueError("enable at least one sensor with --enable-c30d, --enable-rplidar, or --enable-oak")
    if settings.oak.fps <= 0.0:
        raise ValueError("--oak-fps must be greater than zero")
    if settings.oak.preview_width <= 0 or settings.oak.preview_height <= 0:
        raise ValueError("OAK preview dimensions must be greater than zero")
    if settings.oak.timeout_s <= 0.0:
        raise ValueError("--oak-timeout must be greater than zero")


def run_folder_name(start_time: datetime) -> str:
    return f"run_{start_time.strftime('%Y%m%d_%H%M%S')}"


def create_run_folder(output_root: Path, start_time: datetime) -> Path:
    output_root.mkdir(parents=True, exist_ok=True)
    base = output_root / run_folder_name(start_time)
    candidate = base
    suffix = 1
    while candidate.exists():
        candidate = base.with_name(f"{base.name}_{suffix:02d}")
        suffix += 1
    candidate.mkdir(parents=True)
    return candidate


def enabled_sensors(settings: RunSettings) -> list[str]:
    sensors: list[str] = []
    if settings.enable_c30d:
        sensors.append("c30d")
    if settings.enable_rplidar:
        sensors.append("rplidar")
    if settings.enable_oak:
        sensors.append("oak")
    return sensors


def metadata_for_run(settings: RunSettings, start_time: datetime) -> dict[str, Any]:
    return {
        "start_time": start_time.isoformat(),
        "duration_s": settings.duration_s,
        "enabled_sensors": enabled_sensors(settings),
        "c30d": {
            "enabled": settings.enable_c30d,
            "port": settings.c30d_port,
            "baud": settings.c30d_baud,
            "access": "read_only",
        },
        "rplidar": {
            "enabled": settings.enable_rplidar,
            "port": settings.rplidar_port,
            "baud": settings.rplidar_baud,
        },
        "oak": {
            "enabled": settings.enable_oak,
            **asdict(settings.oak),
            "rgb_output_dir": "oak_rgb",
        },
        "safety": {
            "note": "read-only sensor run; sends no motor or steering commands",
            "c30d_write": False,
            "motor_commands": False,
            "ros2": False,
        },
    }


def write_metadata(run_dir: Path, metadata: dict[str, Any]) -> Path:
    path = run_dir / "metadata.yaml"
    path.write_text(yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8")
    return path


def c30d_output_paths(run_dir: Path) -> tuple[Path, Path]:
    return run_dir / "c30d_feedback.csv", run_dir / "c30d_odometry.csv"


def record_c30d(settings: RunSettings, run_dir: Path) -> dict[str, int]:
    from ackermann_robot.drivers.c30d_feedback import (
        C30DFeedbackCandidate,
        parse_feedback_candidates,
    )
    from monitor_c30d_feedback_readonly import (
        READ_SIZE,
        extract_fixed_frames_from_buffer,
        open_readonly_serial_fd,
    )
    from monitor_c30d_odometry_readonly import (
        LiveOdometrySample,
        LiveOdometryState,
        update_live_odometry,
    )
    from ackermann_robot.odometry.c30d_dead_reckoning import load_c30d_calibration

    feedback_path, odometry_path = c30d_output_paths(run_dir)
    feedback_path.parent.mkdir(parents=True, exist_ok=True)
    calibration = load_c30d_calibration("config/c30d_calibration.yaml")

    parsed_count = 0
    invalid_checksum_count = 0
    candidate_battery_values: list[int] = []
    state = LiveOdometryState()
    buffer = bytearray()
    deadline = time.monotonic() + settings.duration_s
    fd: int | None = None

    with feedback_path.open("w", newline="") as feedback_file:
        feedback_writer = csv.DictWriter(
            feedback_file,
            fieldnames=[field.name for field in fields(C30DFeedbackCandidate)],
        )
        feedback_writer.writeheader()

        with odometry_path.open("w", newline="") as odometry_file:
            odometry_writer = csv.DictWriter(
                odometry_file,
                fieldnames=[field.name for field in fields(LiveOdometrySample)],
            )
            odometry_writer.writeheader()

            try:
                fd = open_readonly_serial_fd(settings.c30d_port, settings.c30d_baud)
                while time.monotonic() < deadline:
                    chunk = os.read(fd, READ_SIZE)
                    if not chunk:
                        continue

                    for frame in extract_fixed_frames_from_buffer(buffer, chunk):
                        candidate = replace(
                            parse_feedback_candidates([frame])[0],
                            frame_index=parsed_count,
                        )
                        if not candidate.checksum_valid:
                            invalid_checksum_count += 1
                        candidate_battery_values.append(candidate.candidate_battery_mV)
                        state, odometry = update_live_odometry(
                            candidate=candidate,
                            state=state,
                            forward_m_per_count=calibration.forward_m_per_count,
                            mode="straight_only",
                        )
                        feedback_writer.writerow(asdict(candidate))
                        odometry_writer.writerow(asdict(odometry))
                        parsed_count += 1
            finally:
                if fd is not None:
                    os.close(fd)

    candidate_battery_min = min(candidate_battery_values) if candidate_battery_values else None
    candidate_battery_mean = (
        sum(candidate_battery_values) / len(candidate_battery_values)
        if candidate_battery_values
        else None
    )
    candidate_battery_max = max(candidate_battery_values) if candidate_battery_values else None
    return {
        "feedback_frames": parsed_count,
        "odometry_rows": parsed_count,
        "invalid_checksum_frames": invalid_checksum_count,
        "candidate_battery_mV_min": candidate_battery_min,
        "candidate_battery_mV_mean": candidate_battery_mean,
        "candidate_battery_mV_max": candidate_battery_max,
    }


def record_rplidar(settings: RunSettings, run_dir: Path) -> int:
    from rplidar_scan_sample import DEFAULT_SDK_BINARY, capture_scan

    output_path = run_dir / "rplidar_scan.csv"
    result = capture_scan(
        backend="sdk",
        port=settings.rplidar_port,
        baud=settings.rplidar_baud,
        duration_s=settings.duration_s,
        output_path=output_path,
        sdk_binary=DEFAULT_SDK_BINARY,
    )
    return len(result.points)


def capture_oak_rgb_once(settings: OakCaptureSettings, output_dir: Path) -> Path:
    import cv2
    import depthai as dai

    output_dir.mkdir(parents=True, exist_ok=True)
    with dai.Pipeline() as pipeline:
        cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        rgb_output = cam.requestOutput(
            size=(settings.preview_width, settings.preview_height),
            type=dai.ImgFrame.Type.BGR888p,
            fps=settings.fps,
        )
        rgb_queue = rgb_output.createOutputQueue(maxSize=1, blocking=False)
        pipeline.start()

        frame = None
        deadline = time.monotonic() + settings.timeout_s
        while time.monotonic() < deadline and pipeline.isRunning():
            msg = rgb_queue.tryGet()
            if msg is not None:
                frame = msg.getCvFrame()
                break
            time.sleep(0.05)

        if frame is None:
            raise RuntimeError(f"no OAK RGB frame received within {settings.timeout_s}s")

        output_path = output_dir / "oak_rgb_0000.jpg"
        if not cv2.imwrite(str(output_path), frame):
            raise RuntimeError(f"failed to save OAK RGB frame: {output_path}")
        return output_path


def record_oak(settings: RunSettings, run_dir: Path) -> int:
    capture_oak_rgb_once(settings.oak, run_dir / "oak_rgb")
    return 1


def record_enabled_sensors(settings: RunSettings, run_dir: Path) -> dict[str, Any]:
    results: dict[str, Any] = {}
    if settings.enable_c30d:
        print("Recording C30D feedback read-only.")
        results["c30d"] = record_c30d(settings, run_dir)
        if results["c30d"]["invalid_checksum_frames"]:
            print("warning: invalid C30D feedback checksum frames observed", file=sys.stderr)
    if settings.enable_rplidar:
        print("Recording RPLIDAR scan.")
        results["rplidar_points"] = record_rplidar(settings, run_dir)
    if settings.enable_oak:
        print("Capturing OAK-D Lite RGB frame.")
        results["oak_rgb_frames"] = record_oak(settings, run_dir)
    return results


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    settings = settings_from_args(args)
    try:
        validate_settings(settings)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    start_time = datetime.now().astimezone()
    run_dir = create_run_folder(settings.output_root, start_time)
    metadata_path = write_metadata(run_dir, metadata_for_run(settings, start_time))

    print("READ-ONLY robot sensor run logger.")
    print("C30D is opened read-only when enabled. This script never sends motor commands.")
    print(f"run_dir: {run_dir}")
    print(f"metadata: {metadata_path}")

    try:
        results = record_enabled_sensors(settings, run_dir)
    except Exception as exc:
        print(f"failed during read-only sensor run: {exc}", file=sys.stderr)
        return 1

    for key, value in results.items():
        print(f"{key}: {value}")
    print("done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
