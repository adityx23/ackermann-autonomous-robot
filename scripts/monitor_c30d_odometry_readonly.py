#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from dataclasses import asdict, dataclass, fields, replace
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
SCRIPT_ROOT = REPO_ROOT / "scripts"
for path in (SRC_ROOT, SCRIPT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

DEFAULT_PORT = "/dev/c30d"
DEFAULT_BAUD = 115200
DEFAULT_DURATION_S = 5.0
DEFAULT_PRINT_EVERY = 10
DEFAULT_CONFIG = Path("config/c30d_calibration.yaml")
DEFAULT_OUTPUT_DIR = Path("data/c30d_live")


@dataclass(frozen=True)
class LiveOdometryState:
    x_m: float = 0.0
    y_m: float = 0.0
    theta_rad: float = 0.0


@dataclass(frozen=True)
class LiveOdometrySample:
    frame_index: int
    forward_candidate: int
    yaw_candidate: int
    delta_s_m: float
    x_m: float
    y_m: float
    theta_rad: float


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read-only live provisional C30D odometry monitor."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="C30D serial port to open read-only.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate.")
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_S,
        help="Monitor duration in seconds.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help="Path to provisional C30D calibration YAML.",
    )
    parser.add_argument(
        "--mode",
        choices=("straight_only", "raw_yaw_candidate"),
        default="straight_only",
        help="Yaw handling mode. Yaw is not converted to radians while calibration is null.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional CSV filename to save under data/c30d_live/.",
    )
    parser.add_argument(
        "--print-every",
        type=int,
        default=DEFAULT_PRINT_EVERY,
        help="Print one compact live line every N parsed frames.",
    )
    return parser


def resolve_output_path(output: Path | None, now: datetime | None = None) -> Path | None:
    if output is None:
        return None
    filename = output.name
    if not filename:
        timestamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
        filename = f"c30d_live_odometry_{timestamp}.csv"
    path = DEFAULT_OUTPUT_DIR / filename
    if path.suffix != ".csv":
        path = path.with_suffix(".csv")
    return path


def update_live_odometry(
    candidate,
    state: LiveOdometryState,
    forward_m_per_count: float,
    mode: str,
) -> tuple[LiveOdometryState, LiveOdometrySample]:
    if mode not in ("straight_only", "raw_yaw_candidate"):
        raise ValueError(f"unsupported live C30D odometry mode: {mode}")

    delta_s_m = candidate.candidate_forward_motion * forward_m_per_count
    yaw_candidate = candidate.candidate_yaw_motion
    next_x_m = state.x_m + delta_s_m * math.cos(state.theta_rad)
    next_y_m = state.y_m + delta_s_m * math.sin(state.theta_rad)
    next_state = LiveOdometryState(
        x_m=next_x_m,
        y_m=next_y_m,
        theta_rad=state.theta_rad,
    )
    return next_state, LiveOdometrySample(
        frame_index=candidate.frame_index,
        forward_candidate=candidate.candidate_forward_motion,
        yaw_candidate=yaw_candidate,
        delta_s_m=delta_s_m,
        x_m=next_state.x_m,
        y_m=next_state.y_m,
        theta_rad=next_state.theta_rad,
    )


def format_live_odometry_line(sample: LiveOdometrySample) -> str:
    return (
        f"frame_index={sample.frame_index} "
        f"forward_candidate={sample.forward_candidate} "
        f"yaw_candidate={sample.yaw_candidate} "
        f"delta_s_m={sample.delta_s_m:.6g} "
        f"x_m={sample.x_m:.6g} "
        f"y_m={sample.y_m:.6g} "
        f"theta_rad={sample.theta_rad:.6g}"
    )


def open_csv_writer(output_path: Path | None):
    if output_path is None:
        return None, None

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_file = output_path.open("w", newline="")
    writer = csv.DictWriter(output_file, fieldnames=[field.name for field in fields(LiveOdometrySample)])
    writer.writeheader()
    return output_file, writer


def monitor_odometry(
    port: str,
    baud: int,
    duration_s: float,
    config_path: Path,
    mode: str,
    output_path: Path | None,
    print_every: int,
) -> int:
    from ackermann_robot.drivers.c30d_feedback import parse_feedback_candidates
    from ackermann_robot.odometry.c30d_dead_reckoning import load_c30d_calibration
    from monitor_c30d_feedback_readonly import (
        READ_SIZE,
        extract_fixed_frames_from_buffer,
        open_readonly_serial_fd,
    )

    calibration = load_c30d_calibration(config_path)
    if calibration.yaw_rad_per_count is not None:
        raise ValueError("calibrated live C30D yaw odometry is not implemented")

    parsed_count = 0
    state = LiveOdometryState()
    buffer = bytearray()
    output_file, writer = open_csv_writer(output_path)
    deadline = time.monotonic() + duration_s

    fd: int | None = None
    try:
        fd = open_readonly_serial_fd(port, baud)
        while time.monotonic() < deadline:
            chunk = os.read(fd, READ_SIZE)
            if not chunk:
                continue

            for frame in extract_fixed_frames_from_buffer(buffer, chunk):
                candidate = replace(
                    parse_feedback_candidates([frame])[0],
                    frame_index=parsed_count,
                )
                state, sample = update_live_odometry(
                    candidate=candidate,
                    state=state,
                    forward_m_per_count=calibration.forward_m_per_count,
                    mode=mode,
                )
                parsed_count += 1

                if writer is not None:
                    writer.writerow(asdict(sample))

                if parsed_count == 1 or parsed_count % print_every == 0:
                    print(format_live_odometry_line(sample))
    finally:
        if fd is not None:
            os.close(fd)
        if output_file is not None:
            output_file.close()

    return parsed_count


def validate_args(args: argparse.Namespace) -> None:
    if args.duration <= 0.0:
        raise ValueError("--duration must be greater than zero")
    if args.print_every <= 0:
        raise ValueError("--print-every must be greater than zero")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        validate_args(args)
        output_path = resolve_output_path(args.output)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    print("PROVISIONAL READ-ONLY C30D odometry monitor.")
    print("This script opens the C30D port read-only and never sends motor or steering commands.")
    print("yaw_calibration: unavailable; theta_rad remains fixed at zero")
    print(f"port: {args.port}")
    print(f"baud: {args.baud}")
    print(f"duration_s: {args.duration}")
    print(f"config: {args.config}")
    print(f"mode: {args.mode}")
    if output_path is not None:
        print(f"output_path: {output_path}")

    try:
        frame_count = monitor_odometry(
            port=args.port,
            baud=args.baud,
            duration_s=args.duration,
            config_path=args.config,
            mode=args.mode,
            output_path=output_path,
            print_every=args.print_every,
        )
    except (OSError, ValueError) as exc:
        print(f"failed to monitor provisional C30D odometry read-only: {exc}", file=sys.stderr)
        return 1

    print(f"parsed_frame_count: {frame_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
