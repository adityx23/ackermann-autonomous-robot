#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import os
import sys
import termios
import time
from dataclasses import asdict, fields, replace
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

DEFAULT_PORT = "/dev/c30d"
DEFAULT_BAUD = 115200
DEFAULT_DURATION_S = 5.0
DEFAULT_PRINT_EVERY = 10
DEFAULT_OUTPUT_DIR = Path("data/c30d_live")
READ_SIZE = 256
SERIAL_TIMEOUT_S = 0.1
BAUD_RATES = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
    460800: termios.B460800,
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read-only live monitor for candidate C30D feedback frames."
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


def extract_fixed_frames_from_buffer(buffer: bytearray, chunk: bytes) -> list[bytes]:
    from ackermann_robot.drivers.c30d_frames import FRAME_END, FRAME_LENGTH, FRAME_START

    buffer.extend(chunk)
    frames: list[bytes] = []

    while True:
        start = buffer.find(bytes([FRAME_START]))
        if start < 0:
            buffer.clear()
            return frames

        if start > 0:
            del buffer[:start]

        if len(buffer) < FRAME_LENGTH:
            return frames

        frame = bytes(buffer[:FRAME_LENGTH])
        if frame[FRAME_LENGTH - 1] == FRAME_END:
            frames.append(frame)
            del buffer[:FRAME_LENGTH]
            continue

        del buffer[0]


def resolve_output_path(output: Path | None, now: datetime | None = None) -> Path | None:
    if output is None:
        return None
    filename = output.name
    if not filename:
        timestamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
        filename = f"c30d_live_feedback_{timestamp}.csv"
    path = DEFAULT_OUTPUT_DIR / filename
    if path.suffix != ".csv":
        path = path.with_suffix(".csv")
    return path


def format_live_line(candidate) -> str:
    return (
        f"frame_index={candidate.frame_index} "
        f"forward_candidate={candidate.candidate_forward_motion} "
        f"yaw_candidate={candidate.candidate_yaw_motion} "
        f"imu_candidates=("
        f"{candidate.candidate_imu_12_13},"
        f"{candidate.candidate_imu_14_15},"
        f"{candidate.candidate_imu_16_17},"
        f"{candidate.candidate_imu_18_19}) "
        f"candidate_battery_mV={candidate.candidate_battery_mV} "
        f"checksum_candidate={candidate.checksum_candidate} "
        f"checksum_valid={candidate.checksum_valid}"
    )


def open_csv_writer(output_path: Path | None):
    if output_path is None:
        return None, None

    from ackermann_robot.drivers.c30d_feedback import C30DFeedbackCandidate

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_file = output_path.open("w", newline="")
    writer = csv.DictWriter(
        output_file,
        fieldnames=[field.name for field in fields(C30DFeedbackCandidate)],
    )
    writer.writeheader()
    return output_file, writer


def configure_serial_readonly(fd: int, baud: int) -> None:
    baud_constant = BAUD_RATES.get(baud)
    if baud_constant is None:
        raise ValueError(f"unsupported baud rate for read-only monitor: {baud}")

    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] |= termios.CLOCAL | termios.CREAD
    attrs[2] &= ~termios.PARENB
    attrs[2] &= ~termios.CSTOPB
    attrs[2] &= ~termios.CSIZE
    attrs[2] |= termios.CS8
    if hasattr(termios, "CRTSCTS"):
        attrs[2] &= ~termios.CRTSCTS
    attrs[3] = 0
    attrs[4] = baud_constant
    attrs[5] = baud_constant
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = max(1, int(SERIAL_TIMEOUT_S * 10))
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def open_readonly_serial_fd(port: str, baud: int) -> int:
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure_serial_readonly(fd, baud)
        os.set_blocking(fd, True)
    except Exception:
        os.close(fd)
        raise
    return fd


def monitor_feedback(
    port: str,
    baud: int,
    duration_s: float,
    output_path: Path | None,
    print_every: int,
) -> tuple[int, int]:
    from ackermann_robot.drivers.c30d_feedback import parse_feedback_candidates

    parsed_count = 0
    invalid_checksum_count = 0
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
                parsed_count += 1
                if not candidate.checksum_valid:
                    invalid_checksum_count += 1

                if writer is not None:
                    writer.writerow(asdict(candidate))

                if parsed_count == 1 or parsed_count % print_every == 0:
                    print(format_live_line(candidate))
    finally:
        if fd is not None:
            os.close(fd)
        if output_file is not None:
            output_file.close()

    return parsed_count, invalid_checksum_count


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

    print("READ-ONLY C30D feedback monitor. This script never writes to the serial port.")
    print(f"port: {args.port}")
    print(f"baud: {args.baud}")
    print(f"duration_s: {args.duration}")
    if output_path is not None:
        print(f"output_path: {output_path}")

    try:
        frame_count, invalid_checksum_count = monitor_feedback(
            port=args.port,
            baud=args.baud,
            duration_s=args.duration,
            output_path=output_path,
            print_every=args.print_every,
        )
    except OSError as exc:
        print(f"failed to read C30D feedback read-only: {exc}", file=sys.stderr)
        return 1

    print(f"parsed_frame_count: {frame_count}")
    print(f"invalid_checksum_count: {invalid_checksum_count}")
    if invalid_checksum_count:
        print("warning: invalid C30D feedback checksum frames observed", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
