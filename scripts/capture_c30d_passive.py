#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

DEFAULT_PORT = "/dev/c30d"
DEFAULT_BAUD = 115200
DEFAULT_DURATION_S = 5.0
DEFAULT_OUTPUT_DIR = Path("data/c30d_captures")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Capture passive C30D binary data without writes.")
    parser.add_argument("--port", default=DEFAULT_PORT, help="C30D serial device path.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="C30D serial baud rate.")
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_S,
        help="Passive capture duration in seconds.",
    )
    parser.add_argument("--output", type=Path, default=None, help="Output .bin path.")
    return parser


def default_output_path(now: datetime | None = None) -> Path:
    timestamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
    return DEFAULT_OUTPUT_DIR / f"c30d_capture_{timestamp}.bin"


def capture_passive(port: str, baud: int, duration_s: float, output_path: Path) -> int:
    import serial

    output_path.parent.mkdir(parents=True, exist_ok=True)
    deadline_s = time.monotonic() + duration_s
    total_bytes = 0

    with serial.Serial(port=port, baudrate=baud, timeout=0.1) as handle:
        with output_path.open("wb") as output:
            while time.monotonic() < deadline_s:
                chunk = handle.read(4096)
                if not chunk:
                    continue
                output.write(chunk)
                total_bytes += len(chunk)

    return total_bytes


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.duration < 0:
        print("--duration must be non-negative.", file=sys.stderr)
        return 2

    output_path = args.output or default_output_path()
    print("Passive C30D capture only: this script does not write bytes to the controller.")

    try:
        total_bytes = capture_passive(args.port, args.baud, args.duration, output_path)
    except Exception as exc:
        print(f"Failed to capture from {args.port} at {args.baud}: {exc}", file=sys.stderr)
        return 1

    print(f"Captured {total_bytes} bytes to {output_path}.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
