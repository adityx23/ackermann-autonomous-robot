#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys

DEFAULT_PORT = "/dev/rplidar"
DEFAULT_BAUD = 460800


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Open and close the RPLIDAR serial port.")
    parser.add_argument("--port", default=DEFAULT_PORT, help="RPLIDAR serial device path.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="RPLIDAR serial baud rate.")
    return parser


def open_close_port(port: str, baud: int) -> None:
    import serial

    handle = serial.Serial(port=port, baudrate=baud, timeout=1.0)
    handle.close()


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        open_close_port(args.port, args.baud)
    except Exception as exc:
        print(f"Failed to open {args.port} at {args.baud}: {exc}", file=sys.stderr)
        return 1

    print(f"Opened and closed {args.port} at {args.baud}.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
