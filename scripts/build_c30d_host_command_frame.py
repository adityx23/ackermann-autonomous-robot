#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Build an offline Wheeltec-documented 11-byte C30D native host command "
            "frame candidate. This script never transmits bytes."
        )
    )
    parser.add_argument("--reserved-1", type=lambda value: int(value, 0), default=0)
    parser.add_argument("--reserved-2", type=lambda value: int(value, 0), default=0)
    parser.add_argument("--target-x", type=float, default=0.0)
    parser.add_argument("--target-z", type=float, default=0.0)
    parser.add_argument(
        "--target-y",
        type=float,
        default=0.0,
        help="Ackermann robots do not support Y-axis movement; leave this at zero.",
    )
    parser.add_argument(
        "--raw-int16",
        action="store_true",
        help="Treat target values as already-scaled signed int16 integers.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    from ackermann_robot.drivers.c30d_host_command_frame import (
        PROTOCOL_SOURCE,
        SCALING_LABEL,
        build_ackermann_host_command_frame_from_floats,
        build_host_command_candidate,
    )

    args = build_parser().parse_args(argv)
    try:
        if args.raw_int16:
            candidate = build_host_command_candidate(
                reserved_1=args.reserved_1,
                reserved_2=args.reserved_2,
                target_x=int(args.target_x),
                target_y=int(args.target_y),
                target_z=int(args.target_z),
            )
        else:
            candidate = build_ackermann_host_command_frame_from_floats(
                reserved_1=args.reserved_1,
                reserved_2=args.reserved_2,
                target_x=args.target_x,
                target_y=args.target_y,
                target_z=args.target_z,
            )
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    print(f"protocol_source: {PROTOCOL_SOURCE}")
    print(f"scaling: {SCALING_LABEL}")
    print(f"frame_hex: {candidate.frame.hex(' ')}")
    print(f"checksum: 0x{candidate.checksum:02x}")
    print("transmit_allowed: false")
    print("real_write_disabled: true")
    print("warning: native host-to-C30D serial frame candidate only; this must not be sent yet")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
