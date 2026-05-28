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
            "Build an offline unverified_hypothesis C30D-like 24-byte frame from "
            "payload bytes 1-21. This script never transmits bytes."
        )
    )
    parser.add_argument(
        "payload",
        nargs="*",
        help="Twenty-one positional hex payload bytes for frame bytes 1-21.",
    )
    parser.add_argument(
        "--payload-hex",
        help='Twenty-one hex payload bytes in one string, for example: "00 00 ...".',
    )
    return parser


def parse_hex_payload(tokens: list[str]) -> bytes:
    values: list[int] = []
    for token in tokens:
        for part in token.replace(",", " ").split():
            normalized = part.removeprefix("0x").removeprefix("0X")
            if len(normalized) != 2:
                raise ValueError(f"hex byte must contain exactly two hex digits: {part}")
            values.append(int(normalized, 16))
    return bytes(values)


def payload_tokens_from_args(args: argparse.Namespace) -> list[str]:
    if args.payload_hex is not None and args.payload:
        raise ValueError("use positional payload bytes or --payload-hex, not both")
    if args.payload_hex is not None:
        return [args.payload_hex]
    if args.payload:
        return args.payload
    raise ValueError("provide 21 payload bytes positionally or with --payload-hex")


def main(argv: list[str] | None = None) -> int:
    from ackermann_robot.drivers.c30d_command_hypotheses import build_command_hypothesis

    args = build_parser().parse_args(argv)
    try:
        payload = parse_hex_payload(payload_tokens_from_args(args))
        hypothesis_frame = build_command_hypothesis(payload)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    print("label: unverified_hypothesis")
    print(f"full_frame_hex: {hypothesis_frame.frame.hex(' ')}")
    print(f"checksum_byte: 0x{hypothesis_frame.checksum:02x}")
    print("protocol_known: false")
    print("transmit_allowed: false")
    print("warning: offline hypothesis only; this must not be sent to C30D")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
