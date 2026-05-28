#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

REQUIRED_FLAG_NAMES = (
    "armed",
    "wheels_lifted",
    "manual_enable",
    "i_understand_risk",
)


@dataclass(frozen=True)
class PacketShapeValidation:
    valid: bool
    reasons: tuple[str, ...]
    checksum_expected: int | None
    checksum_actual: int | None
    frame_type: str


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Guarded future C30D write-test harness. It validates packet shape only and "
            "does not transmit bytes."
        )
    )
    parser.add_argument("--armed", action="store_true")
    parser.add_argument("--wheels-lifted", action="store_true")
    parser.add_argument("--manual-enable", action="store_true")
    parser.add_argument("--i-understand-risk", action="store_true")
    parser.add_argument("--packet-hex", help="Hex bytes to validate, not transmit.")
    return parser


def parse_hex_bytes(packet_hex: str) -> bytes:
    values: list[int] = []
    for part in packet_hex.replace(",", " ").split():
        normalized = part.removeprefix("0x").removeprefix("0X")
        if len(normalized) != 2:
            raise ValueError(f"hex byte must contain exactly two hex digits: {part}")
        values.append(int(normalized, 16))
    return bytes(values)


def _validate_delimited_xor_frame(
    packet: bytes,
    *,
    expected_length: int,
    checksum_index: int,
    checksum_end_exclusive: int,
    frame_type: str,
) -> PacketShapeValidation:
    from ackermann_robot.drivers.c30d_checksum import xor_checksum

    reasons: list[str] = []
    checksum_expected: int | None = None
    checksum_actual: int | None = packet[checksum_index] if len(packet) > checksum_index else None

    if len(packet) != expected_length:
        reasons.append(f"expected_{expected_length}_bytes_got_{len(packet)}")
    if len(packet) > 0 and packet[0] != 0x7B:
        reasons.append("byte_0_not_0x7b")
    elif len(packet) == 0:
        reasons.append("missing_start_byte")

    end_index = expected_length - 1
    if len(packet) > end_index and packet[end_index] != 0x7D:
        reasons.append(f"byte_{end_index}_not_0x7d")
    elif len(packet) <= end_index:
        reasons.append("missing_end_byte")

    if len(packet) > checksum_index:
        checksum_expected = xor_checksum(packet[:checksum_end_exclusive])
        if packet[checksum_index] != checksum_expected:
            reasons.append(
                f"checksum_byte_{checksum_index}_not_xor_bytes_0_through_{checksum_end_exclusive - 1}"
            )
    else:
        reasons.append("missing_checksum_byte")

    return PacketShapeValidation(
        valid=not reasons,
        reasons=tuple(reasons),
        checksum_expected=checksum_expected,
        checksum_actual=checksum_actual,
        frame_type=frame_type,
    )


def validate_packet_shape(packet: bytes) -> PacketShapeValidation:
    if len(packet) == 11:
        return _validate_delimited_xor_frame(
            packet,
            expected_length=11,
            checksum_index=9,
            checksum_end_exclusive=9,
            frame_type="ros_command_candidate_11_byte",
        )
    return _validate_delimited_xor_frame(
        packet,
        expected_length=24,
        checksum_index=22,
        checksum_end_exclusive=22,
        frame_type="feedback_like_hypothesis_24_byte",
    )


def missing_guard_names(args: argparse.Namespace) -> list[str]:
    missing = [name for name in REQUIRED_FLAG_NAMES if not getattr(args, name)]
    if args.packet_hex is None:
        missing.append("packet_hex")
    return missing


def format_bool(value: bool) -> str:
    return str(value).lower()


def print_safety_checklist(args: argparse.Namespace) -> None:
    print("safety_checklist:")
    print(f"  armed: {format_bool(args.armed)}")
    print(f"  wheels_lifted: {format_bool(args.wheels_lifted)}")
    print(f"  manual_enable: {format_bool(args.manual_enable)}")
    print(f"  i_understand_risk: {format_bool(args.i_understand_risk)}")
    print(f"  packet_hex_provided: {format_bool(args.packet_hex is not None)}")


def print_write_disabled_status() -> None:
    print("serial_write_allowed: false")
    print("real_write_disabled_in_code: true")
    print("no bytes sent")


def print_packet_validation(validation: PacketShapeValidation) -> None:
    print(f"packet_valid: {format_bool(validation.valid)}")
    print(f"packet_frame_type: {validation.frame_type}")
    print(
        "packet_validation_reasons: "
        f"{', '.join(validation.reasons) if validation.reasons else 'ok'}"
    )
    expected = (
        "none" if validation.checksum_expected is None else f"0x{validation.checksum_expected:02x}"
    )
    actual = "none" if validation.checksum_actual is None else f"0x{validation.checksum_actual:02x}"
    print(f"checksum_expected: {expected}")
    print(f"checksum_actual: {actual}")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    print("C30D guarded write-test harness: validation only, no transmission.")
    print_safety_checklist(args)
    print_write_disabled_status()

    missing = missing_guard_names(args)
    if missing:
        print("refused: missing_required_safety_inputs")
        print(f"missing: {', '.join(missing)}")
        return 1

    try:
        packet = parse_hex_bytes(args.packet_hex)
    except ValueError as exc:
        print("refused: invalid_packet_hex")
        print(f"packet_hex_error: {exc}")
        print_packet_validation(
            PacketShapeValidation(
                valid=False,
                reasons=("invalid_packet_hex",),
                checksum_expected=None,
                checksum_actual=None,
                frame_type="unknown",
            )
        )
        return 1

    validation = validate_packet_shape(packet)
    print_packet_validation(validation)
    print("refused: real_write_disabled_in_code")
    return 0 if validation.valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
