#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Protocol

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
SCRIPT_ROOT = REPO_ROOT / "scripts"
for path in (SRC_ROOT, SCRIPT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

DEFAULT_PORT = "/dev/c30d"
DEFAULT_BAUD = 115200
ZERO_NEUTRAL_FRAME = bytes.fromhex("7b 00 00 00 00 00 00 00 00 7b 7d")
REQUIRED_FLAGS = (
    "armed",
    "manual_enable",
    "wheels_lifted",
    "robot_restrained",
    "manual_power_cutoff_ready",
    "motor_enable_switch_reviewed",
    "i_understand_this_sends_a_real_serial_frame",
)


class SerialHandle(Protocol):
    def write(self, data: bytes) -> int: ...

    def flush(self) -> None: ...

    def close(self) -> None: ...


class SerialFactory(Protocol):
    def __call__(
        self,
        *,
        port: str,
        baudrate: int,
        timeout: float,
        write_timeout: float,
    ) -> SerialHandle: ...


@dataclass(frozen=True)
class ZeroFrameValidation:
    valid: bool
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class WriteResult:
    real_write_performed: bool
    bytes_written: int
    frame_hex: str


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Send the single hardcoded zero/neutral native C30D host command frame once. "
            "No speed, steering, target, or arbitrary packet inputs are accepted."
        )
    )
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--armed", action="store_true")
    parser.add_argument("--manual-enable", action="store_true")
    parser.add_argument("--wheels-lifted", action="store_true")
    parser.add_argument("--robot-restrained", action="store_true")
    parser.add_argument("--manual-power-cutoff-ready", action="store_true")
    parser.add_argument("--motor-enable-switch-reviewed", action="store_true")
    parser.add_argument("--i-understand-this-sends-a-real-serial-frame", action="store_true")
    parser.add_argument(
        "--preflight-results",
        type=Path,
        help="Optional JSON read-only preflight results for the internal readiness check.",
    )
    parser.add_argument("--preflight-duration", type=float, default=3.0)
    parser.add_argument(
        "--c30d-only-preflight",
        action="store_true",
        help="When running preflight, check C30D only and skip RPLIDAR/OAK.",
    )
    return parser


def missing_required_flags(args: argparse.Namespace) -> tuple[str, ...]:
    return tuple(name for name in REQUIRED_FLAGS if not getattr(args, name))


def build_zero_frame() -> bytes:
    from ackermann_robot.drivers.c30d_host_command_frame import (
        build_ackermann_host_command_frame,
    )

    return build_ackermann_host_command_frame(
        reserved_1=0,
        reserved_2=0,
        target_x=0,
        target_y=0,
        target_z=0,
    )


def validate_zero_frame(frame: bytes) -> ZeroFrameValidation:
    from ackermann_robot.drivers.c30d_checksum import xor_checksum

    reasons: list[str] = []
    if len(frame) != 11:
        reasons.append(f"expected_11_bytes_got_{len(frame)}")
    if len(frame) == 0 or frame[0] != 0x7B:
        reasons.append("byte_0_not_0x7b")
    if len(frame) <= 10 or frame[10] != 0x7D:
        reasons.append("byte_10_not_0x7d")
    if len(frame) <= 9:
        reasons.append("missing_checksum_byte")
    else:
        expected_checksum = xor_checksum(frame[:9])
        if frame[9] != expected_checksum:
            reasons.append("checksum_byte_9_not_xor_bytes_0_through_8")
    if len(frame) >= 9 and (
        frame[3:5] != b"\x00\x00" or frame[5:7] != b"\x00\x00" or frame[7:9] != b"\x00\x00"
    ):
        reasons.append("target_values_not_all_zero")
    if frame != ZERO_NEUTRAL_FRAME:
        reasons.append("frame_not_hardcoded_zero_neutral")
    return ZeroFrameValidation(valid=not reasons, reasons=tuple(reasons))


def run_internal_readiness(args: argparse.Namespace):
    import c30d_first_write_readiness as readiness

    try:
        preflight = (
            readiness.preflight_summary_from_json(args.preflight_results)
            if args.preflight_results is not None
            else readiness.run_readonly_preflight(args.preflight_duration, args.c30d_only_preflight)
        )
        threshold = readiness.load_warning_battery_threshold()
    except (OSError, ValueError) as exc:
        raise RuntimeError(f"failed to prepare readiness inputs: {exc}") from exc

    confirmations = {
        "wheels_lifted": args.wheels_lifted,
        "robot_restrained": args.robot_restrained,
        "manual_power_cutoff_ready": args.manual_power_cutoff_ready,
        "motor_enable_switch_reviewed": args.motor_enable_switch_reviewed,
        "i_understand_this_is_not_a_motor_test": True,
    }
    report = readiness.evaluate_readiness(confirmations, preflight, threshold)
    readiness.print_report(report)
    return report


def open_serial_handle(port: str, baud: int) -> SerialHandle:
    import serial

    return serial.Serial(port=port, baudrate=baud, timeout=1.0, write_timeout=1.0)


def write_validated_zero_frame_once(
    *,
    frame: bytes,
    port: str,
    baud: int,
    serial_factory: SerialFactory = open_serial_handle,
) -> WriteResult:
    validation = validate_zero_frame(frame)
    if not validation.valid:
        raise ValueError(f"invalid zero frame: {', '.join(validation.reasons)}")

    handle = serial_factory(port=port, baudrate=baud, timeout=1.0, write_timeout=1.0)
    try:
        bytes_written = handle.write(frame)
        handle.flush()
    finally:
        handle.close()
    return WriteResult(
        real_write_performed=True,
        bytes_written=bytes_written,
        frame_hex=frame.hex(" "),
    )


def print_final_result(result: WriteResult) -> None:
    print(f"real_write_performed: {str(result.real_write_performed).lower()}")
    print(f"bytes_written: {result.bytes_written}")
    print(f"frame_hex: {result.frame_hex}")
    print("warning: zero/neutral frame only, not a motor pulse")


def main(
    argv: list[str] | None = None,
    *,
    serial_factory: SerialFactory = open_serial_handle,
) -> int:
    args = build_parser().parse_args(argv)
    missing = missing_required_flags(args)
    if missing:
        print("refused: missing_required_safety_flags")
        print(f"missing: {', '.join(missing)}")
        print("real_write_performed: false")
        return 1

    frame = build_zero_frame()
    validation = validate_zero_frame(frame)
    print(f"frame_hex: {frame.hex(' ')}")
    if not validation.valid:
        print("refused: zero_frame_validation_failed")
        print(f"zero_frame_validation_reasons: {', '.join(validation.reasons)}")
        print("real_write_performed: false")
        return 1

    try:
        readiness_report = run_internal_readiness(args)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        print("real_write_performed: false")
        return 2

    if not readiness_report.readiness_allowed:
        print("refused: readiness_allowed_false")
        print("real_write_performed: false")
        return 1

    print("WARNING: this opens the real C30D serial port and sends one zero/neutral frame.")
    try:
        result = write_validated_zero_frame_once(
            frame=frame,
            port=args.port,
            baud=args.baud,
            serial_factory=serial_factory,
        )
    except ValueError as exc:
        print(f"refused: {exc}")
        print("real_write_performed: false")
        return 1
    print_final_result(result)
    return 0 if result.bytes_written == len(frame) else 1


if __name__ == "__main__":
    raise SystemExit(main())
