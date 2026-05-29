#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
import time
from typing import Callable, Protocol

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
SCRIPT_ROOT = REPO_ROOT / "scripts"
for path in (SRC_ROOT, SCRIPT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

DEFAULT_PORT = "/dev/c30d"
DEFAULT_BAUD = 115200
DEFAULT_PREFLIGHT_DURATION_S = 5.0
DEFAULT_TARGET_X = 0.03
MAX_ABS_TARGET_X = 0.05
DEFAULT_PULSE_DURATION_S = 0.10
MAX_PULSE_DURATION_S = 0.15
ZERO_SETTLE_S = 0.05
REQUIRED_REAL_WRITE_FLAGS = (
    "armed",
    "manual_enable",
    "wheels_lifted",
    "robot_restrained",
    "manual_power_cutoff_ready",
    "motor_enable_switch_reviewed",
    "i_understand_this_may_spin_the_wheels",
    "execute_real_pulse",
)


class SerialHandle(Protocol):
    def write(self, data: bytes) -> int: ...

    def flush(self) -> None: ...

    def close(self) -> None: ...


class SerialFactory(Protocol):
    def __call__(self, port: str, baud: int) -> SerialHandle: ...


SleepFn = Callable[[float], None]


@dataclass(frozen=True)
class PulseFrames:
    zero_frame: bytes
    pulse_frame: bytes
    pulse_target_x: float
    pulse_target_x_scaled: int
    pulse_duration_s: float


@dataclass(frozen=True)
class PulseWriteResult:
    real_write_performed: bool
    bytes_written_total: int
    pulse_target_x: float
    pulse_duration_s: float


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Dry-run by default. Optionally send one extremely constrained native C30D "
            "tiny forward pulse sequence: zero, pulse, zero, zero."
        )
    )
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--target-x", type=float, default=DEFAULT_TARGET_X)
    parser.add_argument("--duration", type=float, default=DEFAULT_PULSE_DURATION_S)
    parser.add_argument("--armed", action="store_true")
    parser.add_argument("--manual-enable", action="store_true")
    parser.add_argument("--wheels-lifted", action="store_true")
    parser.add_argument("--robot-restrained", action="store_true")
    parser.add_argument("--manual-power-cutoff-ready", action="store_true")
    parser.add_argument("--motor-enable-switch-reviewed", action="store_true")
    parser.add_argument("--i-understand-this-may-spin-the-wheels", action="store_true")
    parser.add_argument("--execute-real-pulse", action="store_true")
    parser.add_argument(
        "--preflight-results",
        type=Path,
        help="Optional JSON read-only preflight results for the internal readiness check.",
    )
    parser.add_argument("--preflight-duration", type=float, default=DEFAULT_PREFLIGHT_DURATION_S)
    parser.add_argument(
        "--c30d-only-preflight",
        dest="preflight_mode",
        action="store_const",
        const="c30d_only",
        default="c30d_only",
        help="Check C30D feedback, data dirs, and battery only. This is the default.",
    )
    parser.add_argument(
        "--full-sensor-preflight",
        dest="preflight_mode",
        action="store_const",
        const="full_sensor",
        help="Check C30D plus RPLIDAR and OAK before the real pulse.",
    )
    return parser


def missing_real_write_flags(args: argparse.Namespace) -> tuple[str, ...]:
    return tuple(name for name in REQUIRED_REAL_WRITE_FLAGS if not getattr(args, name))


def validate_limits(target_x: float, duration_s: float) -> tuple[str, ...]:
    reasons: list[str] = []
    if target_x <= 0.0:
        reasons.append("target_x_must_be_positive_forward_only")
    if abs(target_x) > MAX_ABS_TARGET_X:
        reasons.append("target_x_exceeds_0.05_limit")
    if duration_s <= 0.0:
        reasons.append("duration_must_be_positive")
    if duration_s > MAX_PULSE_DURATION_S:
        reasons.append("duration_exceeds_0.15_limit")
    return tuple(reasons)


def build_pulse_frames(target_x: float, duration_s: float) -> PulseFrames:
    from ackermann_robot.drivers.c30d_host_command_frame import (
        build_ackermann_host_command_frame,
        scale_documentation_candidate,
    )

    limit_reasons = validate_limits(target_x, duration_s)
    if limit_reasons:
        raise ValueError(", ".join(limit_reasons))

    scaled_target_x = scale_documentation_candidate(target_x)
    zero_frame = build_ackermann_host_command_frame(
        reserved_1=0,
        reserved_2=0,
        target_x=0,
        target_y=0,
        target_z=0,
    )
    pulse_frame = build_ackermann_host_command_frame(
        reserved_1=0,
        reserved_2=0,
        target_x=scaled_target_x,
        target_y=0,
        target_z=0,
    )
    validate_frame(zero_frame, expect_zero_target_x=True)
    validate_frame(pulse_frame, expect_zero_target_x=False)
    return PulseFrames(
        zero_frame=zero_frame,
        pulse_frame=pulse_frame,
        pulse_target_x=target_x,
        pulse_target_x_scaled=scaled_target_x,
        pulse_duration_s=duration_s,
    )


def validate_frame(frame: bytes, *, expect_zero_target_x: bool) -> None:
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
    elif frame[9] != xor_checksum(frame[:9]):
        reasons.append("checksum_byte_9_not_xor_bytes_0_through_8")
    if len(frame) >= 9 and (frame[5:7] != b"\x00\x00" or frame[7:9] != b"\x00\x00"):
        reasons.append("target_y_or_target_z_not_zero")
    if expect_zero_target_x and len(frame) >= 5 and frame[3:5] != b"\x00\x00":
        reasons.append("zero_frame_target_x_not_zero")
    if not expect_zero_target_x and len(frame) >= 5:
        target_x = int.from_bytes(frame[3:5], "big", signed=True)
        if target_x <= 0:
            reasons.append("pulse_frame_target_x_not_positive")
        if abs(target_x) > int(MAX_ABS_TARGET_X * 1000):
            reasons.append("pulse_frame_target_x_exceeds_scaled_limit")
    if reasons:
        raise ValueError(", ".join(reasons))


def run_internal_readiness(args: argparse.Namespace):
    import c30d_first_write_readiness as readiness

    try:
        preflight = (
            readiness.preflight_summary_from_json(
                args.preflight_results,
                mode=args.preflight_mode,
                duration_s=args.preflight_duration,
            )
            if args.preflight_results is not None
            else readiness.run_readonly_preflight(args.preflight_duration, args.preflight_mode)
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


def write_frame(handle: SerialHandle, frame: bytes) -> int:
    print(f"frame_hex: {frame.hex(' ')}")
    bytes_written = handle.write(frame)
    handle.flush()
    return bytes_written


def write_pulse_sequence(
    *,
    frames: PulseFrames,
    port: str,
    baud: int,
    serial_factory: SerialFactory = open_serial_handle,
    sleep_fn: SleepFn = time.sleep,
) -> PulseWriteResult:
    handle = serial_factory(port, baud)
    total = 0
    try:
        total += write_frame(handle, frames.zero_frame)
        sleep_fn(ZERO_SETTLE_S)
        total += write_frame(handle, frames.pulse_frame)
        sleep_fn(frames.pulse_duration_s)
        total += write_frame(handle, frames.zero_frame)
        sleep_fn(ZERO_SETTLE_S)
        total += write_frame(handle, frames.zero_frame)
    finally:
        handle.close()
    return PulseWriteResult(
        real_write_performed=True,
        bytes_written_total=total,
        pulse_target_x=frames.pulse_target_x,
        pulse_duration_s=frames.pulse_duration_s,
    )


def print_planned_frames(frames: PulseFrames) -> None:
    print(f"zero_frame_hex: {frames.zero_frame.hex(' ')}")
    print(f"pulse_frame_hex: {frames.pulse_frame.hex(' ')}")
    print(f"pulse_target_x_scaled_int16: {frames.pulse_target_x_scaled}")


def print_result(result: PulseWriteResult) -> None:
    print(f"real_write_performed: {str(result.real_write_performed).lower()}")
    print(f"bytes_written_total: {result.bytes_written_total}")
    print(f"pulse_target_x: {result.pulse_target_x:g}")
    print(f"pulse_duration_s: {result.pulse_duration_s:g}")
    print("warning: wheels may spin briefly")


def main(
    argv: list[str] | None = None,
    *,
    serial_factory: SerialFactory = open_serial_handle,
    sleep_fn: SleepFn = time.sleep,
) -> int:
    args = build_parser().parse_args(argv)
    try:
        frames = build_pulse_frames(args.target_x, args.duration)
    except ValueError as exc:
        print(f"refused: {exc}")
        print("real_write_performed: false")
        return 1

    print_planned_frames(frames)
    print(f"pulse_target_x: {frames.pulse_target_x:g}")
    print(f"pulse_duration_s: {frames.pulse_duration_s:g}")
    print("warning: wheels may spin briefly")

    if not args.execute_real_pulse:
        print("dry_run: true")
        print("refused: execute_real_pulse_required_for_real_write")
        print("real_write_performed: false")
        print("bytes_written_total: 0")
        return 0

    missing = missing_real_write_flags(args)
    if missing:
        print("refused: missing_required_safety_flags")
        print(f"missing: {', '.join(missing)}")
        print("real_write_performed: false")
        print("bytes_written_total: 0")
        return 1

    try:
        readiness_report = run_internal_readiness(args)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        print("real_write_performed: false")
        print("bytes_written_total: 0")
        return 2

    if not readiness_report.readiness_allowed:
        print("refused: readiness_allowed_false")
        print("real_write_performed: false")
        print("bytes_written_total: 0")
        return 1

    print("WARNING: this may briefly spin the wheels. Keep wheels lifted and power cutoff ready.")
    result = write_pulse_sequence(
        frames=frames,
        port=args.port,
        baud=args.baud,
        serial_factory=serial_factory,
        sleep_fn=sleep_fn,
    )
    print_result(result)
    return 0 if result.bytes_written_total == 44 else 1


if __name__ == "__main__":
    raise SystemExit(main())
