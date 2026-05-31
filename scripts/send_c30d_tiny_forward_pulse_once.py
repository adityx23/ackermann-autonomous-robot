#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
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
DEFAULT_RESERVED_1 = 0x00
DEFAULT_RESERVED_2 = 0x00
ZERO_SETTLE_S = 0.05
BASELINE_FEEDBACK_S = 0.20
POST_FEEDBACK_S = 0.20
READ_SIZE = 256
MOVEMENT_FORWARD_DELTA_THRESHOLD = 1
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

    def read(self, size: int) -> bytes: ...

    def flush(self) -> None: ...

    def close(self) -> None: ...


class SerialFactory(Protocol):
    def __call__(self, port: str, baud: int) -> SerialHandle: ...


SleepFn = Callable[[float], None]


@dataclass(frozen=True)
class PulseFrames:
    zero_frame: bytes
    pulse_frame: bytes
    pulse_reserved_1: int
    pulse_reserved_2: int
    pulse_target_x: float
    pulse_target_x_scaled: int
    pulse_duration_s: float


@dataclass(frozen=True)
class FeedbackLogRow:
    monotonic_timestamp: float
    phase: str
    frame_index: int
    forward_candidate: int
    yaw_candidate: int
    candidate_battery_mV: int
    checksum_valid: bool
    raw_frame_hex: str


@dataclass(frozen=True)
class FeedbackSummary:
    max_abs_forward_candidate_baseline: int
    max_abs_forward_candidate_pulse_post: int
    max_abs_yaw_candidate: int
    invalid_checksum_count: int
    movement_feedback_detected: bool


@dataclass(frozen=True)
class PulseWriteResult:
    real_write_performed: bool
    bytes_written_total: int
    pulse_target_x: float
    pulse_duration_s: float
    feedback_summary: FeedbackSummary


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
    parser.add_argument(
        "--reserved-1", type=parse_reserved_control_byte, default=DEFAULT_RESERVED_1
    )
    parser.add_argument(
        "--reserved-2", type=parse_reserved_control_byte, default=DEFAULT_RESERVED_2
    )
    parser.add_argument("--feedback-output", type=Path)
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


def parse_reserved_control_byte(value: str) -> int:
    try:
        parsed = int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("reserved byte must be 0x00 or 0x01") from exc
    if parsed not in (0x00, 0x01):
        raise argparse.ArgumentTypeError("reserved byte must be 0x00 or 0x01")
    return parsed


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


def build_pulse_frames(
    target_x: float,
    duration_s: float,
    reserved_1: int = DEFAULT_RESERVED_1,
    reserved_2: int = DEFAULT_RESERVED_2,
) -> PulseFrames:
    from ackermann_robot.drivers.c30d_host_command_frame import (
        build_ackermann_host_command_frame,
        scale_documentation_candidate,
    )

    limit_reasons = validate_limits(target_x, duration_s)
    if limit_reasons:
        raise ValueError(", ".join(limit_reasons))
    if reserved_1 not in (0x00, 0x01):
        raise ValueError("reserved_1_must_be_0x00_or_0x01")
    if reserved_2 not in (0x00, 0x01):
        raise ValueError("reserved_2_must_be_0x00_or_0x01")

    scaled_target_x = scale_documentation_candidate(target_x)
    zero_frame = build_ackermann_host_command_frame(
        reserved_1=0x00,
        reserved_2=0x00,
        target_x=0,
        target_y=0,
        target_z=0,
    )
    pulse_frame = build_ackermann_host_command_frame(
        reserved_1=reserved_1,
        reserved_2=reserved_2,
        target_x=scaled_target_x,
        target_y=0,
        target_z=0,
    )
    validate_frame(zero_frame, expect_zero_target_x=True)
    validate_frame(pulse_frame, expect_zero_target_x=False)
    return PulseFrames(
        zero_frame=zero_frame,
        pulse_frame=pulse_frame,
        pulse_reserved_1=reserved_1,
        pulse_reserved_2=reserved_2,
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

    return serial.Serial(port=port, baudrate=baud, timeout=0.02, write_timeout=1.0)


def feedback_row_from_candidate(candidate, *, phase: str, timestamp: float) -> FeedbackLogRow:
    return FeedbackLogRow(
        monotonic_timestamp=timestamp,
        phase=phase,
        frame_index=candidate.frame_index,
        forward_candidate=candidate.candidate_forward_motion,
        yaw_candidate=candidate.candidate_yaw_motion,
        candidate_battery_mV=candidate.candidate_battery_mV,
        checksum_valid=candidate.checksum_valid,
        raw_frame_hex=candidate.raw_frame_hex,
    )


def read_feedback_once(
    handle: SerialHandle,
    *,
    buffer: bytearray,
    phase: str,
    rows: list[FeedbackLogRow],
    frame_index: int,
    clock: Callable[[], float] = time.monotonic,
) -> int:
    from ackermann_robot.drivers.c30d_feedback import parse_feedback_candidates
    from monitor_c30d_feedback_readonly import extract_fixed_frames_from_buffer

    chunk = handle.read(READ_SIZE)
    if not chunk:
        return frame_index
    for frame in extract_fixed_frames_from_buffer(buffer, chunk):
        candidate = parse_feedback_candidates([frame])[0]
        candidate = candidate.__class__(
            frame_index=frame_index,
            candidate_forward_motion=candidate.candidate_forward_motion,
            candidate_yaw_motion=candidate.candidate_yaw_motion,
            candidate_imu_12_13=candidate.candidate_imu_12_13,
            candidate_imu_14_15=candidate.candidate_imu_14_15,
            candidate_imu_16_17=candidate.candidate_imu_16_17,
            candidate_imu_18_19=candidate.candidate_imu_18_19,
            candidate_battery_mV=candidate.candidate_battery_mV,
            checksum_candidate=candidate.checksum_candidate,
            checksum_valid=candidate.checksum_valid,
            raw_frame_hex=candidate.raw_frame_hex,
        )
        rows.append(feedback_row_from_candidate(candidate, phase=phase, timestamp=clock()))
        frame_index += 1
    return frame_index


def capture_feedback_phase(
    handle: SerialHandle,
    *,
    buffer: bytearray,
    phase: str,
    duration_s: float,
    rows: list[FeedbackLogRow],
    frame_index: int,
    sleep_fn: SleepFn = time.sleep,
    clock: Callable[[], float] = time.monotonic,
) -> int:
    deadline = clock() + duration_s
    max_iterations = max(1, int(duration_s / 0.005) + 2)
    iterations = 0
    while clock() < deadline and iterations < max_iterations:
        frame_index = read_feedback_once(
            handle,
            buffer=buffer,
            phase=phase,
            rows=rows,
            frame_index=frame_index,
            clock=clock,
        )
        iterations += 1
        sleep_fn(0.005)
    return frame_index


def write_feedback_csv(path: Path, rows: list[FeedbackLogRow]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as output_file:
        writer = csv.DictWriter(
            output_file,
            fieldnames=(
                "monotonic_timestamp",
                "phase",
                "frame_index",
                "forward_candidate",
                "yaw_candidate",
                "candidate_battery_mV",
                "checksum_valid",
                "raw_frame_hex",
            ),
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    "monotonic_timestamp": f"{row.monotonic_timestamp:.6f}",
                    "phase": row.phase,
                    "frame_index": row.frame_index,
                    "forward_candidate": row.forward_candidate,
                    "yaw_candidate": row.yaw_candidate,
                    "candidate_battery_mV": row.candidate_battery_mV,
                    "checksum_valid": str(row.checksum_valid).lower(),
                    "raw_frame_hex": row.raw_frame_hex,
                }
            )


def summarize_feedback(rows: list[FeedbackLogRow]) -> FeedbackSummary:
    baseline_forward = [abs(row.forward_candidate) for row in rows if row.phase == "baseline"]
    pulse_post_forward = [
        abs(row.forward_candidate) for row in rows if row.phase in {"pulse", "zero_after", "post"}
    ]
    max_abs_forward_candidate_baseline = max(baseline_forward, default=0)
    max_abs_forward_candidate_pulse_post = max(pulse_post_forward, default=0)
    max_abs_yaw_candidate = max((abs(row.yaw_candidate) for row in rows), default=0)
    invalid_checksum_count = sum(1 for row in rows if not row.checksum_valid)
    movement_feedback_detected = (
        max_abs_forward_candidate_pulse_post
        > max_abs_forward_candidate_baseline + MOVEMENT_FORWARD_DELTA_THRESHOLD
    )
    return FeedbackSummary(
        max_abs_forward_candidate_baseline=max_abs_forward_candidate_baseline,
        max_abs_forward_candidate_pulse_post=max_abs_forward_candidate_pulse_post,
        max_abs_yaw_candidate=max_abs_yaw_candidate,
        invalid_checksum_count=invalid_checksum_count,
        movement_feedback_detected=movement_feedback_detected,
    )


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
    feedback_output: Path | None,
    serial_factory: SerialFactory = open_serial_handle,
    sleep_fn: SleepFn = time.sleep,
    clock: Callable[[], float] = time.monotonic,
) -> PulseWriteResult:
    handle = serial_factory(port, baud)
    total = 0
    rows: list[FeedbackLogRow] = []
    buffer = bytearray()
    frame_index = 0
    try:
        frame_index = capture_feedback_phase(
            handle,
            buffer=buffer,
            phase="baseline",
            duration_s=BASELINE_FEEDBACK_S,
            rows=rows,
            frame_index=frame_index,
            sleep_fn=sleep_fn,
            clock=clock,
        )
        total += write_frame(handle, frames.zero_frame)
        frame_index = capture_feedback_phase(
            handle,
            buffer=buffer,
            phase="zero_before",
            duration_s=ZERO_SETTLE_S,
            rows=rows,
            frame_index=frame_index,
            sleep_fn=sleep_fn,
            clock=clock,
        )
        total += write_frame(handle, frames.pulse_frame)
        frame_index = capture_feedback_phase(
            handle,
            buffer=buffer,
            phase="pulse",
            duration_s=frames.pulse_duration_s,
            rows=rows,
            frame_index=frame_index,
            sleep_fn=sleep_fn,
            clock=clock,
        )
        total += write_frame(handle, frames.zero_frame)
        frame_index = capture_feedback_phase(
            handle,
            buffer=buffer,
            phase="zero_after",
            duration_s=ZERO_SETTLE_S,
            rows=rows,
            frame_index=frame_index,
            sleep_fn=sleep_fn,
            clock=clock,
        )
        total += write_frame(handle, frames.zero_frame)
        capture_feedback_phase(
            handle,
            buffer=buffer,
            phase="post",
            duration_s=POST_FEEDBACK_S,
            rows=rows,
            frame_index=frame_index,
            sleep_fn=sleep_fn,
            clock=clock,
        )
    finally:
        handle.close()

    if feedback_output is not None:
        write_feedback_csv(feedback_output, rows)
    return PulseWriteResult(
        real_write_performed=True,
        bytes_written_total=total,
        pulse_target_x=frames.pulse_target_x,
        pulse_duration_s=frames.pulse_duration_s,
        feedback_summary=summarize_feedback(rows),
    )


def print_planned_frames(frames: PulseFrames) -> None:
    print(f"pulse_reserved_1: 0x{frames.pulse_reserved_1:02x}")
    print(f"pulse_reserved_2: 0x{frames.pulse_reserved_2:02x}")
    print(f"safe_zero_frame_hex: {frames.zero_frame.hex(' ')}")
    print(f"pulse_frame_hex: {frames.pulse_frame.hex(' ')}")
    print(f"pulse_target_x_scaled_int16: {frames.pulse_target_x_scaled}")


def print_result(result: PulseWriteResult) -> None:
    print(f"real_write_performed: {str(result.real_write_performed).lower()}")
    print(f"bytes_written_total: {result.bytes_written_total}")
    print(f"pulse_target_x: {result.pulse_target_x:g}")
    print(f"pulse_duration_s: {result.pulse_duration_s:g}")
    print(
        "max_abs_forward_candidate during baseline: "
        f"{result.feedback_summary.max_abs_forward_candidate_baseline}"
    )
    print(
        "max_abs_forward_candidate during pulse/post: "
        f"{result.feedback_summary.max_abs_forward_candidate_pulse_post}"
    )
    print(f"max_abs_yaw_candidate: {result.feedback_summary.max_abs_yaw_candidate}")
    print(f"invalid_checksum_count: {result.feedback_summary.invalid_checksum_count}")
    print(
        "movement_feedback_detected: "
        f"{str(result.feedback_summary.movement_feedback_detected).lower()}"
    )
    print("warning: wheels may spin briefly")


def main(
    argv: list[str] | None = None,
    *,
    serial_factory: SerialFactory = open_serial_handle,
    sleep_fn: SleepFn = time.sleep,
    clock: Callable[[], float] = time.monotonic,
) -> int:
    args = build_parser().parse_args(argv)
    try:
        frames = build_pulse_frames(
            args.target_x,
            args.duration,
            reserved_1=args.reserved_1,
            reserved_2=args.reserved_2,
        )
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
        feedback_output=args.feedback_output,
        serial_factory=serial_factory,
        sleep_fn=sleep_fn,
        clock=clock,
    )
    print_result(result)
    return 0 if result.bytes_written_total == 44 else 1


if __name__ == "__main__":
    raise SystemExit(main())
