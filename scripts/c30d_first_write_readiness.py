#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import sys
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
SCRIPT_ROOT = REPO_ROOT / "scripts"
for path in (SRC_ROOT, SCRIPT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

REQUIRED_CONFIRMATIONS = (
    "wheels_lifted",
    "robot_restrained",
    "manual_power_cutoff_ready",
    "motor_enable_switch_reviewed",
    "i_understand_this_is_not_a_motor_test",
)
DEFAULT_PREFLIGHT_DURATION_S = 5.0
PREFLIGHT_MODE_C30D_ONLY = "c30d_only"
PREFLIGHT_MODE_FULL_SENSOR = "full_sensor"


@dataclass(frozen=True)
class PreflightSummary:
    passed: bool
    candidate_battery_mV: float | int | None
    frame_rate_hz: float | None = None
    frame_rate_threshold_hz: float | None = None
    invalid_checksum_count: int | None = None
    battery_warning_reasons: tuple[str, ...] = ()
    battery_block_reasons: tuple[str, ...] = ()
    mode: str = PREFLIGHT_MODE_C30D_ONLY
    duration_s: float = DEFAULT_PREFLIGHT_DURATION_S


@dataclass(frozen=True)
class ZeroFrameValidation:
    valid: bool
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class ReadinessReport:
    readiness_allowed: bool
    reasons: tuple[str, ...]
    preflight: PreflightSummary
    zero_frame: bytes
    zero_frame_validation: ZeroFrameValidation
    warning_battery_mV: int
    preflight_mode: str
    preflight_duration_s: float


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only readiness checker for a native C30D zero-frame first-write experiment. "
            "It never sends bytes."
        )
    )
    parser.add_argument("--wheels-lifted", action="store_true")
    parser.add_argument("--robot-restrained", action="store_true")
    parser.add_argument("--manual-power-cutoff-ready", action="store_true")
    parser.add_argument("--motor-enable-switch-reviewed", action="store_true")
    parser.add_argument("--i-understand-this-is-not-a-motor-test", action="store_true")
    parser.add_argument(
        "--preflight-results",
        type=Path,
        help="Optional JSON file containing read-only preflight results to consume instead of running preflight.",
    )
    parser.add_argument("--preflight-duration", type=float, default=DEFAULT_PREFLIGHT_DURATION_S)
    parser.add_argument(
        "--c30d-only-preflight",
        dest="preflight_mode",
        action="store_const",
        const=PREFLIGHT_MODE_C30D_ONLY,
        default=PREFLIGHT_MODE_C30D_ONLY,
        help="Check C30D feedback, data dirs, and battery only. This is the default.",
    )
    parser.add_argument(
        "--full-sensor-preflight",
        dest="preflight_mode",
        action="store_const",
        const=PREFLIGHT_MODE_FULL_SENSOR,
        help="Check C30D plus RPLIDAR and OAK manually before reporting readiness.",
    )
    return parser


def missing_confirmations(args: argparse.Namespace) -> tuple[str, ...]:
    return tuple(name for name in REQUIRED_CONFIRMATIONS if not getattr(args, name))


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
        expected = xor_checksum(frame[:9])
        if frame[9] != expected:
            reasons.append("checksum_byte_9_not_xor_bytes_0_through_8")
    return ZeroFrameValidation(valid=not reasons, reasons=tuple(reasons))


def build_zero_host_command_frame() -> bytes:
    from ackermann_robot.drivers.c30d_host_command_frame import build_ackermann_host_command_frame

    return build_ackermann_host_command_frame(
        reserved_1=0,
        reserved_2=0,
        target_x=0,
        target_y=0,
        target_z=0,
    )


def _coerce_optional_float(value: Any) -> float | None:
    if value is None:
        return None
    return float(value)


def _coerce_optional_int(value: Any) -> int | None:
    if value is None:
        return None
    return int(value)


def preflight_summary_from_results(
    results: list[Any],
    mode: str = PREFLIGHT_MODE_C30D_ONLY,
    duration_s: float = DEFAULT_PREFLIGHT_DURATION_S,
) -> PreflightSummary:
    c30d = next((result for result in results if getattr(result, "name", None) == "c30d"), None)
    passed = all(bool(getattr(result, "passed", False)) for result in results)
    if c30d is None:
        return PreflightSummary(
            passed=passed,
            candidate_battery_mV=None,
            mode=mode,
            duration_s=duration_s,
        )

    details = getattr(c30d, "details", {})
    return PreflightSummary(
        passed=passed,
        candidate_battery_mV=details.get("candidate_battery_mV_min"),
        frame_rate_hz=_coerce_optional_float(details.get("frame_rate_hz")),
        frame_rate_threshold_hz=_coerce_optional_float(details.get("threshold_hz")),
        invalid_checksum_count=_coerce_optional_int(details.get("invalid_checksum_count")),
        battery_warning_reasons=tuple(details.get("battery_warning_reasons", ())),
        battery_block_reasons=tuple(details.get("battery_block_reasons", ())),
        mode=mode,
        duration_s=duration_s,
    )


def preflight_summary_from_json(
    path: Path,
    mode: str = PREFLIGHT_MODE_C30D_ONLY,
    duration_s: float = DEFAULT_PREFLIGHT_DURATION_S,
) -> PreflightSummary:
    data = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(data, dict) and "preflight" in data:
        preflight = data["preflight"]
        return PreflightSummary(
            passed=bool(preflight.get("passed", False)),
            candidate_battery_mV=preflight.get("candidate_battery_mV"),
            frame_rate_hz=_coerce_optional_float(preflight.get("frame_rate_hz")),
            frame_rate_threshold_hz=_coerce_optional_float(
                preflight.get("frame_rate_threshold_hz") or preflight.get("threshold_hz")
            ),
            invalid_checksum_count=_coerce_optional_int(preflight.get("invalid_checksum_count")),
            battery_warning_reasons=tuple(preflight.get("battery_warning_reasons", ())),
            battery_block_reasons=tuple(preflight.get("battery_block_reasons", ())),
            mode=str(preflight.get("mode", mode)),
            duration_s=float(preflight.get("duration_s", duration_s)),
        )
    if isinstance(data, dict) and "results" in data:
        data = data["results"]
    if not isinstance(data, list):
        raise ValueError("preflight results JSON must contain a list or a preflight mapping")

    passed = all(bool(item.get("passed", False)) for item in data if isinstance(item, dict))
    c30d = next(
        (item for item in data if isinstance(item, dict) and item.get("name") == "c30d"),
        None,
    )
    details = c30d.get("details", {}) if isinstance(c30d, dict) else {}
    return PreflightSummary(
        passed=passed,
        candidate_battery_mV=details.get("candidate_battery_mV_min"),
        frame_rate_hz=_coerce_optional_float(details.get("frame_rate_hz")),
        frame_rate_threshold_hz=_coerce_optional_float(details.get("threshold_hz")),
        invalid_checksum_count=_coerce_optional_int(details.get("invalid_checksum_count")),
        battery_warning_reasons=tuple(details.get("battery_warning_reasons", ())),
        battery_block_reasons=tuple(details.get("battery_block_reasons", ())),
        mode=mode,
        duration_s=duration_s,
    )


def run_readonly_preflight(
    duration_s: float,
    mode: str = PREFLIGHT_MODE_C30D_ONLY,
) -> PreflightSummary:
    from check_robot_sensors import build_parser as build_preflight_parser
    from check_robot_sensors import print_summary, run_preflight_checks

    argv = ["--duration", str(duration_s)]
    if mode == PREFLIGHT_MODE_C30D_ONLY:
        argv.extend(["--no-check-rplidar", "--no-check-oak"])
    elif mode != PREFLIGHT_MODE_FULL_SENSOR:
        raise ValueError(f"unknown preflight mode: {mode}")
    args = build_preflight_parser().parse_args(argv)
    results = run_preflight_checks(args)
    print_summary(results)
    return preflight_summary_from_results(results, mode=mode, duration_s=duration_s)


def load_warning_battery_threshold() -> int:
    from check_robot_sensors import load_battery_safety_config

    return load_battery_safety_config().warn_battery_mV


def unique_reasons_preserving_order(reasons: list[str]) -> tuple[str, ...]:
    unique: list[str] = []
    seen: set[str] = set()
    for reason in reasons:
        if reason in seen:
            continue
        seen.add(reason)
        unique.append(reason)
    return tuple(unique)


def evaluate_readiness(
    confirmations: dict[str, bool],
    preflight: PreflightSummary,
    warning_battery_mV: int,
    zero_frame: bytes | None = None,
) -> ReadinessReport:
    frame = zero_frame or build_zero_host_command_frame()
    validation = validate_zero_frame(frame)
    reasons: list[str] = []

    for name in REQUIRED_CONFIRMATIONS:
        if not confirmations.get(name, False):
            reasons.append(f"missing_confirmation:{name}")
    if not preflight.passed:
        reasons.append("preflight_not_passed")
    if preflight.candidate_battery_mV is None:
        reasons.append("candidate_battery_mV_missing")
    elif preflight.candidate_battery_mV < warning_battery_mV:
        reasons.append("candidate_battery_below_warning_threshold")
    if preflight.frame_rate_hz is None:
        reasons.append("c30d_frame_rate_hz_missing")
    if preflight.frame_rate_threshold_hz is None:
        reasons.append("c30d_frame_rate_threshold_hz_missing")
    elif (
        preflight.frame_rate_hz is not None
        and preflight.frame_rate_hz < preflight.frame_rate_threshold_hz
    ):
        reasons.append("c30d_frame_rate_below_threshold")
    if preflight.invalid_checksum_count is None:
        reasons.append("invalid_checksum_count_missing")
    elif preflight.invalid_checksum_count != 0:
        reasons.append("invalid_c30d_checksum_frames_observed")
    if preflight.battery_warning_reasons:
        reasons.extend(preflight.battery_warning_reasons)
    if preflight.battery_block_reasons:
        reasons.extend(preflight.battery_block_reasons)
    if not validation.valid:
        reasons.extend(validation.reasons)

    unique_reasons = unique_reasons_preserving_order(reasons)
    return ReadinessReport(
        readiness_allowed=not unique_reasons,
        reasons=unique_reasons,
        preflight=preflight,
        zero_frame=frame,
        zero_frame_validation=validation,
        warning_battery_mV=warning_battery_mV,
        preflight_mode=preflight.mode,
        preflight_duration_s=preflight.duration_s,
    )


def format_bool(value: bool) -> str:
    return str(value).lower()


def print_report(report: ReadinessReport) -> None:
    print(f"readiness_allowed: {format_bool(report.readiness_allowed)}")
    print(f"readiness_reasons: {', '.join(report.reasons) if report.reasons else 'ok'}")
    print(f"preflight_mode: {report.preflight_mode}")
    print(f"preflight_duration_s: {report.preflight_duration_s:g}")
    print(f"preflight_status: {'PASS' if report.preflight.passed else 'FAIL'}")
    print(f"c30d_frame_rate_hz: {report.preflight.frame_rate_hz}")
    print(f"c30d_frame_rate_threshold_hz: {report.preflight.frame_rate_threshold_hz}")
    print(f"invalid_checksum_count: {report.preflight.invalid_checksum_count}")
    if report.preflight.invalid_checksum_count:
        print("invalid_checksum_guidance: rerun after checking USB/serial stability")
    print(f"battery_candidate_mV: {report.preflight.candidate_battery_mV}")
    print(f"battery_warning_threshold_mV: {report.warning_battery_mV}")
    print(f"zero_frame_hex: {report.zero_frame.hex(' ')}")
    print(f"zero_frame_valid: {format_bool(report.zero_frame_validation.valid)}")
    print(
        "zero_frame_validation_reasons: "
        f"{', '.join(report.zero_frame_validation.reasons) if report.zero_frame_validation.reasons else 'ok'}"
    )
    print("real_write_enabled: false")
    print("no bytes sent")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        preflight = (
            preflight_summary_from_json(
                args.preflight_results,
                mode=args.preflight_mode,
                duration_s=args.preflight_duration,
            )
            if args.preflight_results is not None
            else run_readonly_preflight(args.preflight_duration, args.preflight_mode)
        )
        threshold = load_warning_battery_threshold()
    except (OSError, ValueError) as exc:
        print(f"failed to prepare readiness inputs: {exc}", file=sys.stderr)
        return 2

    confirmations = {name: getattr(args, name) for name in REQUIRED_CONFIRMATIONS}
    report = evaluate_readiness(confirmations, preflight, threshold)
    print_report(report)
    return 0 if report.readiness_allowed else 1


if __name__ == "__main__":
    raise SystemExit(main())
