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


@dataclass(frozen=True)
class PreflightSummary:
    passed: bool
    candidate_battery_mV: float | int | None
    battery_warning_reasons: tuple[str, ...] = ()
    battery_block_reasons: tuple[str, ...] = ()


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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only readiness checker for a future native C30D first-write experiment. "
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
    parser.add_argument("--preflight-duration", type=float, default=3.0)
    parser.add_argument(
        "--c30d-only-preflight",
        action="store_true",
        help="When running preflight, check C30D only and skip RPLIDAR/OAK.",
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


def preflight_summary_from_results(results: list[Any]) -> PreflightSummary:
    c30d = next((result for result in results if getattr(result, "name", None) == "c30d"), None)
    passed = all(bool(getattr(result, "passed", False)) for result in results)
    if c30d is None:
        return PreflightSummary(passed=passed, candidate_battery_mV=None)

    details = getattr(c30d, "details", {})
    return PreflightSummary(
        passed=passed,
        candidate_battery_mV=details.get("candidate_battery_mV_min"),
        battery_warning_reasons=tuple(details.get("battery_warning_reasons", ())),
        battery_block_reasons=tuple(details.get("battery_block_reasons", ())),
    )


def preflight_summary_from_json(path: Path) -> PreflightSummary:
    data = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(data, dict) and "preflight" in data:
        preflight = data["preflight"]
        return PreflightSummary(
            passed=bool(preflight.get("passed", False)),
            candidate_battery_mV=preflight.get("candidate_battery_mV"),
            battery_warning_reasons=tuple(preflight.get("battery_warning_reasons", ())),
            battery_block_reasons=tuple(preflight.get("battery_block_reasons", ())),
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
        battery_warning_reasons=tuple(details.get("battery_warning_reasons", ())),
        battery_block_reasons=tuple(details.get("battery_block_reasons", ())),
    )


def run_readonly_preflight(duration_s: float, c30d_only: bool) -> PreflightSummary:
    from check_robot_sensors import build_parser as build_preflight_parser
    from check_robot_sensors import print_summary, run_preflight_checks

    argv = ["--duration", str(duration_s)]
    if c30d_only:
        argv.extend(["--no-check-rplidar", "--no-check-oak"])
    args = build_preflight_parser().parse_args(argv)
    results = run_preflight_checks(args)
    print_summary(results)
    return preflight_summary_from_results(results)


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
    )


def format_bool(value: bool) -> str:
    return str(value).lower()


def print_report(report: ReadinessReport) -> None:
    print(f"readiness_allowed: {format_bool(report.readiness_allowed)}")
    print(f"readiness_reasons: {', '.join(report.reasons) if report.reasons else 'ok'}")
    print(f"preflight_status: {'PASS' if report.preflight.passed else 'FAIL'}")
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
            preflight_summary_from_json(args.preflight_results)
            if args.preflight_results is not None
            else run_readonly_preflight(args.preflight_duration, args.c30d_only_preflight)
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
