#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
SCRIPT_ROOT = REPO_ROOT / "scripts"
for path in (SRC_ROOT, SCRIPT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Evaluate a future drive command through safety gates without touching hardware."
    )
    parser.add_argument("--speed-mps", type=float, required=True)
    parser.add_argument("--steering-deg", type=float, default=0.0)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--manual-enable", action="store_true")
    parser.add_argument("--wheels-lifted", action="store_true")
    parser.add_argument(
        "--require-preflight",
        action="store_true",
        help="Reject the dry-run command unless read-only preflight has passed.",
    )
    parser.add_argument(
        "--run-preflight",
        action="store_true",
        help="Run read-only sensor preflight before evaluating the dry-run command.",
    )
    parser.add_argument(
        "--preflight-duration",
        type=float,
        default=3.0,
        help="Duration to pass to check_robot_sensors.py when --run-preflight is set.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("config/command_safety.yaml"),
        help="Command safety config path.",
    )
    return parser


@dataclass(frozen=True)
class PreflightStatus:
    passed: bool
    candidate_battery_mV: float | int | None = None
    warnings: tuple[str, ...] = ()
    reasons: tuple[str, ...] = ()


def preflight_status_from_results(results: list[Any]) -> PreflightStatus:
    c30d = next((result for result in results if getattr(result, "name", None) == "c30d"), None)
    warnings: list[str] = []
    reasons: list[str] = []
    candidate_battery_mV = None
    if c30d is not None:
        details = getattr(c30d, "details", {})
        candidate_battery_mV = details.get("candidate_battery_mV_mean")
        warnings.extend(details.get("battery_warning_reasons", ()))
        reasons.extend(details.get("battery_block_reasons", ()))
    return PreflightStatus(
        passed=all(result.passed for result in results),
        candidate_battery_mV=candidate_battery_mV,
        warnings=tuple(warnings),
        reasons=tuple(reasons),
    )


def coerce_preflight_status(value: bool | PreflightStatus) -> PreflightStatus:
    if isinstance(value, PreflightStatus):
        return value
    return PreflightStatus(passed=bool(value))


def run_preflight(duration_s: float) -> PreflightStatus:
    from check_robot_sensors import build_parser as build_preflight_parser
    from check_robot_sensors import print_summary, run_preflight_checks

    args = build_preflight_parser().parse_args(["--duration", str(duration_s)])
    results = run_preflight_checks(args)
    print_summary(results)
    return preflight_status_from_results(results)


def run_dry_command(args: argparse.Namespace) -> int:
    from ackermann_robot.control.arming import (
        ArmingState,
        evaluate_safety_gate,
        load_command_safety_config,
    )
    from ackermann_robot.control.command_filter import CommandFilter, CommandLimits
    from ackermann_robot.drivers.c30d_commands import (
        C30DCommandCandidate,
        build_dry_run_command_packet,
    )
    from ackermann_robot.messages import DriveCommand, SafetyStatus

    config = load_command_safety_config(args.config)
    command = DriveCommand(
        speed_mps=args.speed_mps,
        steering_deg=args.steering_deg,
        timestamp_s=time.monotonic(),
        source="dry_run_drive_command",
    )
    command_filter = CommandFilter(
        CommandLimits(
            max_speed_mps=config.max_test_speed_mps,
            max_reverse_speed_mps=config.max_test_speed_mps,
            max_steering_deg=25.0,
            max_accel_mps2=config.max_test_speed_mps,
        )
    )
    filtered = command_filter.filter(
        command,
        safety_status=SafetyStatus(enabled=True, stopped=False),
    )
    command_was_clamped = (
        filtered.speed_mps != command.speed_mps or filtered.steering_deg != command.steering_deg
    )
    preflight = (
        coerce_preflight_status(run_preflight(args.preflight_duration))
        if args.run_preflight
        else PreflightStatus(passed=False)
    )
    preflight_passed = preflight.passed
    state = ArmingState(
        preflight_passed=preflight_passed,
        manual_enable=args.manual_enable,
        wheels_lifted_confirmed=args.wheels_lifted,
        dry_run=True,
        serial_write_allowed=False,
    )
    gate = evaluate_safety_gate(
        config=config,
        state=state,
        speed_mps=command.speed_mps,
        duration_s=args.duration,
        command_was_clamped=command_was_clamped,
        require_preflight=args.require_preflight,
    )
    c30d_command = C30DCommandCandidate(
        speed_mps=filtered.speed_mps,
        steering_deg=filtered.steering_deg,
        duration_s=args.duration,
        source=command.source,
    )
    packet = build_dry_run_command_packet(c30d_command)

    print("DRY-RUN drive command only. No serial port is opened. No bytes are written.")
    print(f"require_preflight: {args.require_preflight}")
    print(f"preflight_passed: {preflight_passed}")
    print(f"candidate_battery_mV: {preflight.candidate_battery_mV}")
    print(
        "preflight_battery_warnings: "
        f"{', '.join(preflight.warnings) if preflight.warnings else 'none'}"
    )
    print(
        "preflight_battery_block_reasons: "
        f"{', '.join(preflight.reasons) if preflight.reasons else 'none'}"
    )
    print(f"requested_speed_mps: {command.speed_mps}")
    print(f"requested_steering_deg: {command.steering_deg}")
    print(f"requested_duration_s: {args.duration}")
    print(f"filtered_speed_mps: {filtered.speed_mps}")
    print(f"filtered_steering_deg: {filtered.steering_deg}")
    print(f"filter_reasons: {', '.join(filtered.reasons)}")
    print(f"safety_allowed: {gate.allowed}")
    print(f"safety_reasons: {', '.join(gate.reasons) if gate.reasons else 'ok'}")
    print(f"safety_warnings: {', '.join(gate.warnings) if gate.warnings else 'none'}")
    print(f"serial_write_allowed: {gate.serial_write_allowed}")
    print(f"c30d_protocol_known: {packet.protocol_known}")
    print(f"c30d_packet_hex: {packet.packet_hex}")
    print(f"c30d_packet_notes: {packet.notes}")
    print("would_send: disabled_real_c30d_protocol_not_implemented")
    return 0 if gate.allowed else 1


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return run_dry_command(args)


if __name__ == "__main__":
    raise SystemExit(main())
