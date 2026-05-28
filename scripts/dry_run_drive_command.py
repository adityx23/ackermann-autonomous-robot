#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
for path in (SRC_ROOT,):
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
        "--config",
        type=Path,
        default=Path("config/command_safety.yaml"),
        help="Command safety config path.",
    )
    return parser


def run_dry_command(args: argparse.Namespace) -> int:
    from ackermann_robot.control.arming import (
        ArmingState,
        evaluate_safety_gate,
        load_command_safety_config,
    )
    from ackermann_robot.control.command_filter import CommandFilter, CommandLimits
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
    state = ArmingState(
        preflight_passed=False,
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
    )

    print("DRY-RUN drive command only. No serial port is opened. No bytes are written.")
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
    print("would_send: disabled_real_c30d_protocol_not_implemented")
    return 0 if gate.allowed else 1


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return run_dry_command(args)


if __name__ == "__main__":
    raise SystemExit(main())
