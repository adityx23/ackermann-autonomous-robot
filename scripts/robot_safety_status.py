#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
SCRIPT_ROOT = REPO_ROOT / "scripts"
for path in (SRC_ROOT, SCRIPT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Report Pi-side command safety status.")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("config/command_safety.yaml"),
        help="Command safety config path.",
    )
    parser.add_argument(
        "--run-preflight",
        action="store_true",
        help="Run the read-only sensor preflight before reporting command-test status.",
    )
    parser.add_argument(
        "--preflight-duration",
        type=float,
        default=3.0,
        help="Duration to pass to check_robot_sensors.py when --run-preflight is set.",
    )
    return parser


def print_config(config) -> None:
    print("Command Safety Config")
    print(f"dry_run_default: {config.dry_run_default}")
    print(f"require_manual_enable: {config.require_manual_enable}")
    print(f"require_wheels_lifted_for_motor_test: {config.require_wheels_lifted_for_motor_test}")
    print(f"max_test_speed_mps: {config.max_test_speed_mps}")
    print(f"max_test_duration_s: {config.max_test_duration_s}")
    print(f"allow_serial_write: {config.allow_serial_write}")


def run_preflight(duration_s: float) -> bool:
    from check_robot_sensors import build_parser as build_preflight_parser
    from check_robot_sensors import run_preflight as run_sensor_preflight

    args = build_preflight_parser().parse_args(["--duration", str(duration_s)])
    return run_sensor_preflight(args) == 0


def main(argv: list[str] | None = None) -> int:
    from ackermann_robot.control.arming import (
        ArmingState,
        evaluate_safety_gate,
        load_command_safety_config,
    )

    args = build_parser().parse_args(argv)
    config = load_command_safety_config(args.config)
    print_config(config)

    preflight_passed = False
    if args.run_preflight:
        preflight_passed = run_preflight(args.preflight_duration)
    else:
        print("preflight: not_run")

    state = ArmingState(
        preflight_passed=preflight_passed,
        manual_enable=False,
        wheels_lifted_confirmed=False,
        dry_run=config.dry_run_default,
        serial_write_allowed=False,
    )
    gate = evaluate_safety_gate(
        config=config,
        state=state,
        speed_mps=0.0,
        duration_s=0.0,
    )

    print("Command Test Status")
    print(f"dry_run: {gate.dry_run}")
    print(f"serial_write_allowed: {gate.serial_write_allowed}")
    print(f"allowed_to_run_command_tests_now: {gate.allowed}")
    print(f"reasons: {', '.join(gate.reasons) if gate.reasons else 'ok'}")
    print(f"warnings: {', '.join(gate.warnings) if gate.warnings else 'none'}")
    print("real_motor_command_path: disabled")
    print("c30d_command_protocol: not_implemented")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
