#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
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


def status_for_preflight(config, preflight_passed: bool):
    from ackermann_robot.control.arming import ArmingState, evaluate_safety_gate

    state = ArmingState(
        preflight_passed=preflight_passed,
        manual_enable=False,
        wheels_lifted_confirmed=False,
        dry_run=config.dry_run_default,
        serial_write_allowed=False,
    )
    return evaluate_safety_gate(
        config=config,
        state=state,
        speed_mps=0.0,
        duration_s=0.0,
    )


def main(argv: list[str] | None = None) -> int:
    from ackermann_robot.control.arming import load_command_safety_config

    args = build_parser().parse_args(argv)
    config = load_command_safety_config(args.config)
    print_config(config)

    preflight = PreflightStatus(passed=False)
    if args.run_preflight:
        preflight = coerce_preflight_status(run_preflight(args.preflight_duration))
    else:
        print("preflight: not_run")
    preflight_passed = preflight.passed
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

    gate = status_for_preflight(config, preflight_passed)

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
