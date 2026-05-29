from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class CommandSafetyConfig:
    dry_run_default: bool = True
    require_manual_enable: bool = True
    require_wheels_lifted_for_motor_test: bool = True
    max_test_speed_mps: float = 0.2
    max_test_duration_s: float = 2.0
    allow_serial_write: bool = False


@dataclass(frozen=True)
class ArmingState:
    preflight_passed: bool = False
    manual_enable: bool = False
    wheels_lifted_confirmed: bool = False
    dry_run: bool = True
    serial_write_allowed: bool = False


@dataclass(frozen=True)
class SafetyGateResult:
    allowed: bool
    dry_run: bool
    serial_write_allowed: bool
    reasons: tuple[str, ...] = field(default_factory=tuple)
    warnings: tuple[str, ...] = field(default_factory=tuple)


def load_command_safety_config(
    config_path: str | Path = "config/command_safety.yaml",
) -> CommandSafetyConfig:
    path = Path(config_path)
    loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        raise ValueError(f"command safety config must contain a mapping: {path}")
    data = loaded.get("command_safety")
    if not isinstance(data, dict):
        raise ValueError(f"missing command_safety section: {path}")
    return command_safety_config_from_dict(data)


def command_safety_config_from_dict(data: dict[str, Any]) -> CommandSafetyConfig:
    return CommandSafetyConfig(
        dry_run_default=bool(data.get("dry_run_default", True)),
        require_manual_enable=bool(data.get("require_manual_enable", True)),
        require_wheels_lifted_for_motor_test=bool(
            data.get("require_wheels_lifted_for_motor_test", True)
        ),
        max_test_speed_mps=float(data.get("max_test_speed_mps", 0.2)),
        max_test_duration_s=float(data.get("max_test_duration_s", 2.0)),
        allow_serial_write=bool(data.get("allow_serial_write", False)),
    )


def command_within_limits(
    *,
    speed_mps: float,
    duration_s: float,
    config: CommandSafetyConfig,
) -> bool:
    return abs(speed_mps) <= config.max_test_speed_mps and duration_s <= config.max_test_duration_s


def evaluate_safety_gate(
    *,
    config: CommandSafetyConfig,
    state: ArmingState,
    speed_mps: float,
    duration_s: float,
    command_was_clamped: bool = False,
    require_preflight: bool = False,
) -> SafetyGateResult:
    reasons: list[str] = []
    warnings: list[str] = []

    if config.require_manual_enable and not state.manual_enable:
        reasons.append("manual_enable_required")

    if config.require_wheels_lifted_for_motor_test and not state.wheels_lifted_confirmed:
        reasons.append("wheels_lifted_confirmation_required")

    if not command_within_limits(speed_mps=speed_mps, duration_s=duration_s, config=config):
        reasons.append("command_outside_test_limits")

    if command_was_clamped:
        reasons.append("command_filter_clamped_command")

    if require_preflight and not state.preflight_passed:
        reasons.append("preflight_required")
    elif not state.dry_run and not state.preflight_passed:
        reasons.append("preflight_required_for_real_command_path")
    elif state.dry_run and not state.preflight_passed:
        warnings.append("preflight_not_passed_dry_run_only")

    serial_write_allowed = (
        bool(config.allow_serial_write) and bool(state.serial_write_allowed) and not state.dry_run
    )
    if not state.dry_run and not serial_write_allowed:
        reasons.append("serial_write_not_allowed")

    if config.allow_serial_write:
        warnings.append("config_allows_serial_write_future_path")

    return SafetyGateResult(
        allowed=not reasons,
        dry_run=state.dry_run,
        serial_write_allowed=serial_write_allowed,
        reasons=tuple(reasons),
        warnings=tuple(warnings),
    )
