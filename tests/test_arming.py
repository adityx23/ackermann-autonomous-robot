from __future__ import annotations

from ackermann_robot.control.arming import (
    ArmingState,
    CommandSafetyConfig,
    command_safety_config_from_dict,
    evaluate_safety_gate,
)


def test_default_dry_run_is_safe_and_serial_write_disabled():
    config = CommandSafetyConfig()
    state = ArmingState(
        manual_enable=True,
        wheels_lifted_confirmed=True,
        dry_run=config.dry_run_default,
    )

    result = evaluate_safety_gate(config=config, state=state, speed_mps=0.0, duration_s=1.0)

    assert result.allowed
    assert result.dry_run is True
    assert result.serial_write_allowed is False
    assert "preflight_not_passed_dry_run_only" in result.warnings


def test_command_rejected_without_manual_enable():
    result = evaluate_safety_gate(
        config=CommandSafetyConfig(),
        state=ArmingState(wheels_lifted_confirmed=True),
        speed_mps=0.1,
        duration_s=1.0,
    )

    assert not result.allowed
    assert "manual_enable_required" in result.reasons


def test_command_rejected_without_wheels_lifted_confirmation():
    result = evaluate_safety_gate(
        config=CommandSafetyConfig(),
        state=ArmingState(manual_enable=True),
        speed_mps=0.1,
        duration_s=1.0,
    )

    assert not result.allowed
    assert "wheels_lifted_confirmation_required" in result.reasons


def test_excessive_speed_rejected():
    result = evaluate_safety_gate(
        config=CommandSafetyConfig(max_test_speed_mps=0.2),
        state=ArmingState(manual_enable=True, wheels_lifted_confirmed=True),
        speed_mps=0.3,
        duration_s=1.0,
    )

    assert not result.allowed
    assert "command_outside_test_limits" in result.reasons


def test_serial_write_allowed_false_by_default_even_for_non_dry_run():
    result = evaluate_safety_gate(
        config=CommandSafetyConfig(),
        state=ArmingState(
            preflight_passed=True,
            manual_enable=True,
            wheels_lifted_confirmed=True,
            dry_run=False,
            serial_write_allowed=False,
        ),
        speed_mps=0.1,
        duration_s=1.0,
    )

    assert not result.allowed
    assert result.serial_write_allowed is False
    assert "serial_write_not_allowed" in result.reasons


def test_preflight_required_rejects_when_not_passed():
    result = evaluate_safety_gate(
        config=CommandSafetyConfig(),
        state=ArmingState(
            preflight_passed=False,
            manual_enable=True,
            wheels_lifted_confirmed=True,
        ),
        speed_mps=0.1,
        duration_s=1.0,
        require_preflight=True,
    )

    assert not result.allowed
    assert "preflight_required" in result.reasons


def test_preflight_required_allows_when_passed_with_manual_enable_and_wheels_lifted():
    result = evaluate_safety_gate(
        config=CommandSafetyConfig(),
        state=ArmingState(
            preflight_passed=True,
            manual_enable=True,
            wheels_lifted_confirmed=True,
        ),
        speed_mps=0.1,
        duration_s=1.0,
        require_preflight=True,
    )

    assert result.allowed
    assert result.reasons == ()


def test_command_safety_config_from_dict_defaults_serial_write_false():
    config = command_safety_config_from_dict(
        {
            "dry_run_default": True,
            "require_manual_enable": True,
            "require_wheels_lifted_for_motor_test": True,
            "max_test_speed_mps": 0.1,
            "max_test_duration_s": 1.0,
        }
    )

    assert config.allow_serial_write is False
