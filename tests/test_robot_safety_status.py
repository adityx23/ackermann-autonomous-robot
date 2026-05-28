from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

from ackermann_robot.control.arming import CommandSafetyConfig


def load_status_script():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "robot_safety_status.py"
    spec = importlib.util.spec_from_file_location(script_path.stem, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[script_path.stem] = module
    spec.loader.exec_module(module)
    return module


def test_status_for_preflight_can_represent_passed_and_failed():
    module = load_status_script()
    config = CommandSafetyConfig()

    failed = module.status_for_preflight(config, preflight_passed=False)
    passed = module.status_for_preflight(config, preflight_passed=True)

    assert "preflight_not_passed_dry_run_only" in failed.warnings
    assert "preflight_not_passed_dry_run_only" not in passed.warnings
    assert failed.serial_write_allowed is False
    assert passed.serial_write_allowed is False


def test_robot_safety_status_prints_preflight_result(monkeypatch, tmp_path: Path, capsys):
    module = load_status_script()
    config_path = tmp_path / "command_safety.yaml"
    config_path.write_text(
        "\n".join(
            [
                "command_safety:",
                "  dry_run_default: true",
                "  require_manual_enable: true",
                "  require_wheels_lifted_for_motor_test: true",
                "  max_test_speed_mps: 0.2",
                "  max_test_duration_s: 2.0",
                "  allow_serial_write: false",
            ]
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(
        module,
        "run_preflight",
        lambda duration_s: module.PreflightStatus(
            passed=True,
            candidate_battery_mV=10750,
            warnings=("candidate_battery_below_warning_threshold",),
        ),
    )

    exit_code = module.main(["--config", str(config_path), "--run-preflight"])

    output = capsys.readouterr().out
    assert exit_code == 0
    assert "preflight_passed: True" in output
    assert "candidate_battery_mV: 10750" in output
    assert "candidate_battery_below_warning_threshold" in output
    assert "real_motor_command_path: disabled" in output
