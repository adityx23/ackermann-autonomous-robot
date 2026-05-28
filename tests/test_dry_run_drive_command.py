from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path

import yaml


def load_dry_run_script():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "dry_run_drive_command.py"
    spec = importlib.util.spec_from_file_location(script_path.stem, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[script_path.stem] = module
    spec.loader.exec_module(module)
    return module


def write_command_safety_config(path: Path) -> None:
    path.write_text(
        yaml.safe_dump(
            {
                "command_safety": {
                    "dry_run_default": True,
                    "require_manual_enable": True,
                    "require_wheels_lifted_for_motor_test": True,
                    "max_test_speed_mps": 0.2,
                    "max_test_duration_s": 2.0,
                    "allow_serial_write": False,
                }
            }
        ),
        encoding="utf-8",
    )


def test_dry_run_drive_command_allows_safe_dry_run(tmp_path: Path, capsys):
    module = load_dry_run_script()
    config_path = tmp_path / "command_safety.yaml"
    write_command_safety_config(config_path)

    exit_code = module.main(
        [
            "--speed-mps",
            "0.1",
            "--steering-deg",
            "5",
            "--duration",
            "1.0",
            "--manual-enable",
            "--wheels-lifted",
            "--config",
            str(config_path),
        ]
    )

    output = capsys.readouterr().out
    assert exit_code == 0
    assert "safety_allowed: True" in output
    assert "serial_write_allowed: False" in output
    assert "would_send: disabled_real_c30d_protocol_not_implemented" in output


def test_dry_run_drive_command_rejects_missing_manual_enable(tmp_path: Path, capsys):
    module = load_dry_run_script()
    config_path = tmp_path / "command_safety.yaml"
    write_command_safety_config(config_path)

    exit_code = module.main(
        [
            "--speed-mps",
            "0.1",
            "--duration",
            "1.0",
            "--wheels-lifted",
            "--config",
            str(config_path),
        ]
    )

    output = capsys.readouterr().out
    assert exit_code == 1
    assert "manual_enable_required" in output


def test_dry_run_drive_command_rejects_excessive_speed_and_reports_clamp(
    tmp_path: Path, capsys
):
    module = load_dry_run_script()
    config_path = tmp_path / "command_safety.yaml"
    write_command_safety_config(config_path)

    exit_code = module.main(
        [
            "--speed-mps",
            "0.8",
            "--duration",
            "1.0",
            "--manual-enable",
            "--wheels-lifted",
            "--config",
            str(config_path),
        ]
    )

    output = capsys.readouterr().out
    assert exit_code == 1
    assert "filtered_speed_mps: 0.2" in output
    assert "command_outside_test_limits" in output
    assert "command_filter_clamped_command" in output


def test_dry_run_drive_command_does_not_open_serial(monkeypatch, tmp_path: Path):
    module = load_dry_run_script()
    config_path = tmp_path / "command_safety.yaml"
    write_command_safety_config(config_path)

    def fail_open(*args, **kwargs):
        raise AssertionError("dry-run drive command must not open serial")

    monkeypatch.setattr(os, "open", fail_open)

    exit_code = module.main(
        [
            "--speed-mps",
            "0.1",
            "--duration",
            "1.0",
            "--manual-enable",
            "--wheels-lifted",
            "--config",
            str(config_path),
        ]
    )

    assert exit_code == 0
