from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path


def load_readiness_script():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "c30d_first_write_readiness.py"
    spec = importlib.util.spec_from_file_location(script_path.stem, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[script_path.stem] = module
    spec.loader.exec_module(module)
    return module


def all_confirmations() -> dict[str, bool]:
    module = load_readiness_script()
    return {name: True for name in module.REQUIRED_CONFIRMATIONS}


def test_readiness_allows_when_preflight_battery_and_confirmations_pass():
    module = load_readiness_script()
    preflight = module.PreflightSummary(passed=True, candidate_battery_mV=11000)

    report = module.evaluate_readiness(all_confirmations(), preflight, warning_battery_mV=10800)

    assert report.readiness_allowed is True
    assert report.reasons == ()
    assert report.zero_frame.hex(" ") == "7b 00 00 00 00 00 00 00 00 7b 7d"
    assert report.zero_frame_validation.valid is True


def test_readiness_fails_when_battery_below_warning_threshold():
    module = load_readiness_script()
    preflight = module.PreflightSummary(passed=True, candidate_battery_mV=10799)

    report = module.evaluate_readiness(all_confirmations(), preflight, warning_battery_mV=10800)

    assert report.readiness_allowed is False
    assert "candidate_battery_below_warning_threshold" in report.reasons


def test_readiness_fails_when_confirmation_missing():
    module = load_readiness_script()
    confirmations = all_confirmations()
    confirmations["robot_restrained"] = False
    preflight = module.PreflightSummary(passed=True, candidate_battery_mV=11000)

    report = module.evaluate_readiness(confirmations, preflight, warning_battery_mV=10800)

    assert report.readiness_allowed is False
    assert "missing_confirmation:robot_restrained" in report.reasons


def test_readiness_fails_when_preflight_fails():
    module = load_readiness_script()
    preflight = module.PreflightSummary(passed=False, candidate_battery_mV=11000)

    report = module.evaluate_readiness(all_confirmations(), preflight, warning_battery_mV=10800)

    assert report.readiness_allowed is False
    assert "preflight_not_passed" in report.reasons


def test_readiness_cli_consumes_json_preflight_results(tmp_path: Path, capsys):
    module = load_readiness_script()
    preflight_path = tmp_path / "preflight.json"
    preflight_path.write_text(
        json.dumps(
            {
                "preflight": {
                    "passed": True,
                    "candidate_battery_mV": 11000,
                }
            }
        ),
        encoding="utf-8",
    )

    exit_code = module.main(
        [
            "--wheels-lifted",
            "--robot-restrained",
            "--manual-power-cutoff-ready",
            "--motor-enable-switch-reviewed",
            "--i-understand-this-is-not-a-motor-test",
            "--preflight-results",
            str(preflight_path),
        ]
    )

    output = capsys.readouterr().out
    assert exit_code == 0
    assert "readiness_allowed: true" in output
    assert "preflight_status: PASS" in output
    assert "battery_candidate_mV: 11000" in output
    assert "zero_frame_hex: 7b 00 00 00 00 00 00 00 00 7b 7d" in output
    assert "real_write_enabled: false" in output
    assert "no bytes sent" in output
