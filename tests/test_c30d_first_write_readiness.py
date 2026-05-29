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


def stable_preflight(module, **overrides):
    values = {
        "passed": True,
        "candidate_battery_mV": 11000,
        "frame_rate_hz": 25.0,
        "frame_rate_threshold_hz": 10.0,
        "invalid_checksum_count": 0,
        "mode": module.PREFLIGHT_MODE_C30D_ONLY,
        "duration_s": 5.0,
    }
    values.update(overrides)
    return module.PreflightSummary(**values)


def test_readiness_parser_defaults_to_five_second_c30d_only_preflight():
    module = load_readiness_script()

    args = module.build_parser().parse_args([])

    assert args.preflight_duration == 5.0
    assert args.preflight_mode == module.PREFLIGHT_MODE_C30D_ONLY


def test_c30d_only_readiness_passes_when_c30d_is_stable():
    module = load_readiness_script()

    report = module.evaluate_readiness(
        all_confirmations(),
        stable_preflight(module),
        warning_battery_mV=10800,
    )

    assert report.readiness_allowed is True
    assert report.reasons == ()
    assert report.preflight_mode == module.PREFLIGHT_MODE_C30D_ONLY
    assert report.preflight_duration_s == 5.0
    assert report.zero_frame.hex(" ") == "7b 00 00 00 00 00 00 00 00 7b 7d"
    assert report.zero_frame_validation.valid is True


def test_c30d_only_readiness_fails_with_invalid_checksum(capsys):
    module = load_readiness_script()

    report = module.evaluate_readiness(
        all_confirmations(),
        stable_preflight(module, invalid_checksum_count=1),
        warning_battery_mV=10800,
    )
    module.print_report(report)

    output = capsys.readouterr().out
    assert report.readiness_allowed is False
    assert "invalid_c30d_checksum_frames_observed" in report.reasons
    assert "invalid_checksum_count: 1" in output
    assert "rerun after checking USB/serial stability" in output


def test_readiness_fails_when_c30d_frame_rate_below_threshold():
    module = load_readiness_script()

    report = module.evaluate_readiness(
        all_confirmations(),
        stable_preflight(module, frame_rate_hz=9.9, frame_rate_threshold_hz=10.0),
        warning_battery_mV=10800,
    )

    assert report.readiness_allowed is False
    assert "c30d_frame_rate_below_threshold" in report.reasons


def test_readiness_fails_when_battery_below_warning_threshold():
    module = load_readiness_script()
    preflight = stable_preflight(module, candidate_battery_mV=10799)

    report = module.evaluate_readiness(all_confirmations(), preflight, warning_battery_mV=10800)

    assert report.readiness_allowed is False
    assert "candidate_battery_below_warning_threshold" in report.reasons


def test_readiness_reasons_are_unique_preserving_order_for_duplicate_battery_warning():
    module = load_readiness_script()
    preflight = stable_preflight(
        module,
        candidate_battery_mV=10799,
        battery_warning_reasons=("candidate_battery_below_warning_threshold",),
    )

    report = module.evaluate_readiness(all_confirmations(), preflight, warning_battery_mV=10800)

    assert report.reasons == ("candidate_battery_below_warning_threshold",)


def test_readiness_fails_when_confirmation_missing():
    module = load_readiness_script()
    confirmations = all_confirmations()
    confirmations["robot_restrained"] = False

    report = module.evaluate_readiness(
        confirmations,
        stable_preflight(module),
        warning_battery_mV=10800,
    )

    assert report.readiness_allowed is False
    assert "missing_confirmation:robot_restrained" in report.reasons


def test_readiness_fails_when_preflight_fails():
    module = load_readiness_script()

    report = module.evaluate_readiness(
        all_confirmations(),
        stable_preflight(module, passed=False),
        warning_battery_mV=10800,
    )

    assert report.readiness_allowed is False
    assert "preflight_not_passed" in report.reasons


def test_full_sensor_mode_still_uses_all_sensor_checks(monkeypatch):
    module = load_readiness_script()
    import check_robot_sensors

    captured = {}

    def fake_run_preflight_checks(args):
        captured["duration"] = args.duration
        captured["check_c30d"] = args.check_c30d
        captured["check_rplidar"] = args.check_rplidar
        captured["check_oak"] = args.check_oak
        return [
            check_robot_sensors.CheckResult("data", True, "ok", {}),
            check_robot_sensors.CheckResult("data/runs", True, "ok", {}),
            check_robot_sensors.CheckResult("device:/dev/c30d", True, "ok", {}),
            check_robot_sensors.CheckResult(
                "c30d",
                True,
                "ok",
                {
                    "candidate_battery_mV_min": 11000,
                    "frame_rate_hz": 25.0,
                    "threshold_hz": 10.0,
                    "invalid_checksum_count": 0,
                },
            ),
            check_robot_sensors.CheckResult("device:/dev/rplidar", True, "ok", {}),
            check_robot_sensors.CheckResult("rplidar", True, "ok", {}),
            check_robot_sensors.CheckResult("oak", True, "ok", {}),
        ]

    monkeypatch.setattr(check_robot_sensors, "run_preflight_checks", fake_run_preflight_checks)
    monkeypatch.setattr(check_robot_sensors, "print_summary", lambda _results: None)

    preflight = module.run_readonly_preflight(6.0, module.PREFLIGHT_MODE_FULL_SENSOR)

    assert preflight.passed is True
    assert preflight.mode == module.PREFLIGHT_MODE_FULL_SENSOR
    assert preflight.duration_s == 6.0
    assert captured == {
        "duration": 6.0,
        "check_c30d": True,
        "check_rplidar": True,
        "check_oak": True,
    }


def test_c30d_only_mode_disables_rplidar_and_oak(monkeypatch):
    module = load_readiness_script()
    import check_robot_sensors

    captured = {}

    def fake_run_preflight_checks(args):
        captured["check_rplidar"] = args.check_rplidar
        captured["check_oak"] = args.check_oak
        return [
            check_robot_sensors.CheckResult("data", True, "ok", {}),
            check_robot_sensors.CheckResult("data/runs", True, "ok", {}),
            check_robot_sensors.CheckResult("device:/dev/c30d", True, "ok", {}),
            check_robot_sensors.CheckResult(
                "c30d",
                True,
                "ok",
                {
                    "candidate_battery_mV_min": 11000,
                    "frame_rate_hz": 25.0,
                    "threshold_hz": 10.0,
                    "invalid_checksum_count": 0,
                },
            ),
        ]

    monkeypatch.setattr(check_robot_sensors, "run_preflight_checks", fake_run_preflight_checks)
    monkeypatch.setattr(check_robot_sensors, "print_summary", lambda _results: None)

    preflight = module.run_readonly_preflight(5.0)

    assert preflight.passed is True
    assert preflight.mode == module.PREFLIGHT_MODE_C30D_ONLY
    assert captured == {"check_rplidar": False, "check_oak": False}


def test_readiness_cli_consumes_json_preflight_results(tmp_path: Path, capsys):
    module = load_readiness_script()
    preflight_path = tmp_path / "preflight.json"
    preflight_path.write_text(
        json.dumps(
            {
                "preflight": {
                    "passed": True,
                    "candidate_battery_mV": 11000,
                    "frame_rate_hz": 25.0,
                    "frame_rate_threshold_hz": 10.0,
                    "invalid_checksum_count": 0,
                    "mode": "c30d_only",
                    "duration_s": 5.0,
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
    assert "preflight_mode: c30d_only" in output
    assert "preflight_duration_s: 5" in output
    assert "preflight_status: PASS" in output
    assert "invalid_checksum_count: 0" in output
    assert "battery_candidate_mV: 11000" in output
    assert "zero_frame_hex: 7b 00 00 00 00 00 00 00 00 7b 7d" in output
    assert "real_write_enabled: false" in output
    assert "no bytes sent" in output
