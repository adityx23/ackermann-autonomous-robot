from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import importlib.util
from pathlib import Path
import sys


def load_probe_script():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "probe_c30d_stm32_bootloader.py"
    spec = importlib.util.spec_from_file_location("probe_c30d_stm32_bootloader", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@dataclass(frozen=True)
class FakeCompletedProcess:
    returncode: int
    stdout: str
    stderr: str = ""


def test_stm32flash_command_is_identification_only():
    module = load_probe_script()

    command = module.build_stm32flash_probe_command("/usr/bin/stm32flash", "/dev/ttyUSB0")

    assert command == ("/usr/bin/stm32flash", "/dev/ttyUSB0")
    forbidden = {"-w", "--write", "-e", "--erase", "-u", "--unlock", "-k", "--read-protect"}
    assert not forbidden.intersection(command)


def test_forbidden_stm32flash_arguments_are_rejected():
    module = load_probe_script()

    for forbidden in ("-w", "--write", "-e", "--erase", "-u", "--unlock"):
        try:
            module.validate_stm32flash_command(("stm32flash", forbidden, "/dev/ttyUSB0"))
        except ValueError as exc:
            assert "stm32flash probe command must be exactly" in str(exc) or "forbidden" in str(exc)
        else:  # pragma: no cover - explicit failure path
            raise AssertionError(f"accepted forbidden argument {forbidden}")


def test_probe_runs_only_stm32flash_port_command(monkeypatch):
    module = load_probe_script()
    commands: list[tuple[str, ...]] = []
    monkeypatch.setattr(module, "list_likely_serial_ports", lambda: ("/dev/ttyUSB0",))

    def fake_run(command, **kwargs):
        commands.append(tuple(command))
        assert kwargs["capture_output"] is True
        assert kwargs["text"] is True
        assert kwargs["check"] is False
        return FakeCompletedProcess(
            0,
            "stm32flash 0.7\nVersion      : 0x31\nDevice ID    : 0x0413 (STM32F4xx)\nReadout Protection: Disabled\n",
        )

    result = module.run_probe(
        port="/dev/ttyUSB0",
        input_fn=lambda _prompt: "YES",
        which_fn=lambda _name: "/usr/bin/stm32flash",
        run_fn=fake_run,
    )

    assert commands == [("/usr/bin/stm32flash", "/dev/ttyUSB0")]
    assert result.bootloader_responded is True
    assert result.mcu_id == "0x0413 (STM32F4xx)"
    assert result.readout_protection_status == "Disabled"


def test_missing_stm32flash_prints_guidance_and_does_not_run(monkeypatch, capsys):
    module = load_probe_script()
    monkeypatch.setattr(module, "list_likely_serial_ports", lambda: ("/dev/ttyUSB0",))

    def forbidden_run(*_args, **_kwargs):
        raise AssertionError("stm32flash must not run when missing")

    result = module.run_probe(
        port="/dev/ttyUSB0",
        input_fn=lambda _prompt: "YES",
        which_fn=lambda _name: None,
        run_fn=forbidden_run,
    )

    output = capsys.readouterr().out
    assert "stm32flash not found" in output
    assert "sudo apt install stm32flash" in output
    assert result.stm32flash_path is None
    assert result.command == ()


def test_user_refusal_runs_no_query(monkeypatch):
    module = load_probe_script()
    monkeypatch.setattr(module, "list_likely_serial_ports", lambda: ("/dev/ttyUSB0",))

    def forbidden_run(*_args, **_kwargs):
        raise AssertionError("bootloader query must not run without YES")

    result = module.run_probe(
        port="/dev/ttyUSB0",
        input_fn=lambda _prompt: "NO",
        which_fn=lambda _name: "/usr/bin/stm32flash",
        run_fn=forbidden_run,
    )

    assert result.command == ()
    assert result.bootloader_responded is False
    assert "refused" in result.next_safe_step.lower()


def test_report_contains_required_fields(tmp_path: Path):
    module = load_probe_script()
    result = module.BootloaderProbeResult(
        serial_ports_found=("/dev/ttyUSB0", "/dev/serial/by-id/c30d"),
        selected_port="/dev/ttyUSB0",
        stm32flash_path="/usr/bin/stm32flash",
        command=("/usr/bin/stm32flash", "/dev/ttyUSB0"),
        returncode=0,
        bootloader_responded=True,
        mcu_id="0x0413 (STM32F4xx)",
        readout_protection_status="Disabled",
        next_safe_step="Record MCU ID/readout status; do not flash or erase.",
    )

    report = module.write_report(result, tmp_path)
    text = report.read_text(encoding="utf-8")

    assert "Serial Ports Found" in text
    assert "Selected Port" in text
    assert "bootloader_responded: true" in text
    assert "0x0413" in text
    assert "readout protection status: Disabled" in text
    assert "Next Safe Step" in text


def test_main_writes_timestamped_report(tmp_path: Path, monkeypatch):
    module = load_probe_script()
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(module, "list_likely_serial_ports", lambda: ("/dev/ttyUSB0",))

    def fake_run(_command, **_kwargs):
        return FakeCompletedProcess(0, "stm32flash\nDevice ID    : 0x0413 (STM32F4xx)\n")

    code = module.main(
        ["--port", "/dev/ttyUSB0"],
        input_fn=lambda _prompt: "YES",
        which_fn=lambda _name: "/usr/bin/stm32flash",
        run_fn=fake_run,
        clock=lambda: datetime(2026, 5, 31, 22, 30, 0),
    )

    assert code == 0
    assert (
        tmp_path / "data" / "c30d_firmware_probe" / "20260531_223000" / "bootloader_probe_report.md"
    ).is_file()


def test_no_ros_or_ros2_imports_exist():
    source = (
        Path(__file__).resolve().parents[1] / "scripts" / "probe_c30d_stm32_bootloader.py"
    ).read_text(encoding="utf-8")
    assert "import rospy" not in source
    assert "import rclpy" not in source
