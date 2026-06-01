#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
from glob import glob
from pathlib import Path
import re
import shutil
import subprocess
from typing import Callable, Protocol

REPORT_ROOT = Path("data/c30d_firmware_probe")
PORT_PATTERNS = ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/serial/by-id/*")
FORBIDDEN_STM32FLASH_ARGS = {
    "-w",
    "--write",
    "-e",
    "--erase",
    "-u",
    "--unlock",
    "-k",
    "--read-protect",
    "-j",
    "--start",
    "-o",
    "--option",
}
INSTALL_GUIDANCE = (
    "Install stm32flash with your OS package manager, for example: "
    "sudo apt install stm32flash. Do not run write, erase, unlock, or readout-protection commands."
)


class CompletedProcessLike(Protocol):
    returncode: int
    stdout: str
    stderr: str


RunFn = Callable[..., CompletedProcessLike]
InputFn = Callable[[str], str]
WhichFn = Callable[[str], str | None]
ClockFn = Callable[[], datetime]


@dataclass(frozen=True)
class BootloaderProbeResult:
    serial_ports_found: tuple[str, ...]
    selected_port: str | None
    stm32flash_path: str | None
    command: tuple[str, ...] = ()
    command_stdout: str = ""
    command_stderr: str = ""
    returncode: int | None = None
    bootloader_responded: bool = False
    mcu_id: str | None = None
    readout_protection_status: str | None = None
    next_safe_step: str = ""


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only STM32 bootloader identification probe for the C30D. "
            "It never writes firmware, erases, unlocks readout protection, or sends C30D motion commands."
        )
    )
    parser.add_argument("--port", help="Serial download/BOOT-mode port to probe.")
    return parser


def list_likely_serial_ports(patterns: tuple[str, ...] = PORT_PATTERNS) -> tuple[str, ...]:
    ports: list[str] = []
    for pattern in patterns:
        ports.extend(glob(pattern))
    return tuple(sorted(dict.fromkeys(ports)))


def select_port(explicit_port: str | None, ports: tuple[str, ...]) -> str | None:
    if explicit_port:
        return explicit_port
    if len(ports) == 1:
        return ports[0]
    return None


def prompt_for_bootloader_setup(input_fn: InputFn = input) -> bool:
    print("C30D STM32 bootloader identification only. No firmware write/erase/unlock is allowed.")
    print("Before continuing:")
    print("- power robot safely")
    print("- set motor enable OFF")
    print("- put C30D into download/BOOT mode if available")
    response = input_fn("Type YES to run the read-only bootloader query: ")
    return response == "YES"


def validate_stm32flash_command(command: tuple[str, ...]) -> None:
    if len(command) != 2:
        raise ValueError("stm32flash probe command must be exactly: stm32flash <port>")
    forbidden = [arg for arg in command[1:] if arg in FORBIDDEN_STM32FLASH_ARGS]
    if forbidden:
        raise ValueError(f"forbidden stm32flash argument present: {', '.join(forbidden)}")


def build_stm32flash_probe_command(stm32flash_path: str, port: str) -> tuple[str, ...]:
    command = (stm32flash_path, port)
    validate_stm32flash_command(command)
    return command


def parse_mcu_id(output: str) -> str | None:
    patterns = (
        r"Device ID\s*:\s*(0x[0-9a-fA-F]+(?:\s*\([^\n]+\))?)",
        r"PID\s*:\s*(0x[0-9a-fA-F]+(?:\s*\([^\n]+\))?)",
        r"Chip ID\s*:\s*(0x[0-9a-fA-F]+(?:\s*\([^\n]+\))?)",
    )
    for pattern in patterns:
        match = re.search(pattern, output)
        if match:
            return match.group(1).strip()
    return None


def parse_readout_protection(output: str) -> str | None:
    patterns = (
        r"Readout Protection\s*:?\s*([^\n\r]+)",
        r"RDP\s*:?\s*([^\n\r]+)",
        r"readout protection\s*:?\s*([^\n\r]+)",
    )
    for pattern in patterns:
        match = re.search(pattern, output, flags=re.IGNORECASE)
        if match:
            return match.group(1).strip()
    return None


def bootloader_responded(returncode: int, output: str) -> bool:
    if returncode != 0:
        return False
    lowered = output.lower()
    failure_tokens = ("failed", "no response", "timeout", "error")
    if any(token in lowered for token in failure_tokens):
        return False
    success_tokens = ("device id", "bootloader", "version", "stm32")
    return any(token in lowered for token in success_tokens)


def next_safe_step_for(result: BootloaderProbeResult) -> str:
    if result.stm32flash_path is None:
        return "Install stm32flash, then rerun this read-only probe with --port."
    if result.selected_port is None:
        return "Select the C30D download/BOOT serial port with --port."
    if result.bootloader_responded:
        return "Record MCU ID/readout status; do not flash or erase. Continue protocol work or plan backup/recovery before any firmware action."
    return "Check BOOT/download wiring and selected serial port; do not use flash, erase, or unlock commands."


def run_stm32flash_probe(
    *,
    port: str,
    stm32flash_path: str,
    run_fn: RunFn = subprocess.run,
) -> tuple[tuple[str, ...], int, str, str]:
    command = build_stm32flash_probe_command(stm32flash_path, port)
    completed = run_fn(command, capture_output=True, text=True, check=False)
    return command, int(completed.returncode), completed.stdout, completed.stderr


def timestamped_report_dir(clock: ClockFn = datetime.now) -> Path:
    return REPORT_ROOT / clock().strftime("%Y%m%d_%H%M%S")


def render_report(result: BootloaderProbeResult) -> str:
    ports = "\n".join(f"- `{port}`" for port in result.serial_ports_found) or "- none"
    command = " ".join(result.command) if result.command else "not run"
    return "\n".join(
        [
            "# C30D STM32 Bootloader Probe Report",
            "",
            "## Safety Scope",
            "- Read-only STM32 bootloader identification only.",
            "- No firmware write, erase, unlock, readout-protection change, or C30D motion command was requested.",
            "",
            "## Serial Ports Found",
            ports,
            "",
            "## Selected Port",
            f"- `{result.selected_port}`" if result.selected_port else "- none",
            "",
            "## Tool",
            (
                f"- stm32flash: `{result.stm32flash_path}`"
                if result.stm32flash_path
                else "- stm32flash: missing"
            ),
            "",
            "## Command Run",
            f"- `{command}`",
            "",
            "## Bootloader Response",
            f"- bootloader_responded: {str(result.bootloader_responded).lower()}",
            f"- returncode: {result.returncode}",
            f"- MCU ID: {result.mcu_id or 'unknown'}",
            f"- readout protection status: {result.readout_protection_status or 'unknown'}",
            "",
            "## stdout",
            "```",
            result.command_stdout.rstrip(),
            "```",
            "",
            "## stderr",
            "```",
            result.command_stderr.rstrip(),
            "```",
            "",
            "## Next Safe Step",
            f"- {result.next_safe_step}",
            "",
        ]
    )


def write_report(result: BootloaderProbeResult, report_dir: Path) -> Path:
    report_dir.mkdir(parents=True, exist_ok=True)
    report_path = report_dir / "bootloader_probe_report.md"
    report_path.write_text(render_report(result), encoding="utf-8")
    return report_path


def run_probe(
    *,
    port: str | None,
    input_fn: InputFn = input,
    which_fn: WhichFn = shutil.which,
    run_fn: RunFn = subprocess.run,
) -> BootloaderProbeResult:
    ports = list_likely_serial_ports()
    selected_port = select_port(port, ports)
    stm32flash_path = which_fn("stm32flash")

    if selected_port is None:
        result = BootloaderProbeResult(
            serial_ports_found=ports,
            selected_port=None,
            stm32flash_path=stm32flash_path,
        )
        return result.__class__(**{**result.__dict__, "next_safe_step": next_safe_step_for(result)})

    if stm32flash_path is None:
        print("stm32flash not found.")
        print(INSTALL_GUIDANCE)
        result = BootloaderProbeResult(
            serial_ports_found=ports,
            selected_port=selected_port,
            stm32flash_path=None,
        )
        return result.__class__(**{**result.__dict__, "next_safe_step": next_safe_step_for(result)})

    if not prompt_for_bootloader_setup(input_fn=input_fn):
        result = BootloaderProbeResult(
            serial_ports_found=ports,
            selected_port=selected_port,
            stm32flash_path=stm32flash_path,
            next_safe_step="Probe refused by user confirmation; no bootloader query was run.",
        )
        return result

    command, returncode, stdout, stderr = run_stm32flash_probe(
        port=selected_port,
        stm32flash_path=stm32flash_path,
        run_fn=run_fn,
    )
    combined_output = stdout + "\n" + stderr
    responded = bootloader_responded(returncode, combined_output)
    result = BootloaderProbeResult(
        serial_ports_found=ports,
        selected_port=selected_port,
        stm32flash_path=stm32flash_path,
        command=command,
        command_stdout=stdout,
        command_stderr=stderr,
        returncode=returncode,
        bootloader_responded=responded,
        mcu_id=parse_mcu_id(combined_output),
        readout_protection_status=parse_readout_protection(combined_output),
    )
    return result.__class__(**{**result.__dict__, "next_safe_step": next_safe_step_for(result)})


def print_ports(ports: tuple[str, ...]) -> None:
    print("likely_serial_ports:")
    if not ports:
        print("  none")
        return
    for port in ports:
        print(f"  {port}")


def main(
    argv: list[str] | None = None,
    *,
    input_fn: InputFn = input,
    which_fn: WhichFn = shutil.which,
    run_fn: RunFn = subprocess.run,
    clock: ClockFn = datetime.now,
) -> int:
    args = build_parser().parse_args(argv)
    result = run_probe(
        port=args.port,
        input_fn=input_fn,
        which_fn=which_fn,
        run_fn=run_fn,
    )
    report_path = write_report(result, timestamped_report_dir(clock))
    print_ports(result.serial_ports_found)
    print(f"selected_port: {result.selected_port}")
    print(f"bootloader_responded: {str(result.bootloader_responded).lower()}")
    print(f"mcu_id: {result.mcu_id or 'unknown'}")
    print(f"readout_protection_status: {result.readout_protection_status or 'unknown'}")
    print(f"next_safe_step: {result.next_safe_step}")
    print(f"report_path: {report_path}")
    return 0 if result.bootloader_responded else 1


if __name__ == "__main__":
    raise SystemExit(main())
