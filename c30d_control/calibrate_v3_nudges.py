#!/usr/bin/env python3
"""Interactive, safety-focused calibration for C30D V3_0 nudge commands."""

import csv
import re
import subprocess
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path


WRAPPER = Path(__file__).resolve().with_name("c30d_drive.py")
LOG_DIR = Path(__file__).resolve().parent / "logs"
COMMANDS = ("nudge", "nudge_left", "nudge_right")
DEFAULT_TRIALS = 3
COMMAND_TIMEOUT_SECONDS = 20
LOW_BATTERY_RE = re.compile(r"\blow\s*=\s*1\b", re.IGNORECASE)

CSV_FIELDS = (
    "timestamp",
    "command",
    "trial",
    "result",
    "distance_cm",
    "heading_change_deg",
    "notes",
)


class CalibrationError(RuntimeError):
    """Raised when a wrapper command cannot be completed safely."""


def run_wrapper(action):
    """Run one c30d_drive.py action and return its combined output."""
    try:
        completed = subprocess.run(
            [sys.executable, str(WRAPPER), action],
            cwd=WRAPPER.parent,
            capture_output=True,
            text=True,
            timeout=COMMAND_TIMEOUT_SECONDS,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        raise CalibrationError(
            f"{action!r} timed out after {COMMAND_TIMEOUT_SECONDS} seconds"
        ) from exc
    except OSError as exc:
        raise CalibrationError(f"could not run {action!r}: {exc}") from exc

    output = "\n".join(
        part.strip() for part in (completed.stdout, completed.stderr) if part.strip()
    )
    if output:
        print(output)

    if completed.returncode != 0:
        raise CalibrationError(
            f"{action!r} failed with exit code {completed.returncode}"
        )
    return output


def stop_robot():
    """Request STOP, reporting an error without hiding an earlier failure."""
    try:
        run_wrapper("stop")
        return None
    except CalibrationError as exc:
        print(f"SAFETY WARNING: STOP failed: {exc}", file=sys.stderr)
        return str(exc)


def checked_status():
    """Read status and abort immediately if the firmware reports low=1."""
    output = run_wrapper("status")
    if LOW_BATTERY_RE.search(output):
        raise CalibrationError("low battery reported (low=1)")
    return output


def prompt_trial_count(command):
    while True:
        answer = input(
            f"How many trials for {command}? [{DEFAULT_TRIALS}]: "
        ).strip()
        if not answer:
            return DEFAULT_TRIALS
        try:
            count = int(answer)
        except ValueError:
            print("Enter a positive whole number.")
            continue
        if count > 0:
            return count
        print("Enter a positive whole number.")


def prompt_measurement(label):
    while True:
        answer = input(f"Enter measured {label}: ").strip()
        try:
            return float(answer)
        except ValueError:
            print("Enter a number, for example 12.5 or -3.")


def confirm_motion(action):
    answer = input(
        f"Press Enter to run {action}, or type skip/quit: "
    ).strip().lower()
    if answer == "quit":
        return "quit"
    if answer == "skip":
        return "skip"
    if answer:
        print("Motion not confirmed.")
        return "skip"
    return "run"


def write_row(writer, log_file, command, trial, result, **values):
    row = {
        "timestamp": datetime.now().astimezone().isoformat(timespec="seconds"),
        "command": command,
        "trial": trial,
        "result": result,
        "distance_cm": "",
        "heading_change_deg": "",
        "notes": "",
    }
    row.update(values)
    writer.writerow(row)
    log_file.flush()


def run_trial(command, trial, writer, log_file):
    prompt = input(
        "Place robot at start line, hand near motor enable, press Enter to run "
        "or type skip/quit."
    ).strip().lower()
    if prompt == "quit":
        write_row(writer, log_file, command, trial, "quit")
        return "quit", None
    if prompt == "skip":
        write_row(writer, log_file, command, trial, "skipped")
        return "continue", None
    if prompt:
        print("Trial not confirmed; skipping.")
        write_row(writer, log_file, command, trial, "skipped")
        return "continue", None

    try:
        print("\nPre-center safety status:")
        checked_status()
    except CalibrationError as exc:
        stop_error = stop_robot()
        notes = str(exc)
        if stop_error:
            notes += f"; STOP failed: {stop_error}"
        write_row(writer, log_file, command, trial, "aborted", notes=notes)
        print(f"ABORTING CALIBRATION: {exc}", file=sys.stderr)
        return "abort", None

    center_choice = confirm_motion("center")
    if center_choice == "quit":
        write_row(writer, log_file, command, trial, "quit")
        return "quit", None
    if center_choice == "skip":
        write_row(writer, log_file, command, trial, "skipped")
        return "continue", None

    try:
        run_wrapper("center")
    except CalibrationError as exc:
        stop_error = stop_robot()
        notes = str(exc)
        if stop_error:
            notes += f"; STOP failed: {stop_error}"
        write_row(writer, log_file, command, trial, "failed", notes=notes)
        print(f"Center failed; calibration aborted: {exc}", file=sys.stderr)
        return "abort", None

    if stop_robot() is not None:
        write_row(
            writer,
            log_file,
            command,
            trial,
            "aborted",
            notes="STOP failed after center",
        )
        return "abort", None

    try:
        print("\nPre-motion safety status:")
        checked_status()
    except CalibrationError as exc:
        stop_error = stop_robot()
        notes = str(exc)
        if stop_error:
            notes += f"; STOP failed: {stop_error}"
        write_row(writer, log_file, command, trial, "aborted", notes=notes)
        print(f"ABORTING CALIBRATION: {exc}", file=sys.stderr)
        return "abort", None

    motion_choice = confirm_motion(command)
    if motion_choice == "quit":
        write_row(writer, log_file, command, trial, "quit")
        return "quit", None
    if motion_choice == "skip":
        write_row(writer, log_file, command, trial, "skipped")
        return "continue", None

    motion_error = None
    stop_error = None
    try:
        run_wrapper(command)
    except CalibrationError as exc:
        motion_error = str(exc)
    finally:
        stop_error = stop_robot()

    if motion_error or stop_error:
        notes = "; ".join(
            message
            for message in (
                f"motion failed: {motion_error}" if motion_error else "",
                f"STOP failed: {stop_error}" if stop_error else "",
            )
            if message
        )
        write_row(writer, log_file, command, trial, "failed", notes=notes)
        print(f"Trial failed: {notes}", file=sys.stderr)
        return ("abort" if stop_error else "continue"), None

    try:
        print("\nPost-motion status:")
        run_wrapper("status")
    except CalibrationError as exc:
        print(f"Post-motion status failed: {exc}", file=sys.stderr)

    distance = prompt_measurement("distance_cm")
    heading = prompt_measurement("heading_change_deg")
    notes = input("Enter notes: ").strip()
    write_row(
        writer,
        log_file,
        command,
        trial,
        "completed",
        distance_cm=distance,
        heading_change_deg=heading,
        notes=notes,
    )
    return "continue", (distance, heading)


def print_averages(measurements):
    print("\nAverages (completed trials only):")
    for command in measurements:
        values = measurements[command]
        if not values:
            print(f"  {command}: no completed trials")
            continue
        average_distance = sum(value[0] for value in values) / len(values)
        average_heading = sum(value[1] for value in values) / len(values)
        print(
            f"  {command}: n={len(values)}, "
            f"distance_cm={average_distance:.2f}, "
            f"heading_change_deg={average_heading:.2f}"
        )


def main():
    if not WRAPPER.is_file():
        print(f"Wrapper not found: {WRAPPER}", file=sys.stderr)
        return 1

    commands = list(COMMANDS)
    include_forward = input(
        "Include forward calibration? Type yes to include it [no]: "
    ).strip().lower()
    if include_forward == "yes":
        commands.append("forward")

    trial_counts = {
        command: prompt_trial_count(command) for command in commands
    }

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    log_path = LOG_DIR / f"v3_nudge_calibration_{timestamp}.csv"
    measurements = defaultdict(list)
    should_stop = False

    print(f"\nLogging all trials to {log_path}")
    print("No motion runs without an immediate confirmation prompt.")

    try:
        with log_path.open("w", newline="", encoding="utf-8") as log_file:
            writer = csv.DictWriter(log_file, fieldnames=CSV_FIELDS)
            writer.writeheader()
            log_file.flush()

            for command in commands:
                for trial in range(1, trial_counts[command] + 1):
                    print(f"\n--- {command}: trial {trial}/{trial_counts[command]} ---")
                    outcome, measurement = run_trial(
                        command, trial, writer, log_file
                    )
                    if measurement is not None:
                        measurements[command].append(measurement)
                    if outcome in ("quit", "abort"):
                        should_stop = True
                        break
                if should_stop:
                    break
    except (EOFError, KeyboardInterrupt):
        print("\nInput interrupted. Sending STOP.", file=sys.stderr)
        stop_robot()
        return 130
    except OSError as exc:
        print(f"Could not write calibration log: {exc}", file=sys.stderr)
        stop_robot()
        return 1

    print_averages({command: measurements[command] for command in commands})
    print(f"CSV log: {log_path}")
    return 1 if should_stop else 0


if __name__ == "__main__":
    raise SystemExit(main())
