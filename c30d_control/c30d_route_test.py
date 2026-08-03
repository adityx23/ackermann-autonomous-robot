#!/usr/bin/env python3
"""Run a repeatable, manually confirmed C30D route."""

import argparse
import csv
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
NAV_SCRIPT = SCRIPT_DIR / "c30d_nav_primitives.py"
DRIVE_SCRIPT = SCRIPT_DIR / "c30d_drive.py"
LOG_DIR = SCRIPT_DIR / "logs"
COMMAND_TIMEOUT_SECONDS = 120
STATUS_TIMEOUT_SECONDS = 25

ROUTE = (
    ("forward_cm", "30"),
    ("turn_left_deg", "60"),
    ("forward_cm", "30"),
    ("turn_right_deg", "60"),
    ("forward_cm", "30"),
)

CSV_FIELDS = (
    "timestamp",
    "run_number",
    "segment_number",
    "command",
    "success",
    "duration_seconds",
    "battery_mv",
    "status_summary",
    "error",
)


class RouteError(RuntimeError):
    """Raised when a route subprocess or status check fails."""


def format_command(script, *arguments):
    return " ".join(("python3", script.name, *arguments))


def run_script(script, *arguments, timeout=COMMAND_TIMEOUT_SECONDS):
    command = ["python3", str(script), *arguments]
    display = format_command(script, *arguments)
    print(f">>> {display}")

    try:
        completed = subprocess.run(
            command,
            cwd=SCRIPT_DIR,
            timeout=timeout,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
    except subprocess.TimeoutExpired as exc:
        raise RouteError(f"{display} timed out") from exc
    except OSError as exc:
        raise RouteError(f"could not run {display}: {exc}") from exc

    output = completed.stdout or ""
    if output:
        print(output, end="" if output.endswith("\n") else "\n")

    if completed.returncode != 0:
        raise RouteError(f"{display} exited with code {completed.returncode}")
    return output


def run_status():
    output = run_script(
        DRIVE_SCRIPT, "status", timeout=STATUS_TIMEOUT_SECONDS
    )
    if "PONG C30D V3_0" not in output:
        raise RouteError("status response did not contain PONG C30D V3_0")
    return output


def stop_robot():
    try:
        run_script(DRIVE_SCRIPT, "stop")
        return None
    except RouteError as exc:
        print(f"SAFETY WARNING: stop failed: {exc}", file=sys.stderr)
        return str(exc)


def status_summary(output):
    return " | ".join(line.strip() for line in output.splitlines() if line.strip())


def battery_mv(output):
    patterns = (
        r"\bbattery_mv\s*[=:]\s*(\d+)",
        r"\bbattery\s*[=:]\s*(\d+)\s*mV\b",
        r"\bbatt(?:ery)?_mv\s*[=:]\s*(\d+)",
        r"\bmv\s*[=:]\s*(\d+)",
    )
    for pattern in patterns:
        match = re.search(pattern, output, re.IGNORECASE)
        if match:
            return match.group(1)
    return ""


def print_route():
    print("Route:")
    for number, (action, value) in enumerate(ROUTE, start=1):
        print(f"  {number}. {format_command(NAV_SCRIPT, action, value)}")
    print("  End: python3 c30d_drive.py center")
    print("       python3 c30d_drive.py stop")
    print("       python3 c30d_drive.py status")


def confirm_run(number, total):
    answer = input(
        f"Run {number}/{total}: press Enter to begin the complete route, "
        "or type q/quit to stop: "
    ).strip().lower()
    return answer not in ("q", "quit")


def confirm_segment(number, action, value):
    answer = input(
        f"Segment {number}/{len(ROUTE)}: {action} {value}. "
        "Press Enter to run, or type q/quit to stop: "
    ).strip().lower()
    return answer not in ("q", "quit")


def finish_route():
    print("\nRoute complete. Centering, stopping, and reading final status.")
    errors = []
    final_status = ""

    try:
        run_script(DRIVE_SCRIPT, "center")
    except RouteError as exc:
        errors.append(str(exc))

    stop_error = stop_robot()
    if stop_error:
        errors.append(stop_error)

    try:
        final_status = run_status()
    except RouteError as exc:
        errors.append(str(exc))
        stop_error = stop_robot()
        if stop_error:
            errors.append(stop_error)

    return final_status, errors


def write_log(records):
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")
    path = LOG_DIR / f"c30d_route_repeatability_{stamp}.csv"
    with path.open("w", newline="", encoding="utf-8") as log_file:
        writer = csv.DictWriter(log_file, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(records)
    return path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run the manually confirmed C30D route repeatability test."
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=1,
        choices=range(1, 6),
        metavar="N",
        help="number of complete route runs (1-5; default: 1)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the route without running any subprocesses",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    print_route()

    if args.dry_run:
        return 0

    for required_script in (NAV_SCRIPT, DRIVE_SCRIPT):
        if not required_script.is_file():
            print(f"Required script not found: {required_script}", file=sys.stderr)
            return 1

    records = []
    completed_runs = 0
    failed_segments = 0
    exit_code = 0
    cancelled = False

    try:
        for run_number in range(1, args.runs + 1):
            if not confirm_run(run_number, args.runs):
                print("Repeatability test cancelled.")
                cancelled = True
                break

            run_failed = False
            for segment_number, (action, value) in enumerate(ROUTE, start=1):
                if not confirm_segment(segment_number, action, value):
                    print("Route cancelled. Sending stop.")
                    stop_robot()
                    cancelled = True
                    run_failed = True
                    break

                started = time.monotonic()
                errors = []
                output = ""
                command = format_command(NAV_SCRIPT, action, value)

                try:
                    run_script(NAV_SCRIPT, action, value)
                except RouteError as exc:
                    errors.append(str(exc))

                stop_error = stop_robot()
                if stop_error:
                    errors.append(stop_error)

                try:
                    output = run_status()
                except RouteError as exc:
                    errors.append(str(exc))
                    if not stop_error:
                        retry_stop_error = stop_robot()
                        if retry_stop_error:
                            errors.append(retry_stop_error)

                success = not errors
                records.append(
                    {
                        "timestamp": datetime.now().astimezone().isoformat(),
                        "run_number": run_number,
                        "segment_number": segment_number,
                        "command": command,
                        "success": success,
                        "duration_seconds": f"{time.monotonic() - started:.3f}",
                        "battery_mv": battery_mv(output),
                        "status_summary": status_summary(output),
                        "error": "; ".join(errors),
                    }
                )

                if not success:
                    failed_segments += 1
                    run_failed = True
                    exit_code = 1
                    print("Route aborted.", file=sys.stderr)
                    break

            if cancelled or run_failed:
                break

            final_started = time.monotonic()
            final_status, final_errors = finish_route()
            records.append(
                {
                    "timestamp": datetime.now().astimezone().isoformat(),
                    "run_number": run_number,
                    "segment_number": "",
                    "command": "CENTER; STOP; STATUS",
                    "success": not final_errors,
                    "duration_seconds": f"{time.monotonic() - final_started:.3f}",
                    "battery_mv": battery_mv(final_status),
                    "status_summary": status_summary(final_status),
                    "error": "; ".join(final_errors),
                }
            )
            if final_errors:
                exit_code = 1
                print("Final route cleanup/status failed.", file=sys.stderr)
                break

            completed_runs += 1

    except (EOFError, KeyboardInterrupt):
        print("\nInput interrupted. Sending stop.", file=sys.stderr)
        stop_robot()
        exit_code = 130

    log_path = write_log(records)
    print(f"\nCSV log: {log_path}")
    print(
        f"Final summary: completed runs {completed_runs}/{args.runs}; "
        f"failed segments: {failed_segments}"
    )
    if cancelled and exit_code == 0:
        print("Test ended by user confirmation.")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
