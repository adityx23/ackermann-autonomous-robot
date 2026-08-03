#!/usr/bin/env python3
import subprocess
import sys
import time
import math

NUDGE_CM = 15.24
TURN_DEG = 59.73

DRIVE = "./c30d_drive.py"


def run_drive(cmd: str) -> None:
    print(f"\n>>> {cmd}")
    result = subprocess.run(
        ["python3", DRIVE, cmd],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=25,
    )
    print(result.stdout)
    if result.returncode != 0:
        raise RuntimeError(f"Command failed: {cmd}")


def check_status() -> None:
    result = subprocess.run(
        ["python3", DRIVE, "status"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=25,
    )
    print(result.stdout)
    if "low=1" in result.stdout:
        raise RuntimeError("Battery low according to firmware status. Aborting.")
    if "PONG C30D V3_0" not in result.stdout:
        raise RuntimeError("C30D did not respond correctly. Aborting.")


def safe_stop_center() -> None:
    try:
        run_drive("stop")
    except Exception as exc:
        print(f"WARNING: Cleanup stop failed: {exc}")

    try:
        run_drive("center")
    except Exception as exc:
        print(f"WARNING: Cleanup center failed: {exc}")

    try:
        run_drive("stop")
    except Exception as exc:
        print(f"WARNING: Cleanup stop failed: {exc}")


def forward_cm(cm: float) -> None:
    if cm <= 0:
        raise ValueError("forward_cm must be positive")

    count = max(1, round(cm / NUDGE_CM))
    actual = count * NUDGE_CM

    print(f"Requested forward: {cm:.1f} cm")
    print(f"Using {count} nudge command(s), expected distance: {actual:.1f} cm")

    confirm = input("Place robot safely. Motor enable ON. Press Enter to run, or type quit: ")
    if confirm.strip().lower() in ("q", "quit", "no", "n"):
        print("Cancelled.")
        return

    check_status()
    run_drive("center")

    for i in range(count):
        print(f"\nForward nudge {i + 1}/{count}")
        check_status()
        run_drive("nudge")
        run_drive("stop")
        time.sleep(1.0)

    safe_stop_center()


def turn_deg(direction: str, deg: float) -> None:
    if deg <= 0:
        raise ValueError("turn angle must be positive")
    if direction not in ("left", "right"):
        raise ValueError("direction must be left or right")

    count = max(1, round(deg / TURN_DEG))
    actual = count * TURN_DEG

    print(f"Requested turn {direction}: {deg:.1f} deg")
    print(f"Using {count} nudge_{direction} command(s), expected heading change: {actual:.1f} deg")

    if abs(actual - deg) > 20:
        print("WARNING: Requested angle is much smaller/different than calibrated primitive.")
        print("v3.0 turn primitive is coarse: ~59.73 deg per command.")

    confirm = input("Place robot safely. Motor enable ON. Press Enter to run, or type quit: ")
    if confirm.strip().lower() in ("q", "quit", "no", "n"):
        print("Cancelled.")
        return

    check_status()
    run_drive("center")

    cmd = "nudge_left" if direction == "left" else "nudge_right"

    for i in range(count):
        print(f"\nTurn {direction} nudge {i + 1}/{count}")
        check_status()
        run_drive(cmd)
        run_drive("stop")
        time.sleep(1.0)

    safe_stop_center()


def usage() -> None:
    print(
        "Usage:\n"
        "  python3 c30d_nav_primitives.py forward_cm <cm>\n"
        "  python3 c30d_nav_primitives.py turn_left_deg <deg>\n"
        "  python3 c30d_nav_primitives.py turn_right_deg <deg>\n"
        "\n"
        "Examples:\n"
        "  python3 c30d_nav_primitives.py forward_cm 30\n"
        "  python3 c30d_nav_primitives.py turn_left_deg 60\n"
        "  python3 c30d_nav_primitives.py turn_right_deg 120\n"
    )


def main() -> None:
    if len(sys.argv) != 3:
        usage()
        sys.exit(1)

    action = sys.argv[1]
    value = float(sys.argv[2])

    if action == "forward_cm":
        forward_cm(value)
    elif action == "turn_left_deg":
        turn_deg("left", value)
    elif action == "turn_right_deg":
        turn_deg("right", value)
    else:
        usage()
        sys.exit(1)


if __name__ == "__main__":
    main()
