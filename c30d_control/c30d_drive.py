#!/usr/bin/env python3
import os
import sys
import time
import termios
import select

PORT = "/dev/c30d"
OPEN_SETTLE_SECONDS = 0.1
COMMAND_GAP_SECONDS = 0.1

COMMANDS = {
    "forward": "FWD_STRAIGHT_HOLD",
    "left": "FWD_LEFT_HOLD",
    "right": "FWD_RIGHT_HOLD",
    "reverse": "REV_STRAIGHT_HOLD",
    "rev_left": "REV_LEFT_HOLD",
    "rev_right": "REV_RIGHT_HOLD",

    "nudge": "FWD_STRAIGHT_SHORT",
    "nudge_left": "FWD_LEFT_SHORT",
    "nudge_right": "FWD_RIGHT_SHORT",
    "rev_nudge": "REV_STRAIGHT_SHORT",
    "rev_nudge_left": "REV_LEFT_SHORT",
    "rev_nudge_right": "REV_RIGHT_SHORT",

    "center": "CENTER",
}

STATUS_CMDS = [
    "PING",
    "STATUS",
    "ACKERMANN_STATUS",
    "SERVO_STATUS",
    "DRIVE_STATUS",
    "COUNTS",
    "OUT_STATUS",
]

def open_serial():
    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    time.sleep(OPEN_SETTLE_SECONDS)
    return fd

def drain(fd, seconds=0.4):
    end = time.time() + seconds
    out = b""
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            try:
                out += os.read(fd, 4096)
            except BlockingIOError:
                pass
    return out

def cmd(fd, s, wait=1.0):
    drain(fd, 0.25)
    data = (s + "\r\n").encode()
    sent = 0
    while sent < len(data):
        try:
            written = os.write(fd, data[sent:])
            if written == 0:
                raise RuntimeError("Serial write made no progress")
            sent += written
        except BlockingIOError:
            select.select([], [fd], [], 0.05)
    termios.tcdrain(fd)
    response = drain(fd, wait).decode(errors="replace").strip()
    time.sleep(COMMAND_GAP_SECONDS)
    return response

def reliable_cmd(fd, s, expected=None, tries=5, wait=1.2):
    last = ""
    for i in range(1, tries + 1):
        resp = cmd(fd, s, wait)
        last = resp
        print(resp)
        if expected is None or expected in resp:
            return resp
        time.sleep(0.25)
    return last

def arm_and_motion(fd, motion_cmd):
    arm = reliable_cmd(fd, "ARM_PULSE", "ACK ARM_PULSE", tries=5, wait=1.0)
    if "ACK ARM_PULSE" not in arm:
        print("ERROR: Could not arm. Sending STOP.")
        print(cmd(fd, "STOP", 1.0))
        return 1

    resp = reliable_cmd(fd, motion_cmd, "ACK ACKERMANN_DONE", tries=5, wait=3.0)
    print(cmd(fd, "STOP", 1.0))

    if "ACK ACKERMANN_DONE" not in resp:
        print("ERROR: Motion command did not ACK cleanly.")
        return 1

    return 0

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 c30d_drive.py [status|stop|center|forward|left|right|reverse|rev_left|rev_right|nudge|nudge_left|nudge_right|rev_nudge]")
        return 2

    action = sys.argv[1].lower()

    fd = open_serial()
    try:
        if action == "status":
            print("=" * 60)
            print("PING")
            ping = reliable_cmd(fd, "PING", "PONG C30D V3_0", tries=3, wait=1.0)
            if "PONG C30D V3_0" not in ping:
                print("ERROR: PING failed after 3 attempts.")
                return 1

            for c in STATUS_CMDS[1:]:
                print("=" * 60)
                print(c)
                print(cmd(fd, c, 1.0))
            return 0

        if action == "stop":
            print(cmd(fd, "STOP", 1.0))
            return 0

        if action not in COMMANDS:
            print(f"Unknown action: {action}")
            print("Valid: status, stop, center, forward, left, right, reverse, rev_left, rev_right, nudge, nudge_left, nudge_right, rev_nudge, rev_nudge_left, rev_nudge_right")
            return 2

        return arm_and_motion(fd, COMMANDS[action])

    finally:
        os.close(fd)

if __name__ == "__main__":
    raise SystemExit(main())
