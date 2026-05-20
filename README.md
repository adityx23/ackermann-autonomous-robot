# Ackermann Robot Native Stack

Native non-ROS2 control stack for an Ackermann steering robot.

## Hardware

- Raspberry Pi 5 robot supervisor
- C30D master robot controller
- Two encoder DC motors
- Steering servo
- Luxonis OAK-D Lite depth camera
- RPLIDAR C1
- ICM-20948 IMU
- Jetson Orin Nano for heavy compute offload

## Development

Activate the environment:

    source .venv/bin/activate

Run the full local check:

    make check

Run tests:

    pytest -q

Run the dry-run supervisor:

    python scripts/run_dry_supervisor.py

The dry supervisor runs a bounded mock-only loop by default. It creates `MockC30DDriver`,
does not open serial ports, does not access OAK-D, RPLIDAR, or IMU hardware, and records
only neutral mock drive commands unless a test injects a different command.

Run lint:

    ruff check src tests scripts

Format code:

    black src tests scripts

Run Codex:

    codex

## Configuration

Runtime configuration lives in `config/*.yaml`. The current mock-only stack reads robot
geometry and limits, safety timeouts, sensor connection settings, and network endpoints
from those files before any real hardware drivers are added.

## Hardware Discovery

These scripts are manual hardware discovery checks. They do not move motors or steering.

List and briefly connect to an OAK-D Lite without starting a long stream:

    python scripts/test_oak_detect.py

Open and close the RPLIDAR C1 serial port:

    python scripts/test_rplidar_port.py --port /dev/rplidar --baud 460800

Capture one OAK-D Lite RGB frame plus a raw depth frame when stereo depth is available:

    python scripts/oak_capture_once.py

The OAK capture saves files under `data/oak_tests/`. RGB is saved as `.png`; depth is
saved as a raw NumPy `.npy` array in millimeters for later processing.

Capture a short finite RPLIDAR C1 scan sample:

    python scripts/rplidar_scan_sample.py --port /dev/rplidar --baud 460800 --duration 5

RPLIDAR samples are saved as CSV files under `data/rplidar_tests/` with timestamp,
angle, distance, and quality columns. These raw sensor files will feed the custom
SLAM pipeline we build later.

Capture passive C30D bytes without writing anything to the controller:

    python scripts/capture_c30d_passive.py --port /dev/c30d --baud 115200 --duration 5

Inspect a passive C30D capture without assigning protocol field meanings:

    python scripts/analyze_c30d_capture.py data/c30d_captures/c30d_capture_YYYYMMDD_HHMMSS.bin

Analyze the newest saved C30D capture:

    python scripts/analyze_c30d_capture.py --latest

The analyzer works only from saved `.bin` files. It reports byte counts, marker counts,
candidate frames between `0x7B` and `0x7D`, and frame length distribution without assigning
protocol field meanings.

## Logs

Use `setup_logging()` from `ackermann_robot.utils.logging_utils` to create a timestamped
run folder under `logs/`, such as `logs/run_YYYYMMDD_HHMMSS/`. Each run folder contains
`robot.log` for structured supervisor logs.

## Notes

This project is intentionally native and non-ROS2. The Raspberry Pi 5 handles low-level supervision, safety, hardware drivers, logging, and communication with the Jetson. The Jetson handles heavier compute such as vision, mapping, 3D reconstruction, and learned navigation.
