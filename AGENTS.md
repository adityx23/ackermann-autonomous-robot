# AGENTS.md

## Project

This repository is for a native, non-ROS2 Ackermann steering robot.

The Raspberry Pi 5 is the low-level robot supervisor. It runs motor/sensor interfaces, safety checks, logging, teleop, and communication with the Jetson.

The Jetson Orin Nano is used for heavier edge compute such as vision-based navigation, mapping, 3D reconstruction, learned planning, and policy inference.

Do not assume ROS2 is available. This project should run natively in Python unless explicitly requested otherwise.

## Main hardware

- Raspberry Pi 5 running the main robot supervisor
- C30D master robot controller interfaced with the Pi 5
- Two encoder DC motors controlled through the C30D
- Steering servo controlled through the C30D
- Luxonis OAK-D Lite depth camera
- RPLIDAR C1
- ICM-20948 IMU
- Jetson Orin Nano for heavy compute offload over the network

## Safety rules

- Never write code that moves motors automatically on import.
- Never run real motor commands unless the user explicitly asks.
- All motor/steering scripts must support a dry-run mode.
- Any script that can move the robot must print a clear warning before starting.
- Default speed limits must be conservative.
- If sensor data or Jetson commands are stale, the safe behavior is stop motors and hold/center steering.
- Keep low-level safety on the Raspberry Pi 5, not on the Jetson.

## Architecture rules

- Use src/ackermann_robot as the main Python package.
- Drivers go in src/ackermann_robot/drivers.
- Control logic goes in src/ackermann_robot/control.
- State estimation goes in src/ackermann_robot/estimation.
- Pi-to-Jetson communication goes in src/ackermann_robot/comms.
- Utility math, transforms, and timing go in src/ackermann_robot/utils.
- Configuration must live in config/*.yaml.
- Avoid hardcoded serial ports, I2C addresses, IP addresses, and calibration values.
- Use logging instead of print for reusable modules.
- Write small hardware test scripts in scripts/.
- Write unit tests in tests/ using mocks where real hardware is unavailable.
- Do not invent unknown C30D protocol details; create placeholders/interfaces until the real protocol is provided.

## Coding style

- Python 3 compatible.
- Use type hints where practical.
- Prefer dataclasses for messages/state.
- Keep drivers separate from control logic.
- Keep hardware access isolated behind clean interfaces.
- Do not hide exceptions silently; log them with useful context.

## Common commands

Activate environment:

    source .venv/bin/activate

Run tests:

    pytest -q

Run lint:

    ruff check src tests scripts

Format:

    black src tests scripts

## Development workflow

Before major edits:

- Inspect the existing tree.
- Make a short implementation plan.
- Modify the smallest necessary set of files.
- Add or update tests when possible.
- Do not move real motors unless the user explicitly asks for a hardware movement test.
