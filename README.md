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

Run tests:

    pytest -q

Run lint:

    ruff check src tests scripts

Format code:

    black src tests scripts

Run Codex:

    codex

## Notes

This project is intentionally native and non-ROS2. The Raspberry Pi 5 handles low-level supervision, safety, hardware drivers, logging, and communication with the Jetson. The Jetson handles heavier compute such as vision, mapping, 3D reconstruction, and learned navigation.
