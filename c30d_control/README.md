# C30D Firmware Control Tools

Host-side Python tools for the custom C30D V3.0 command firmware used by the robot.
They communicate with the controller over `/dev/c30d` at 115200 baud and cover direct
drive commands, calibrated navigation primitives, interactive calibration, and route
repeatability tests.

## Contents

- `c30d_drive.py` — low-level status, stop, centering, and armed motion commands
- `c30d_nav_primitives.py` — distance and heading helpers built from calibrated nudges
- `calibrate_v3_nudges.py` — interactive ground-calibration workflow
- `c30d_route_test.py` — manually confirmed repeatability route with CSV logging
- `config/v3_calibration.txt` — measured V3.0 ground-calibration results

## Safety warning

These scripts can command real motors and steering hardware. Unlike the mock and
offline-analysis tools elsewhere in this repository, `c30d_drive.py` is a live serial
control path and does not provide a dry-run mode. Use it only with the robot physically
secured, a person at the motor-enable control, and a clear test area. The higher-level
calibration and route tools require interactive confirmation and issue stop commands on
failure, but those safeguards do not replace physical precautions.

Generated calibration and route logs belong in `c30d_control/logs/` and are intentionally
excluded from version control.
