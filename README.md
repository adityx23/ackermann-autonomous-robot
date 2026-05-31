# Ackermann Robot Native Stack

Native non-ROS2 control stack for an Ackermann steering robot.

## Hardware

- Raspberry Pi 5 robot supervisor
- C30D integrated motor, steering-servo, encoder, and onboard IMU controller
- Two encoder DC motors connected directly to the C30D through 6-pin JST connectors
- Steering servo connected directly to the C30D
- 12V battery connected to the C30D motor/controller power path
- Raspberry Pi powered separately from the C30D/motor power path
- Luxonis OAK-D Lite depth camera
- RPLIDAR C1
- C30D-integrated IMU feedback
- Jetson Orin Nano for heavy compute offload

With the current wiring, motor and steering commands must go through the C30D unless the
robot is rewired with separate motor drivers, encoder wiring, steering-servo wiring, and
IMU access. See `docs/c30d_integrated_architecture.md`.

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

Run a native read-only sensor preflight check:

    python scripts/check_robot_sensors.py

Run only selected checks for a shorter hardware smoke test:

    python scripts/check_robot_sensors.py --duration 3 --check-c30d --no-check-rplidar --no-check-oak

Use explicit device paths:

    python scripts/check_robot_sensors.py --c30d-port /dev/c30d --rplidar-port /dev/rplidar

The preflight checker reports `/dev/c30d`, `/dev/rplidar`, `data/`, and `data/runs/`
status, opens C30D read-only when enabled, captures RPLIDAR read-only for a bounded
duration, and attempts one low-bandwidth RGB-only OAK-D Lite frame. It prints a PASS/FAIL
summary and returns nonzero if any enabled check fails. For C30D it also reports invalid
feedback checksum count/percentage and min/mean/max `candidate_battery_mV`. Provisional
candidate battery thresholds live in `config/battery_safety.yaml`: warn below 10800 mV,
block motor-test readiness below 10500 mV, and critical below 10200 mV. It never writes
to C30D, never sends motor or steering commands, and does not use ROS2.

Print the Pi-side command safety and arming status:

    python scripts/robot_safety_status.py

Print the current C30D command protocol status:

    python scripts/c30d_command_protocol_status.py

Current board identification status: the controller appears to be a WHEELTEC C30D ROS
bottom-layer controller candidate with STM32F407VET6 MCU, Raspberry Pi / ROS host
communication on serial port 3, one-key serial download on serial port 1, direct motor
A/B/C/D interfaces, C30D steering/servo expansion, motor enable switch, 6V-17V power
input, CAN, PS2/controller, Bluetooth module, and SWD interfaces. The IMU is tracked as
ICM20948 on the new version or MPU6050 on the old version. This board identification
does not establish the command protocol; real motor commands remain disabled.

Build a non-transmitting C30D-like command hypothesis frame offline:

    python scripts/build_c30d_hypothesis_frame.py 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
    python scripts/build_c30d_hypothesis_frame.py --payload-hex "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00"

Build a non-transmitting Wheeltec-documented 11-byte native host command candidate offline:

    python scripts/build_c30d_host_command_frame.py --target-x 0.0 --target-z 0.0

Wheeltec documentation uses ROS terminology, but this project treats the frame as native
Python/serial host-to-C30D tooling only. No ROS or ROS2 runtime is required or used. This
builder uses the documented candidate frame: `0x7B` header, two reserved/control bytes,
signed int16 big-endian target X/Y/Z values, XOR checksum over bytes 0-8, and `0x7D` end.
Ackermann helpers keep target Y at zero by default. Float scaling by 1000 is
documentation-derived and not live-tested. Output is non-transmitting and prints
`transmit_allowed: false` and `real_write_disabled: true`; real writes remain disabled.

The hypothesis builder accepts exactly 21 hex payload bytes for frame bytes 1-21, either
positionally or through `--payload-hex`, and prints a 24-byte frame shape using `0x7B`
start, XOR checksum over bytes 0-21 at byte 22, and `0x7D` end. Output is labeled
`unverified_hypothesis` with `protocol_known: false` and `transmit_allowed: false`. It
is offline tooling only: do not send these bytes to the C30D.

Run the guarded future C30D write-test harness without transmitting anything:

    python scripts/c30d_write_test_harness.py --armed --wheels-lifted --manual-enable --i-understand-risk --packet-hex "7b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 7b 7d"

Strong warning: this harness is not an enabled write path. It validates only supplied
packet shapes: old 24-byte feedback-like hypotheses and documented-candidate 11-byte native
host command frames. Default behavior refuses to run, all guard flags are required, and even
when all guards and a valid packet shape are supplied it still prints
`serial_write_allowed: false`, `real_write_disabled_in_code: true`, and `no bytes sent`.
It does not open `/dev/c30d`, does not write serial bytes, and must not be used as
evidence that a command packet is valid.

First write experiment planning:

    docs/c30d_first_write_experiment_plan.md

Check first-write readiness without transmitting anything:

    python scripts/c30d_first_write_readiness.py --wheels-lifted --robot-restrained --manual-power-cutoff-ready --motor-enable-switch-reviewed --i-understand-this-is-not-a-motor-test

The readiness checker runs or consumes read-only preflight results, builds the native zero
host command frame internally, validates its 11-byte shape and checksum, reports the
candidate battery voltage, and prints `real_write_enabled: false` plus `no bytes sent`.
For first-write readiness it defaults to a 5 second C30D-only stability window that checks
C30D feedback, data directories, frame rate, invalid checksum count, and battery. It does
not require RPLIDAR or OAK for a zero/neutral C30D serial frame. Use
`--full-sensor-preflight` when you intentionally want C30D plus RPLIDAR and OAK checked.
Readiness fails if candidate battery voltage is below the warning threshold, C30D frame
rate is below threshold, or any invalid checksum appears during the stability window. If
invalid checksums appear, rerun after checking USB/serial stability. It does not open a
write path and does not move motors or steering.

Send the native zero/neutral host command frame exactly once:

    python scripts/send_c30d_zero_frame_once.py --armed --manual-enable --wheels-lifted --robot-restrained --manual-power-cutoff-ready --motor-enable-switch-reviewed --i-understand-this-sends-a-real-serial-frame --c30d-only-preflight

Strong warning: this is a real serial write path. Default behavior refuses to write, all
listed guard flags are required, and the script runs the same 5 second C30D-only
first-write readiness check internally before opening `/dev/c30d`. It only builds and
validates the hardcoded native zero/neutral frame `7b 00 00 00 00 00 00 00 00 7b 7d`
using the native host command frame builder. It accepts no packet, speed, steering, motor
pulse, or target inputs. The script prints the exact frame hex before writing, then writes
those 11 bytes once, flushes, and closes. It reports `real_write_performed: true`,
`bytes_written: 11`, `frame_hex`, and `warning: zero/neutral frame only, not a motor pulse`
only after the write returns.

Run a dry-run only tiny forward pulse plan without transmitting anything:

    python scripts/send_c30d_tiny_forward_pulse_once.py

Execute one extremely constrained native tiny forward pulse sequence:

    python scripts/send_c30d_tiny_forward_pulse_once.py --armed --manual-enable --wheels-lifted --robot-restrained --manual-power-cutoff-ready --motor-enable-switch-reviewed --i-understand-this-may-spin-the-wheels --execute-real-pulse --feedback-output data/c30d_live/tiny_pulse_feedback.csv

Strong warning: this is the first possible motor movement script. Default behavior is
dry-run only and writes no bytes. A real pulse requires every listed guard flag and the
same first-write readiness gate before opening `/dev/c30d`. The script builds only native
host command frames using target Y and target Z fixed at zero. Defaults are
`--target-x 0.03` and `--duration 0.10`; hard limits reject `abs(target_x) > 0.05` or
duration above `0.15` seconds. The safe zero/stop frame is always the physically tested
`7b 00 00 00 00 00 00 00 00 7b 7d`; reserved/control-byte experiments apply only to
the pulse frame. The real sequence is fixed: safe zero frame, 0.05 second pause, pulse
frame, pulse duration pause, safe zero frame, 0.05 second pause, safe zero frame, then close.
During the same serial session it logs C30D feedback before, during, and after the pulse
when `--feedback-output` is provided. The CSV includes monotonic timestamp, phase,
forward/yaw candidates, candidate battery, checksum validity, and raw frame hex. The
script prints `pulse_reserved_1`, `pulse_reserved_2`, `safe_zero_frame_hex`,
`pulse_frame_hex`, baseline vs pulse/post forward-candidate maxima, max yaw candidate,
invalid checksum count, and whether movement feedback was detected. It accepts
`--reserved-1` and `--reserved-2` only as `0x00` or `0x01` for the pulse-frame-only
reserved/control-byte experiment. It accepts no steering
command, no target Y, no target Z, and no arbitrary packet input.

Steering commands, arbitrary driving, and larger/longer motor commands are still not
implemented or approved. Any broader motion test requires a separate explicit code change
and review, plus the physical safety checklist in the plan.

Summarize saved C30D feedback frame structure from passive `.bin` captures:

    python scripts/analyze_c30d_frame_structure.py data/c30d_captures/*.bin

This analyzer prints the observed feedback frame template, checksum rule, candidate
fields with per-capture min/max/mean stats, and unknown byte positions excluding bytes
already covered by candidate fields. Current candidate fields are
`candidate_forward_motion` from `int16_be_02_03`, `candidate_yaw_motion` from
`int16_be_06_07`, candidate IMU fields from `int16_be_12_13`, `int16_be_14_15`,
`int16_be_16_17`, and `int16_be_18_19`, and `candidate_battery_mV` from
`uint16_be_20_21`. It reads saved files only and does not infer the command protocol.

Search local files for C30D protocol references:

    python scripts/search_c30d_protocol_references.py

Run the read-only sensor preflight as part of the status report:

    python scripts/robot_safety_status.py --run-preflight --preflight-duration 3

Evaluate a future drive command through the dry-run safety gate:

    python scripts/dry_run_drive_command.py --speed-mps 0.1 --steering-deg 5 --duration 1 --manual-enable --wheels-lifted

Require a passed read-only preflight before accepting the dry-run command:

    python scripts/dry_run_drive_command.py --speed-mps 0.1 --steering-deg 5 --duration 1 --manual-enable --wheels-lifted --require-preflight --run-preflight

Try an unsafe dry-run command and see it rejected/clamped:

    python scripts/dry_run_drive_command.py --speed-mps 0.8 --steering-deg 5 --duration 1 --manual-enable --wheels-lifted

The command safety scaffold is configured by `config/command_safety.yaml`. Dry-run is the
default, manual enable and wheels-lifted confirmation are required for motor test
commands, `--require-preflight` can make read-only preflight a dry-run gate, and
`allow_serial_write` is false. When preflight is run, candidate battery warning/block
reasons are printed and a below-block candidate battery rejects preflight-required
dry-run command tests. C30D feedback decoding is partially understood as
read-only candidate fields from the integrated motor/servo/encoder/IMU controller, but
the real C30D motor/steering command protocol is unknown and not implemented. Because the
motors and steering servo are wired directly to the C30D, movement requires the C30D
command protocol unless the hardware is rewired. `dry_run_drive_command.py` builds only
a placeholder packet marked `UNIMPLEMENTED`; it does not contain real bytes and must not
be transmitted. `build_c30d_hypothesis_frame.py` can construct offline C30D-like frames
for research, but every output is an `unverified_hypothesis` with transmission disabled.
Real motor commands are blocked until official C30D command documentation or known-good
command examples are available. These scripts do not open `/dev/c30d` for commands, do
not write serial bytes, and do not move motors or steering.

The C30D protocol research scaffold lives in `docs/c30d_protocol_research.md`. Current
status: read-only feedback capture and candidate decoding work, the dry-run command path
works as a safety exercise, and real command transmission is blocked until documentation
or examples establish the actual C30D command packet format.

Capture a short finite RPLIDAR C1 scan sample with the SLAMTEC SDK backend:

    python scripts/rplidar_scan_sample.py --backend sdk --port /dev/rplidar --baud 460800 --duration 5

Capture with the Python wrapper backend for comparison:

    python scripts/rplidar_scan_sample.py --backend pyrplidar --port /dev/rplidar --baud 460800 --duration 5

RPLIDAR samples are saved as CSV files under `data/rplidar_tests/` with timestamp,
angle, distance, and quality columns. These raw sensor files will feed the custom
SLAM pipeline we build later. The SDK backend uses
`external/rplidar_sdk/output/Linux/Release/ultra_simple` for a bounded subprocess run
and saves raw stdout beside the CSV when parsing is incomplete.

Plot a recorded RPLIDAR scan without touching hardware:

    python scripts/plot_lidar_scan.py data/rplidar_tests/rplidar_scan_YYYYMMDD_HHMMSS.csv

Split a multi-revolution RPLIDAR capture into individual 360-degree scans:

    python scripts/split_lidar_scans.py data/rplidar_tests/rplidar_scan_YYYYMMDD_HHMMSS.csv

Save the split scans as separate CSV files:

    python scripts/split_lidar_scans.py data/rplidar_tests/rplidar_scan_YYYYMMDD_HHMMSS.csv --save-scans

Plot one segmented scan for inspection:

    python scripts/plot_single_lidar_scan.py data/rplidar_tests/rplidar_scan_YYYYMMDD_HHMMSS.csv --scan-index 0

Build a simple offline occupancy-grid PNG from a recorded RPLIDAR scan:

    python scripts/build_occupancy_grid_from_scan.py data/rplidar_tests/rplidar_scan_YYYYMMDD_HHMMSS.csv --width-m 8 --height-m 8 --resolution-m 0.05

Scan segmentation is needed before scan matching and SLAM because a several-second
capture can contain multiple lidar revolutions. Matching should operate on individual
360-degree scans with coherent timestamps, not one mixed cloud spanning multiple turns.

The occupancy-grid builder ray-traces from the single recorded sensor pose: cells along
each beam are marked free and hit endpoints are marked occupied. This is still a
single-pose/static scan map for offline inspection, not full SLAM with motion,
loop closure, or CKF/PF state estimation.

The first SLAM foundation lives in `ackermann_robot.slam`. It provides reusable recorded
scan dataclasses, a CSV loader for RPLIDAR captures, and a basic metric occupancy grid for
future CKF/PF state estimation and mapping work.

Capture passive C30D bytes without writing anything to the controller:

    python scripts/capture_c30d_passive.py --port /dev/c30d --baud 115200 --duration 5

Inspect a passive C30D capture without assigning protocol field meanings:

    python scripts/analyze_c30d_capture.py data/c30d_captures/c30d_capture_YYYYMMDD_HHMMSS.bin

Analyze the newest saved C30D capture:

    python scripts/analyze_c30d_capture.py --latest

The analyzer works only from saved `.bin` files. It reports byte counts, marker counts,
candidate frames between `0x7B` and `0x7D`, and frame length distribution without assigning
protocol field meanings.

Run the read-only candidate frame statistics layer on a saved C30D capture:

    python scripts/c30d_frame_stats.py data/c30d_captures/c30d_capture_YYYYMMDD_HHMMSS.bin

Analyze the newest saved C30D capture with frame statistics:

    python scripts/c30d_frame_stats.py --latest

The frame statistics command also works only from saved `.bin` files. It prints total
bytes, valid fixed-length frame count, rejected/resync count, frame length distribution,
the first 20 valid frames in hex, constant byte positions, changing byte positions, and
exact repeated frame patterns. Fixed-length C30D frame extraction uses `0x7B` at index 0,
24 total bytes, and `0x7D` at index 23 so payload bytes equal to `0x7D` do not truncate
frames. These are passive structure observations only; no byte is labeled as encoder,
IMU, or any other protocol field yet.

Analyze read-only checksum hypotheses for saved C30D captures:

    python scripts/analyze_c30d_checksum.py data/c30d_captures/*.bin

Read-only captures confirmed the C30D feedback checksum as XOR of bytes 0 through 21,
stored at byte 22. The feedback parser and CSV exporters report `checksum_valid` for
every fixed 24-byte frame, and read-only live monitors print invalid checksum counts and
warnings if any appear. The checksum analyzer remains useful for auditing saved captures.
It reads only saved `.bin` files and never writes to C30D.

Analyze candidate payload fields in checksum-valid feedback frames:

    python scripts/analyze_c30d_payload_fields.py data/c30d_captures/*.bin

The payload-field analyzer reads saved `.bin` files only. It filters to checksum-valid
fixed-length feedback frames, then reports uint16 big-endian bytes 20-21 stats, byte 20
and byte 21 unique values, checksum-valid/invalid counts, and
`candidate_battery_voltage_V = uint16_be_20_21 / 1000.0`. This is candidate-only
analysis, not confirmed battery voltage.

Compare multiple saved C30D captures against the first file as the baseline:

    python scripts/compare_c30d_captures.py data/c30d_captures/stationary.bin data/c30d_captures/motion.bin

Use a different input as the baseline and print more top-changing candidates:

    python scripts/compare_c30d_captures.py data/c30d_captures/*.bin --baseline-index 1 --top 12

The comparative analyzer is also offline and read-only. It uses the same fixed 24-byte
frame extractor, reports frame counts per file, byte-level min/max/mean/standard
deviation/unique-count summaries, byte positions that vary more than the baseline, and
adjacent payload-only signed int16 candidates labeled only as `candidate_int16_be_XX_YY`
or `candidate_int16_le_XX_YY`.

Plot read-only candidate int16 fields from one or more saved C30D captures:

    python scripts/plot_c30d_candidate_fields.py data/c30d_captures/stationary.bin data/c30d_captures/motion.bin

Plot selected adjacent payload byte pairs and only one byte order:

    python scripts/plot_c30d_candidate_fields.py data/c30d_captures/motion.bin --pairs 02_03 06_07 08_09 --endian le

Save plots to a custom analysis directory:

    python scripts/plot_c30d_candidate_fields.py data/c30d_captures/*.bin --output-dir data/c30d_analysis

Candidate field plots are saved as PNG files under `data/c30d_analysis/` by default.
The plotting tool uses adjacent signed int16 candidate pairs from `02_03` through
`18_19` in big-endian, little-endian, or both. Byte 22 is excluded because it is the
confirmed feedback checksum byte. Plot labels remain candidate names only and do not assign encoder,
IMU, steering, or other physical meanings.

Plot one candidate field across multiple captures on a single comparison graph:

    python scripts/compare_c30d_candidate_timeseries.py data/c30d_captures/stationary.bin data/c30d_captures/motion.bin --pair 02_03

Plot several selected candidates, one PNG per pair:

    python scripts/compare_c30d_candidate_timeseries.py data/c30d_captures/*.bin --pairs 02_03 06_07 08_09 --endian le

Use the aligned suggested candidate pairs without listing them:

    python scripts/compare_c30d_candidate_timeseries.py data/c30d_captures/*.bin --pairs

The focused comparison plotter overlays the same candidate field across captures instead
of overlaying many fields from one capture. Bare `--pairs` uses only aligned pairs
`02_03`, `06_07`, `08_09`, `10_11`, `12_13`, `14_15`, `16_17`, and `18_19`; overlapping
pairs such as `03_04`, `05_06`, and `07_08` are not included by default.

Export read-only candidate C30D feedback fields from a saved capture:

    python scripts/export_c30d_feedback_candidates.py data/c30d_captures/motion.bin

Write the CSV to a custom analysis directory:

    python scripts/export_c30d_feedback_candidates.py data/c30d_captures/motion.bin --output-dir data/c30d_analysis

The feedback exporter is offline and read-only. It extracts the same fixed 24-byte C30D
frames and writes CSV rows under `data/c30d_analysis/` by default. Decoded columns remain
candidate names only: `candidate_forward_motion` from bytes `02_03`,
`candidate_yaw_motion` from bytes `06_07`, candidate IMU/motion fields from `12_13`,
`14_15`, `16_17`, and `18_19`, `candidate_battery_mV` from bytes `20_21`, plus
`checksum_candidate` and `checksum_valid`.
These names reflect current hand-spin observations and are not confirmed protocol fields.

Monitor live C30D feedback candidates without writing to the controller:

    python scripts/monitor_c30d_feedback_readonly.py --duration 5

Use a different serial port or print every parsed frame:

    python scripts/monitor_c30d_feedback_readonly.py --port /dev/c30d --baud 115200 --duration 10 --print-every 1

Save decoded live candidate rows under `data/c30d_live/`:

    python scripts/monitor_c30d_feedback_readonly.py --duration 5 --output c30d_live_feedback.csv

The live monitor is explicitly read-only. It opens the C30D serial port with a read-only
file descriptor, extracts fixed 24-byte frames with `0x7B` at byte 0 and `0x7D` at byte
23, decodes the same candidate fields as the offline exporter, reports invalid checksum
counts, and never writes bytes or sends motor/steering commands. Live field names remain
candidate labels only.

Monitor provisional live C30D odometry without writing to the controller:

    python scripts/monitor_c30d_odometry_readonly.py --duration 5 --mode straight_only

Log raw yaw candidate counts while keeping `theta_rad` fixed:

    python scripts/monitor_c30d_odometry_readonly.py --duration 5 --mode raw_yaw_candidate --print-every 1

Save provisional live odometry rows under `data/c30d_live/`:

    python scripts/monitor_c30d_odometry_readonly.py --duration 5 --output c30d_live_odometry.csv

The live odometry monitor is read-only and provisional. It opens the C30D serial port
with a read-only file descriptor, loads `config/c30d_calibration.yaml`, converts live
`candidate_forward_motion` counts to `delta_s_m` with `forward_m_per_count`, and prints
compact `x_m`, `y_m`, and `theta_rad` lines. Because yaw calibration is unavailable,
both modes log raw `candidate_yaw_motion` counts without converting them to radians.
`straight_only` disables yaw integration, so `theta_rad` remains fixed at zero. The script
never writes bytes to the controller or sends motor/steering commands.

Summarize one or more exported feedback candidate CSV files:

    python scripts/summarize_c30d_feedback_candidates.py data/c30d_analysis/motion_feedback_candidates.csv

Estimate sample rate from a known capture duration:

    python scripts/summarize_c30d_feedback_candidates.py data/c30d_analysis/motion_feedback_candidates.csv --duration-s 5

Compute candidate calibration helpers from a known distance or yaw motion:

    python scripts/summarize_c30d_feedback_candidates.py data/c30d_analysis/forward_feedback_candidates.csv --known-distance-m 1.0
    python scripts/summarize_c30d_feedback_candidates.py data/c30d_analysis/yaw_feedback_candidates.csv --known-yaw-deg 90

The summary helper reads only exported CSV files and uses the standard library CSV parser.
For each candidate motion field it prints min, max, mean, standard deviation, sum,
absolute sum, and nonzero count. Optional calibration outputs are
`meters_per_forward_sum` and `radians_per_yaw_sum`; they are helper ratios for candidate
field analysis only, not confirmed controller units.

Replay provisional offline C30D dead-reckoning odometry from an exported feedback CSV:

    python scripts/replay_c30d_odometry.py data/c30d_analysis/motion_feedback_candidates.csv --mode straight_only

Preserve raw yaw candidate counts for later calibration analysis:

    python scripts/replay_c30d_odometry.py data/c30d_analysis/motion_feedback_candidates.csv --mode raw_yaw_candidate

Use a different provisional calibration file or output directory:

    python scripts/replay_c30d_odometry.py data/c30d_analysis/motion_feedback_candidates.csv --config config/c30d_calibration.yaml --output-dir data/c30d_analysis

The odometry replay is offline, read-only, and provisional. It reads exported feedback
candidate CSV files, loads `config/c30d_calibration.yaml`, converts
`candidate_forward_motion` to `delta_s_m` with `forward_m_per_count`, and writes odometry
CSV files under `data/c30d_analysis/` by default. Because `yaw_rad_per_count` is still
null, `straight_only` assumes zero yaw change and `raw_yaw_candidate` preserves raw
`candidate_yaw_motion` counts without converting them to radians. `theta_rad` remains
unchanged until yaw calibration is available.

Plot one or more provisional C30D odometry CSV files:

    python scripts/plot_c30d_odometry.py data/c30d_analysis/motion_feedback_candidates_odometry_straight_only.csv

Overlay multiple odometry replays:

    python scripts/plot_c30d_odometry.py data/c30d_analysis/*_odometry_straight_only.csv

The odometry plotter reads only odometry CSV files and uses the standard library CSV
parser for loading. It saves `c30d_odometry_xy.png` for `x_m` versus `y_m` and
`c30d_odometry_x_over_frame.png` for `x_m` over `frame_index` under
`data/c30d_analysis/` by default. It also prints the final `x_m`, `y_m`, and `theta_rad`
for each input file. These plots remain provisional until the C30D field calibration is
confirmed.

Record a unified read-only sensor run:

    python scripts/record_readonly_sensor_run.py --duration 5 --enable-c30d

Record C30D feedback, RPLIDAR, and one OAK-D Lite RGB frame into the same run folder:

    python scripts/record_readonly_sensor_run.py --duration 5 --enable-c30d --enable-rplidar --enable-oak

Use a custom output root:

    python scripts/record_readonly_sensor_run.py --duration 5 --enable-c30d --output-root data/runs

The unified logger creates `data/runs/run_YYYYMMDD_HHMMSS/` by default and writes
`metadata.yaml` with start time, duration, enabled sensors, C30D port/baud, RPLIDAR
port/baud, and OAK capture settings. Enabled sensor outputs are `c30d_feedback.csv`,
provisional straight-only `c30d_odometry.csv`, `rplidar_scan.csv`, and
`oak_rgb/oak_rgb_0000.jpg`. C30D is opened read-only and the logger never sends motor or
steering commands. RPLIDAR rows keep the standard `timestamp_s`, `angle_deg`,
`distance_mm`, and `quality` columns; SDK captures assign point timestamps across the
host capture window instead of stamping the whole file with one time.

Validate and summarize a saved unified read-only sensor run without touching hardware:

    python scripts/validate_readonly_sensor_run.py data/runs/run_YYYYMMDD_HHMMSS

The validator reads only `metadata.yaml`, CSV files, and `oak_rgb/` images from the saved
run folder. It prints row counts, CSV columns, first/last timestamps or frame indexes when
available, final C30D odometry, RPLIDAR point and distance summaries, and OAK RGB image
file sizes. For RPLIDAR CSVs it also prints timestamp duration, warns when multiple rows
share one constant timestamp, and reports zero and nonpositive `distance_mm` points. It
exits nonzero only when data is missing for a sensor that was enabled in `metadata.yaml`.

Replay and summarize a saved unified read-only sensor run without touching hardware:

    python scripts/replay_readonly_sensor_run.py data/runs/run_YYYYMMDD_HHMMSS

The replay tool validates the saved run folder, writes outputs under
`replay_outputs/`, plots C30D odometry when `c30d_odometry.csv` exists, plots RPLIDAR XY
points, builds a ray-traced occupancy grid from valid `distance_mm > 0` lidar points, and
prints OAK RGB image paths. Its summary includes enabled sensors, C30D row count, final
odometry, lidar total/valid/zero-distance counts, and OAK image count.

## Logs

Use `setup_logging()` from `ackermann_robot.utils.logging_utils` to create a timestamped
run folder under `logs/`, such as `logs/run_YYYYMMDD_HHMMSS/`. Each run folder contains
`robot.log` for structured supervisor logs.

## Notes

This project is intentionally native and non-ROS2. The Raspberry Pi 5 handles low-level supervision, safety, hardware drivers, logging, and communication with the Jetson. The Jetson handles heavier compute such as vision, mapping, 3D reconstruction, and learned navigation.
