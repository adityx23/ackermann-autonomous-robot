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
`18_19` in big-endian, little-endian, or both. Byte 22 is excluded because it is only a
checksum candidate. Plot labels remain candidate names only and do not assign encoder,
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

## Logs

Use `setup_logging()` from `ackermann_robot.utils.logging_utils` to create a timestamped
run folder under `logs/`, such as `logs/run_YYYYMMDD_HHMMSS/`. Each run folder contains
`robot.log` for structured supervisor logs.

## Notes

This project is intentionally native and non-ROS2. The Raspberry Pi 5 handles low-level supervision, safety, hardware drivers, logging, and communication with the Jetson. The Jetson handles heavier compute such as vision, mapping, 3D reconstruction, and learned navigation.
