#!/usr/bin/env python3

import argparse
import csv
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

DEFAULT_PORT = "/dev/rplidar"
DEFAULT_BAUD = 460800
DEFAULT_DURATION_S = 5.0
DEFAULT_OUTPUT_DIR = Path("data/rplidar_tests")
CSV_COLUMNS = ["timestamp_s", "angle_deg", "distance_mm", "quality"]


@dataclass(frozen=True)
class ScanPoint:
    timestamp_s: float
    angle_deg: float
    distance_mm: float
    quality: int | None = None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Capture a finite RPLIDAR scan sample to CSV.")
    parser.add_argument("--port", default=DEFAULT_PORT, help="RPLIDAR serial device path.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="RPLIDAR serial baud rate.")
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_S,
        help="Scan capture duration in seconds.",
    )
    parser.add_argument("--output", type=Path, default=None, help="Output CSV path.")
    return parser


def default_output_path(now: datetime | None = None) -> Path:
    timestamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
    return DEFAULT_OUTPUT_DIR / f"rplidar_scan_{timestamp}.csv"


def csv_row(point: ScanPoint) -> dict[str, str]:
    return {
        "timestamp_s": f"{point.timestamp_s:.6f}",
        "angle_deg": f"{point.angle_deg:.3f}",
        "distance_mm": f"{point.distance_mm:.3f}",
        "quality": "" if point.quality is None else str(point.quality),
    }


def measurement_to_point(measurement: object, timestamp_s: float) -> ScanPoint:
    return ScanPoint(
        timestamp_s=timestamp_s,
        angle_deg=float(getattr(measurement, "angle")),
        distance_mm=float(getattr(measurement, "distance")),
        quality=getattr(measurement, "quality", None),
    )


def summarize_points(points: list[ScanPoint]) -> dict[str, float | int | None]:
    if not points:
        return {
            "count": 0,
            "min_angle": None,
            "max_angle": None,
            "min_distance": None,
            "max_distance": None,
        }
    angles = [point.angle_deg for point in points]
    distances = [point.distance_mm for point in points]
    return {
        "count": len(points),
        "min_angle": min(angles),
        "max_angle": max(angles),
        "min_distance": min(distances),
        "max_distance": max(distances),
    }


def capture_scan(port: str, baud: int, duration_s: float, output_path: Path) -> list[ScanPoint]:
    from pyrplidar import PyRPlidar

    output_path.parent.mkdir(parents=True, exist_ok=True)
    lidar = PyRPlidar()
    points: list[ScanPoint] = []

    try:
        lidar.connect(port=port, baudrate=baud, timeout=1)
        scan_generator = lidar.start_scan()
        deadline_s = time.monotonic() + duration_s

        with output_path.open("w", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=CSV_COLUMNS)
            writer.writeheader()
            for measurement in scan_generator():
                now_s = time.time()
                point = measurement_to_point(measurement, now_s)
                points.append(point)
                writer.writerow(csv_row(point))
                if time.monotonic() >= deadline_s:
                    break
    finally:
        try:
            lidar.stop()
        except Exception as exc:
            print(f"Warning: failed to send RPLIDAR stop command: {exc}", file=sys.stderr)
        try:
            lidar.disconnect()
        except Exception as exc:
            print(f"Warning: failed to disconnect RPLIDAR: {exc}", file=sys.stderr)

    return points


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.duration < 0:
        print("--duration must be non-negative.", file=sys.stderr)
        return 2

    output_path = args.output or default_output_path()
    print("RPLIDAR sensor-only scan sample: this script does not access or command the C30D.")

    try:
        points = capture_scan(args.port, args.baud, args.duration, output_path)
    except Exception as exc:
        print(f"Failed to capture RPLIDAR scan from {args.port} at {args.baud}: {exc}", file=sys.stderr)
        return 1

    summary = summarize_points(points)
    print(f"points: {summary['count']}")
    print(f"min_angle_deg: {summary['min_angle']}")
    print(f"max_angle_deg: {summary['max_angle']}")
    print(f"min_distance_mm: {summary['min_distance']}")
    print(f"max_distance_mm: {summary['max_distance']}")
    print(f"output: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
