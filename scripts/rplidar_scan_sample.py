#!/usr/bin/env python3

import argparse
import csv
import signal
import subprocess
import sys
import time
import traceback
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import re

DEFAULT_PORT = "/dev/rplidar"
DEFAULT_BAUD = 460800
DEFAULT_DURATION_S = 5.0
DEFAULT_OUTPUT_DIR = Path("data/rplidar_tests")
DEFAULT_SDK_BINARY = Path("external/rplidar_sdk/output/Linux/Release/ultra_simple")
CSV_COLUMNS = ["timestamp_s", "angle_deg", "distance_mm", "quality"]
SDK_SCAN_RE = re.compile(
    r"theta:\s*(?P<angle>[+-]?\d+(?:\.\d+)?)\s+"
    r"Dist:\s*(?P<distance>[+-]?\d+(?:\.\d+)?)\s+"
    r"Q:\s*(?P<quality>\d+)",
    re.IGNORECASE,
)


@dataclass(frozen=True)
class ScanPoint:
    timestamp_s: float
    angle_deg: float
    distance_mm: float
    quality: int | None = None


@dataclass(frozen=True)
class CaptureResult:
    points: list[ScanPoint]
    output_path: Path
    raw_log_path: Path | None = None


class CaptureError(RuntimeError):
    def __init__(self, message: str, raw_log_path: Path | None = None) -> None:
        super().__init__(message)
        self.raw_log_path = raw_log_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Capture a finite RPLIDAR scan sample to CSV.")
    parser.add_argument("--port", default=DEFAULT_PORT, help="RPLIDAR serial device path.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="RPLIDAR serial baud rate.")
    parser.add_argument(
        "--backend",
        choices=("sdk", "pyrplidar"),
        default="sdk",
        help="Capture backend to use.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_S,
        help="Scan capture duration in seconds.",
    )
    parser.add_argument("--output", type=Path, default=None, help="Output CSV path.")
    parser.add_argument(
        "--sdk-binary",
        type=Path,
        default=DEFAULT_SDK_BINARY,
        help="Path to the SLAMTEC SDK ultra_simple binary.",
    )
    parser.add_argument(
        "--verbose", action="store_true", help="Print tracebacks for capture failures."
    )
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


def parse_sdk_scan_line(line: str, timestamp_s: float) -> ScanPoint | None:
    match = SDK_SCAN_RE.search(line)
    if match is None:
        return None
    return ScanPoint(
        timestamp_s=timestamp_s,
        angle_deg=float(match.group("angle")),
        distance_mm=float(match.group("distance")),
        quality=int(match.group("quality")),
    )


def timestamp_for_index(
    index: int, count: int, start_timestamp_s: float, end_timestamp_s: float
) -> float:
    if count <= 1:
        return start_timestamp_s
    fraction = index / (count - 1)
    return start_timestamp_s + fraction * (end_timestamp_s - start_timestamp_s)


def parse_sdk_output(
    stdout: str,
    timestamp_s: float | None = None,
    start_timestamp_s: float | None = None,
    end_timestamp_s: float | None = None,
) -> tuple[list[ScanPoint], list[str]]:
    points: list[ScanPoint] = []
    unparsed_scan_lines: list[str] = []
    scan_lines = [
        line
        for line in stdout.splitlines()
        if SDK_SCAN_RE.search(line) is not None or "theta:" in line or "Dist:" in line
    ]
    if timestamp_s is not None:
        start_time_s = timestamp_s
        end_time_s = timestamp_s
    else:
        start_time_s = time.time() if start_timestamp_s is None else start_timestamp_s
        end_time_s = start_time_s if end_timestamp_s is None else end_timestamp_s

    scan_line_index = 0
    for line in stdout.splitlines():
        is_scan_like = SDK_SCAN_RE.search(line) is not None or "theta:" in line or "Dist:" in line
        sample_time_s = (
            timestamp_for_index(scan_line_index, len(scan_lines), start_time_s, end_time_s)
            if is_scan_like
            else start_time_s
        )
        point = parse_sdk_scan_line(line, sample_time_s)
        if point is not None:
            points.append(point)
        elif "theta:" in line or "Dist:" in line:
            unparsed_scan_lines.append(line)
        if is_scan_like:
            scan_line_index += 1

    return points, unparsed_scan_lines


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


def write_points_csv(points: list[ScanPoint], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        for point in points:
            writer.writerow(csv_row(point))


def raw_log_path_for(output_path: Path) -> Path:
    return output_path.with_suffix(".raw.log")


def capture_scan_with_sdk(
    port: str,
    baud: int,
    duration_s: float,
    output_path: Path,
    sdk_binary: Path = DEFAULT_SDK_BINARY,
) -> CaptureResult:
    if not sdk_binary.exists():
        raise FileNotFoundError(f"SLAMTEC SDK binary not found: {sdk_binary}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    command = [str(sdk_binary), "--channel", "--serial", port, str(baud)]
    start_timestamp_s = time.time()
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    try:
        stdout, stderr = process.communicate(timeout=duration_s)
    except subprocess.TimeoutExpired:
        process.send_signal(signal.SIGINT)
        try:
            stdout, stderr = process.communicate(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                stdout, stderr = process.communicate(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                stdout, stderr = process.communicate()
    end_timestamp_s = time.time()

    points, unparsed_scan_lines = parse_sdk_output(
        stdout,
        start_timestamp_s=start_timestamp_s,
        end_timestamp_s=end_timestamp_s,
    )
    write_points_csv(points, output_path)

    raw_log_path = None
    if not points or unparsed_scan_lines or process.returncode not in (0, -signal.SIGINT):
        raw_log_path = raw_log_path_for(output_path)
        raw_log_path.write_text(stdout, encoding="utf-8")

    if process.returncode not in (0, -signal.SIGINT) and not points:
        message = (
            stderr.strip() or stdout.strip() or f"ultra_simple exited with {process.returncode}"
        )
        raise CaptureError(message, raw_log_path)

    return CaptureResult(points=points, output_path=output_path, raw_log_path=raw_log_path)


def capture_scan_with_pyrplidar(
    port: str, baud: int, duration_s: float, output_path: Path
) -> CaptureResult:
    from pyrplidar import PyRPlidar

    output_path.parent.mkdir(parents=True, exist_ok=True)
    lidar = PyRPlidar()
    points: list[ScanPoint] = []

    try:
        lidar.connect(port=port, baudrate=baud, timeout=1)
        scan_generator = lidar.start_scan()
        deadline_s = time.monotonic() + duration_s

        measurements = scan_generator() if callable(scan_generator) else scan_generator
        with output_path.open("w", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=CSV_COLUMNS)
            writer.writeheader()
            for measurement in measurements:
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
            print(f"Warning: failed to send RPLIDAR stop command: {exc!r}", file=sys.stderr)
        try:
            lidar.disconnect()
        except Exception as exc:
            print(f"Warning: failed to disconnect RPLIDAR: {exc!r}", file=sys.stderr)

    return CaptureResult(points=points, output_path=output_path)


def capture_scan(
    backend: str,
    port: str,
    baud: int,
    duration_s: float,
    output_path: Path,
    sdk_binary: Path = DEFAULT_SDK_BINARY,
) -> CaptureResult:
    if backend == "sdk":
        return capture_scan_with_sdk(port, baud, duration_s, output_path, sdk_binary)
    if backend == "pyrplidar":
        return capture_scan_with_pyrplidar(port, baud, duration_s, output_path)
    raise ValueError(f"unsupported RPLIDAR backend: {backend}")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.duration < 0:
        print("--duration must be non-negative.", file=sys.stderr)
        return 2

    output_path = args.output or default_output_path()
    print("RPLIDAR sensor-only scan sample: this script does not access or command the C30D.")
    print(f"backend: {args.backend}")

    try:
        result = capture_scan(
            args.backend, args.port, args.baud, args.duration, output_path, args.sdk_binary
        )
    except Exception as exc:
        print(
            f"Failed to capture RPLIDAR scan from {args.port} at {args.baud}: {exc!r}",
            file=sys.stderr,
        )
        raw_log_path = getattr(exc, "raw_log_path", None)
        if raw_log_path is not None:
            print(f"raw_log: {raw_log_path}", file=sys.stderr)
        if args.verbose:
            traceback.print_exc()
        return 1

    summary = summarize_points(result.points)
    print(f"points: {summary['count']}")
    print(f"min_angle_deg: {summary['min_angle']}")
    print(f"max_angle_deg: {summary['max_angle']}")
    print(f"min_distance_mm: {summary['min_distance']}")
    print(f"max_distance_mm: {summary['max_distance']}")
    print(f"output: {result.output_path}")
    if result.raw_log_path is not None:
        print(f"raw_log: {result.raw_log_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
