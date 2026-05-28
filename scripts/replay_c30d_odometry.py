#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

DEFAULT_CONFIG = Path("config/c30d_calibration.yaml")
DEFAULT_OUTPUT_DIR = Path("data/c30d_analysis")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Replay provisional offline C30D dead-reckoning odometry from a candidate CSV."
    )
    parser.add_argument("input_csv", type=Path, help="Exported C30D feedback candidate CSV file.")
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help="Path to provisional C30D calibration YAML.",
    )
    parser.add_argument(
        "--mode",
        choices=("straight_only", "raw_yaw_candidate"),
        default="straight_only",
        help="Yaw handling mode. Yaw is not converted to radians while calibration is null.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Directory for the odometry replay CSV.",
    )
    return parser


def replay_to_csv(
    input_csv: Path,
    config_path: Path,
    mode: str,
    output_dir: Path,
) -> tuple[Path, int]:
    from ackermann_robot.odometry.c30d_dead_reckoning import (
        load_c30d_calibration,
        load_feedback_candidate_csv,
        output_path_for,
        replay_dead_reckoning,
        write_odometry_csv,
    )

    calibration = load_c30d_calibration(config_path)
    rows = load_feedback_candidate_csv(input_csv)
    samples = replay_dead_reckoning(rows, calibration, mode)
    output_path = output_path_for(input_csv, output_dir, mode)
    write_odometry_csv(samples, output_path)
    return output_path, len(samples)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        output_path, row_count = replay_to_csv(
            args.input_csv,
            args.config,
            args.mode,
            args.output_dir,
        )
    except (OSError, ValueError) as exc:
        print(f"failed to replay provisional C30D odometry: {exc}", file=sys.stderr)
        return 1

    print("Provisional read-only C30D dead-reckoning replay from exported CSV only.")
    print("yaw_calibration: unavailable; theta_rad remains unchanged")
    print(f"mode: {args.mode}")
    print(f"output_path: {output_path}")
    print(f"row_count: {row_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
