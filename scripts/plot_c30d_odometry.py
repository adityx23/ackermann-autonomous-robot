#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path

DEFAULT_OUTPUT_DIR = Path("data/c30d_analysis")
XY_PLOT_NAME = "c30d_odometry_xy.png"
X_OVER_FRAME_PLOT_NAME = "c30d_odometry_x_over_frame.png"


@dataclass(frozen=True)
class OdometrySample:
    frame_index: int
    x_m: float
    y_m: float
    theta_rad: float


@dataclass(frozen=True)
class OdometrySeries:
    csv_path: Path
    samples: list[OdometrySample]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plot provisional offline C30D odometry CSV files."
    )
    parser.add_argument(
        "odometry_csvs",
        nargs="+",
        type=Path,
        help="Odometry CSV files produced by replay_c30d_odometry.py.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Directory for saved PNG plots.",
    )
    return parser


def load_odometry_csv(csv_path: Path) -> OdometrySeries:
    samples: list[OdometrySample] = []
    with csv_path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            raise ValueError(f"{csv_path} is missing a CSV header")

        required_fields = ["frame_index", "x_m", "y_m", "theta_rad"]
        missing_fields = [field for field in required_fields if field not in reader.fieldnames]
        if missing_fields:
            raise ValueError(f"{csv_path} is missing required columns: {', '.join(missing_fields)}")

        for row_number, row in enumerate(reader, start=2):
            try:
                samples.append(
                    OdometrySample(
                        frame_index=int(row["frame_index"]),
                        x_m=float(row["x_m"]),
                        y_m=float(row["y_m"]),
                        theta_rad=float(row["theta_rad"]),
                    )
                )
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{csv_path}:{row_number} has invalid odometry data") from exc
    return OdometrySeries(csv_path=csv_path, samples=samples)


def final_pose(series: OdometrySeries) -> tuple[float, float, float]:
    if not series.samples:
        return math.nan, math.nan, math.nan
    sample = series.samples[-1]
    return sample.x_m, sample.y_m, sample.theta_rad


def output_paths(output_dir: Path) -> tuple[Path, Path]:
    return output_dir / XY_PLOT_NAME, output_dir / X_OVER_FRAME_PLOT_NAME


def save_xy_plot(series_list: list[OdometrySeries], output_path: Path) -> Path:
    import matplotlib.pyplot as plt

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8, 7))
    for series in series_list:
        ax.plot(
            [sample.x_m for sample in series.samples],
            [sample.y_m for sample in series.samples],
            linewidth=1.4,
            label=series.csv_path.stem,
        )

    ax.set_title("Provisional C30D odometry x/y")
    ax.set_xlabel("x_m")
    ax.set_ylabel("y_m")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    if series_list:
        ax.legend(loc="best", fontsize="small")
    fig.tight_layout()
    fig.savefig(output_path, dpi=140)
    plt.close(fig)
    return output_path


def save_x_over_frame_plot(series_list: list[OdometrySeries], output_path: Path) -> Path:
    import matplotlib.pyplot as plt

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(10, 5))
    for series in series_list:
        ax.plot(
            [sample.frame_index for sample in series.samples],
            [sample.x_m for sample in series.samples],
            linewidth=1.2,
            label=series.csv_path.stem,
        )

    ax.set_title("Provisional C30D odometry x over frame")
    ax.set_xlabel("frame_index")
    ax.set_ylabel("x_m")
    ax.grid(True, alpha=0.3)
    if series_list:
        ax.legend(loc="best", fontsize="small")
    fig.tight_layout()
    fig.savefig(output_path, dpi=140)
    plt.close(fig)
    return output_path


def save_plots(series_list: list[OdometrySeries], output_dir: Path) -> tuple[Path, Path]:
    xy_path, x_over_frame_path = output_paths(output_dir)
    return (
        save_xy_plot(series_list, xy_path),
        save_x_over_frame_plot(series_list, x_over_frame_path),
    )


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        series_list = [load_odometry_csv(csv_path) for csv_path in args.odometry_csvs]
        xy_path, x_over_frame_path = save_plots(series_list, args.output_dir)
    except (OSError, ValueError) as exc:
        print(f"failed to plot provisional C30D odometry: {exc}", file=sys.stderr)
        return 1

    print("Provisional read-only C30D odometry plots from odometry CSV files only.")
    print(f"xy_plot_path: {xy_path}")
    print(f"x_over_frame_plot_path: {x_over_frame_path}")
    for series in series_list:
        x_m, y_m, theta_rad = final_pose(series)
        print(
            f"{series.csv_path}: final_x_m={x_m:.6g} "
            f"final_y_m={y_m:.6g} final_theta_rad={theta_rad:.6g}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
