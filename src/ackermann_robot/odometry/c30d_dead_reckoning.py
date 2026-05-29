from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, TypedDict

import yaml

DeadReckoningMode = Literal["straight_only", "raw_yaw_candidate"]
ODOMETRY_FIELDNAMES = [
    "frame_index",
    "delta_s_m",
    "yaw_candidate",
    "x_m",
    "y_m",
    "theta_rad",
]


class FeedbackCandidateRow(TypedDict):
    frame_index: int
    candidate_forward_motion: int
    candidate_yaw_motion: int


@dataclass(frozen=True)
class C30DCalibration:
    forward_m_per_count: float
    yaw_rad_per_count: float | None
    sample_rate_hz: float | None
    provisional: bool = True


@dataclass(frozen=True)
class C30DOdometrySample:
    frame_index: int
    delta_s_m: float
    yaw_candidate: int
    x_m: float
    y_m: float
    theta_rad: float


def load_c30d_calibration(config_path: str | Path) -> C30DCalibration:
    path = Path(config_path)
    loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict) or not isinstance(loaded.get("c30d"), dict):
        raise ValueError(f"{path} must contain a c30d mapping")

    c30d = loaded["c30d"]
    forward_m_per_count = c30d.get("forward_m_per_count")
    if not isinstance(forward_m_per_count, int | float):
        raise ValueError(f"{path} missing numeric c30d.forward_m_per_count")

    yaw_rad_per_count = c30d.get("yaw_rad_per_count")
    if yaw_rad_per_count is not None and not isinstance(yaw_rad_per_count, int | float):
        raise ValueError(f"{path} c30d.yaw_rad_per_count must be numeric or null")

    sample_rate_hz = c30d.get("sample_rate_hz")
    if sample_rate_hz is not None and not isinstance(sample_rate_hz, int | float):
        raise ValueError(f"{path} c30d.sample_rate_hz must be numeric or null")

    return C30DCalibration(
        forward_m_per_count=float(forward_m_per_count),
        yaw_rad_per_count=None if yaw_rad_per_count is None else float(yaw_rad_per_count),
        sample_rate_hz=None if sample_rate_hz is None else float(sample_rate_hz),
    )


def load_feedback_candidate_csv(csv_path: str | Path) -> list[FeedbackCandidateRow]:
    path = Path(csv_path)
    rows: list[FeedbackCandidateRow] = []
    with path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            raise ValueError(f"{path} is missing a CSV header")

        required_fields = [
            "frame_index",
            "candidate_forward_motion",
            "candidate_yaw_motion",
        ]
        missing_fields = [field for field in required_fields if field not in reader.fieldnames]
        if missing_fields:
            raise ValueError(f"{path} is missing required columns: {', '.join(missing_fields)}")

        for row_number, row in enumerate(reader, start=2):
            try:
                rows.append(
                    {
                        "frame_index": int(row["frame_index"]),
                        "candidate_forward_motion": int(row["candidate_forward_motion"]),
                        "candidate_yaw_motion": int(row["candidate_yaw_motion"]),
                    }
                )
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{path}:{row_number} has invalid integer candidate data") from exc
    return rows


def replay_dead_reckoning(
    rows: list[FeedbackCandidateRow],
    calibration: C30DCalibration,
    mode: DeadReckoningMode,
) -> list[C30DOdometrySample]:
    if mode not in ("straight_only", "raw_yaw_candidate"):
        raise ValueError(f"unsupported C30D dead-reckoning mode: {mode}")
    if calibration.yaw_rad_per_count is not None:
        raise ValueError(
            "calibrated C30D yaw odometry is not implemented for this provisional helper"
        )

    x_m = 0.0
    y_m = 0.0
    theta_rad = 0.0
    samples: list[C30DOdometrySample] = []

    for row in rows:
        delta_s_m = row["candidate_forward_motion"] * calibration.forward_m_per_count
        yaw_candidate = row["candidate_yaw_motion"] if mode == "raw_yaw_candidate" else 0

        x_m += delta_s_m * math.cos(theta_rad)
        y_m += delta_s_m * math.sin(theta_rad)

        samples.append(
            C30DOdometrySample(
                frame_index=row["frame_index"],
                delta_s_m=delta_s_m,
                yaw_candidate=yaw_candidate,
                x_m=x_m,
                y_m=y_m,
                theta_rad=theta_rad,
            )
        )

    return samples


def output_path_for(input_csv: str | Path, output_dir: str | Path, mode: DeadReckoningMode) -> Path:
    return Path(output_dir) / f"{Path(input_csv).stem}_odometry_{mode}.csv"


def write_odometry_csv(samples: list[C30DOdometrySample], output_path: str | Path) -> Path:
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=ODOMETRY_FIELDNAMES)
        writer.writeheader()
        for sample in samples:
            writer.writerow(
                {
                    "frame_index": sample.frame_index,
                    "delta_s_m": f"{sample.delta_s_m:.12g}",
                    "yaw_candidate": sample.yaw_candidate,
                    "x_m": f"{sample.x_m:.12g}",
                    "y_m": f"{sample.y_m:.12g}",
                    "theta_rad": f"{sample.theta_rad:.12g}",
                }
            )
    return path
