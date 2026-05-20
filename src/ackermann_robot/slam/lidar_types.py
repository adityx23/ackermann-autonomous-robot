from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class LidarPoint:
    timestamp_s: float
    angle_deg: float
    distance_mm: float
    quality: int | None = None
    scan_id: int | None = None

    def is_valid(self) -> bool:
        return (
            math.isfinite(self.timestamp_s)
            and math.isfinite(self.angle_deg)
            and math.isfinite(self.distance_mm)
            and self.distance_mm > 0.0
        )

    def to_xy_m(self) -> tuple[float, float]:
        angle_rad = math.radians(self.angle_deg)
        distance_m = self.distance_mm / 1000.0
        return distance_m * math.cos(angle_rad), distance_m * math.sin(angle_rad)


@dataclass(frozen=True)
class LidarScan:
    points: list[LidarPoint]
    start_time_s: float
    end_time_s: float

    def valid_points(self) -> list[LidarPoint]:
        return [point for point in self.points if point.is_valid()]

    def to_xy_m(self) -> list[tuple[float, float]]:
        return [point.to_xy_m() for point in self.valid_points()]


@dataclass(frozen=True)
class LidarScanSequence:
    scans: list[LidarScan]

    def total_points(self) -> int:
        return sum(len(scan.points) for scan in self.scans)
