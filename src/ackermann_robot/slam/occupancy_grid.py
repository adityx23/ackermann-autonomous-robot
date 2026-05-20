from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from ackermann_robot.slam.lidar_types import LidarScan

UNKNOWN = -1
FREE = 0
OCCUPIED = 100


@dataclass
class OccupancyGrid:
    width: int
    height: int
    resolution_m: float
    origin_x_m: float = 0.0
    origin_y_m: float = 0.0
    data: NDArray[np.int8] = field(init=False)

    def __post_init__(self) -> None:
        if self.width <= 0:
            raise ValueError("width must be positive")
        if self.height <= 0:
            raise ValueError("height must be positive")
        if self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")
        self.data = np.full((self.height, self.width), UNKNOWN, dtype=np.int8)

    def world_to_grid(self, x_m: float, y_m: float) -> tuple[int, int] | None:
        grid_x = int((x_m - self.origin_x_m) // self.resolution_m)
        grid_y = int((y_m - self.origin_y_m) // self.resolution_m)
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            return grid_x, grid_y
        return None

    def grid_to_world(self, grid_x: int, grid_y: int) -> tuple[float, float]:
        if not (0 <= grid_x < self.width and 0 <= grid_y < self.height):
            raise ValueError(f"grid cell out of bounds: ({grid_x}, {grid_y})")
        x_m = self.origin_x_m + (grid_x + 0.5) * self.resolution_m
        y_m = self.origin_y_m + (grid_y + 0.5) * self.resolution_m
        return x_m, y_m

    def mark_occupied(self, x_m: float, y_m: float) -> bool:
        cell = self.world_to_grid(x_m, y_m)
        if cell is None:
            return False
        grid_x, grid_y = cell
        self.data[grid_y, grid_x] = OCCUPIED
        return True

    def mark_lidar_points(self, scan: LidarScan) -> int:
        marked = 0
        for x_m, y_m in scan.to_xy_m():
            if self.mark_occupied(x_m, y_m):
                marked += 1
        return marked
