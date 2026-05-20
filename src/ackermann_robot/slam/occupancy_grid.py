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

    def mark_free(self, x_m: float, y_m: float) -> bool:
        cell = self.world_to_grid(x_m, y_m)
        if cell is None:
            return False
        grid_x, grid_y = cell
        if self.data[grid_y, grid_x] != OCCUPIED:
            self.data[grid_y, grid_x] = FREE
        return True

    def mark_lidar_points(self, scan: LidarScan) -> int:
        marked = 0
        for x_m, y_m in scan.to_xy_m():
            if self.mark_occupied(x_m, y_m):
                marked += 1
        return marked

    def update_from_lidar_scan(
        self, scan: LidarScan, sensor_x: float = 0.0, sensor_y: float = 0.0
    ) -> int:
        updated_hits = 0
        for point in scan.valid_points():
            hit_x, hit_y = point.to_xy_m()
            if self.mark_lidar_ray(sensor_x, sensor_y, sensor_x + hit_x, sensor_y + hit_y):
                updated_hits += 1
        return updated_hits

    def mark_lidar_ray(self, sensor_x: float, sensor_y: float, hit_x: float, hit_y: float) -> bool:
        start = self.world_to_grid(sensor_x, sensor_y)
        end = self.world_to_grid(hit_x, hit_y)
        if start is None or end is None:
            return False

        ray_cells = list(_bresenham_cells(start, end))
        for grid_x, grid_y in ray_cells[:-1]:
            if self.data[grid_y, grid_x] != OCCUPIED:
                self.data[grid_y, grid_x] = FREE

        end_x, end_y = end
        self.data[end_y, end_x] = OCCUPIED
        return True


def _bresenham_cells(start: tuple[int, int], end: tuple[int, int]) -> list[tuple[int, int]]:
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    step_x = 1 if x0 < x1 else -1
    step_y = 1 if y0 < y1 else -1
    error = dx - dy
    cells: list[tuple[int, int]] = []

    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            return cells
        doubled_error = 2 * error
        if doubled_error > -dy:
            error -= dy
            x0 += step_x
        if doubled_error < dx:
            error += dx
            y0 += step_y
