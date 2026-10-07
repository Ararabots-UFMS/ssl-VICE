"""The opponent's half, inflated by the robot radius, for defensive coverage."""

import numpy as np

from movement.entities.obstacle.static_obstacle import StaticObstacle
from utils.math_util import Vector2D


class HalfFieldObstacle(StaticObstacle):
    def __init__(self, defensive_half: int, padding: float = 90.0):
        if defensive_half not in (-1, 1):
            raise ValueError("defensive_half must be -1 or 1")
        self.side = defensive_half
        self.padding = padding

    def distanceTo(self, curPosition: Vector2D) -> float:
        return self.side * curPosition.x - self.padding

    def isCollidingAt(self, curPosition: Vector2D) -> bool:
        return self.distanceTo(curPosition) <= 0

    def adaptDestination(self, tarPosition: Vector2D, margin: float = 50) -> Vector2D:
        return Vector2D(
            self.side * max(self.side * tarPosition.x, self.padding + margin),
            tarPosition.y,
        )

    def _check_positions(self, positions: np.ndarray) -> bool:
        return bool(np.any(self.side * positions[:, 0] <= self.padding))

    def _check_segments(self, starts: np.ndarray, ends: np.ndarray) -> bool:
        # A half-plane is convex, so checking both endpoints is exact.
        return self._check_positions(starts) or self._check_positions(ends)
