from copy import copy

import numpy as np

from movement.entities.obstacle.static_obstacle import StaticObstacle

from utils.math_util import Vector2D


# How far outside a grown obstacle the robot or its goal is left. Less, and a route
# has to leave along the tangent to count as clear.
ROUTE_EDGE = 10.0


class GenericCircleObstacle(StaticObstacle):
    def __init__(
        self, center: Vector2D, radius: float, padding: float = 90.0, clearance: float = 0.0
    ):
        self.center: Vector2D = center
        self.radius: float = radius + padding
        # Room a passing route keeps beyond the edge. Only for_route applies it.
        self.clearance: float = clearance

    def for_route(self, start: Vector2D, goal: Vector2D) -> "GenericCircleObstacle":
        """
        Grown by the clearance, but never over the robot or its goal: a route has to be
        able to start and end where they are, so near them only the edge is kept.
        """
        if self.clearance <= 0.0:
            return self
        nearest = min(self.center.distance(start), self.center.distance(goal))
        radius = min(self.radius + self.clearance, nearest - ROUTE_EDGE)
        if radius <= self.radius:
            return self
        grown = copy(self)
        grown.radius = radius
        grown.clearance = 0.0
        return grown

    def distanceTo(self, curPosition: Vector2D) -> float:
        return self.center.distance(curPosition) - self.radius

    def isCollidingAt(self, curPosition: Vector2D) -> bool:
        if self.distanceTo(curPosition) < 0:
            return True
        return False

    def adaptDestination(self, tarPosition: Vector2D, margin: float = 30) -> Vector2D:
        # Projects the inside target point into the outer edge of the circle
        # TODO Check if the order of subtraction is correct
        if not self.isCollidingAt(tarPosition):
            return tarPosition

        center_to_target = tarPosition.subtract(self.center)

        dist = center_to_target.size()
        if dist == 0:
            center_to_target = Vector2D(1, 0)  # arbitrary direction

        center_to_target = center_to_target.norm()

        return self.center.add(center_to_target.multiplyByScalar(self.radius + margin))

    def bounds(self) -> tuple:
        return (
            self.center.x - self.radius,
            self.center.y - self.radius,
            self.center.x + self.radius,
            self.center.y + self.radius,
        )

    def _check_positions(self, positions: np.ndarray) -> bool:
        center = np.array([self.center.x, self.center.y])
        diffs = positions - center
        dists_sq = np.einsum("ij,ij->i", diffs, diffs)
        
        return bool(np.any(dists_sq < self.radius ** 2))

    def _check_segments(self, starts: np.ndarray, ends: np.ndarray) -> bool:
        """Exact segment-versus-disc test, closed form: no subdivision needed."""
        center = np.array([self.center.x, self.center.y])
        direction = ends - starts                       # (N, 2)
        to_start = starts - center                      # (N, 2)

        length_sq = np.einsum("ij,ij->i", direction, direction)
        projection = -np.einsum("ij,ij->i", to_start, direction)
        # A zero-length segment is just its start point; the clip keeps the closest
        # point on the segment rather than on the infinite line through it.
        param = np.divide(
            projection, length_sq, out=np.zeros_like(projection), where=length_sq > 0
        )
        np.clip(param, 0.0, 1.0, out=param)

        offset = to_start + param[:, np.newaxis] * direction
        dists_sq = np.einsum("ij,ij->i", offset, offset)

        return bool(np.any(dists_sq < self.radius ** 2))
