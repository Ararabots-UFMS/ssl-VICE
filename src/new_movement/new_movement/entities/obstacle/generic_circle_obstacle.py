import numpy as np

from new_movement.entities.obstacle.static_obstacle import StaticObstacle

from utils.math_util import Vector2D


class GenericCircleObstacle(StaticObstacle):
    def __init__(self, center: Vector2D, radius: float, padding: float = 90.0):
        self.center: Vector2D = center
        self.radius: float = radius + padding

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
