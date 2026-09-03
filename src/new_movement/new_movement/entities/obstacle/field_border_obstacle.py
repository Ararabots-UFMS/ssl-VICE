from math import copysign
import numpy as np

from new_movement.entities.obstacle.static_obstacle import StaticObstacle

from system_interfaces.msg import VisionGeometry

from utils.math_util import Vector2D


class FieldBorderObstacle(StaticObstacle):
    def __init__(
        self, geometry: VisionGeometry, padding: float = 90.0
    ):  # 90mm = robots radius
        self.top_left_point: Vector2D = None
        self.top_right_point: Vector2D = None
        self.bot_left_point: Vector2D = None
        self.bot_right_point: Vector2D = None

        for line in geometry.field_lines:
            if line.name == "TopTouchLine":
                # TODO Check if padding signal is correct
                self.top_left_point = Vector2D(
                    line.x1 + padding, line.y1 - padding
                )  # Not sure if x1 is on the left or right side
                self.top_right_point = Vector2D(line.x2 - padding, line.y2 - padding)

            elif line.name == "BottomTouchLine":
                self.bot_left_point = Vector2D(line.x1 + padding, line.y1 + padding)
                self.bot_right_point = Vector2D(line.x2 - padding, line.y2 + padding)

        if not all(
            points is not None
            for points in [
                self.top_left_point,
                self.top_right_point,
                self.bot_left_point,
                self.bot_right_point,
            ]
        ):
            # Fallback: use general field dimensions to generate an approximate rectangle
            fl = getattr(geometry, "field_length", 0) or 0
            fw = getattr(geometry, "field_width", 0) or 0
            if fl > 0 and fw > 0:
                half_l = fl / 2.0
                half_w = fw / 2.0
                # Applies padding by moving inward
                self.top_left_point = self.top_left_point or Vector2D(-half_l + padding, half_w - padding)
                self.top_right_point = self.top_right_point or Vector2D(half_l - padding, half_w - padding)
                self.bot_left_point = self.bot_left_point or Vector2D(-half_l + padding, -half_w + padding)
                self.bot_right_point = self.bot_right_point or Vector2D(half_l - padding, -half_w + padding)
            else:
                default_extent = 1000.0
                self.top_left_point = self.top_left_point or Vector2D(-default_extent, default_extent)
                self.top_right_point = self.top_right_point or Vector2D(default_extent, default_extent)
                self.bot_left_point = self.bot_left_point or Vector2D(-default_extent, -default_extent)
                self.bot_right_point = self.bot_right_point or Vector2D(default_extent, -default_extent)
            self._fallback = True
        else:
            self._fallback = False

    def distanceTo(self, curPosition: Vector2D) -> float:
        if not self.isCollidingAt(curPosition):
            closest_corner = self._findClosestCorner(curPosition)
            distance = min(
                abs(curPosition.x - closest_corner.x),
                abs(curPosition.y - closest_corner.y),
            )
            return distance

        else:
            dx = max(
                self.top_left_point.x - curPosition.x,
                0,
                curPosition.x - self.top_right_point.x,
            )
            dy = max(
                self.bot_right_point.y - curPosition.y,
                0,
                curPosition.y - self.top_right_point.y,
            )
            distance = Vector2D(dx, dy).size()

            return -distance

    def isCollidingAt(self, curPosition: Vector2D) -> bool:
        if (
            curPosition.x > self.top_left_point.x
            and curPosition.x < self.top_right_point.x
        ):
            if (
                curPosition.y > self.bot_left_point.y
                and curPosition.y < self.top_left_point.y
            ):
                return False

        return True

    def adaptDestination(self, tarPosition: Vector2D, margin: float = 50) -> Vector2D:
        closest_corner = self._findClosestCorner(tarPosition)

        new_destination = Vector2D(tarPosition.x, tarPosition.y)

        if abs(tarPosition.x) > abs(closest_corner.x):
            new_destination.x = closest_corner.x - copysign(margin, closest_corner.x)
        if abs(tarPosition.y) > abs(closest_corner.y):
            new_destination.y = closest_corner.y - copysign(margin, closest_corner.y)

        return new_destination

    def _findClosestCorner(self, curPosition: Vector2D) -> Vector2D:
        closest_corner = None
        for corner in [
            self.top_left_point,
            self.top_right_point,
            self.bot_left_point,
            self.bot_right_point,
        ]:
            if closest_corner is not None:
                if corner.distance(curPosition) < closest_corner.distance(curPosition):
                    closest_corner = corner
            else:
                closest_corner = corner

        return closest_corner

    def _check_positions(self, positions: np.ndarray) -> bool:
        xs, ys = positions[:, 0], positions[:, 1]
        inside = (
            (xs > self.top_left_point.x)
            & (xs < self.top_right_point.x)
            & (ys > self.bot_left_point.y)
            & (ys < self.top_left_point.y)
        )
        return bool(np.any(~inside))

    def _check_segments(self, starts: np.ndarray, ends: np.ndarray) -> bool:
        """
        Colliding here means leaving the field, and the playable area is a convex
        rectangle: a segment stays inside exactly when both of its endpoints do. So
        checking the two endpoints is already an exact sweep — unlike every other
        obstacle, this shape never needed subdivision.
        """
        return self._check_positions(starts) or self._check_positions(ends)
