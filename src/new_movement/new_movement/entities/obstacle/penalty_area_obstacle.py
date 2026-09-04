import numpy as np

from new_movement.entities.obstacle.static_obstacle import StaticObstacle

from utils.field_util import FieldSide
from utils.math_util import Vector2D

from system_interfaces.msg import VisionGeometry


class PenaltyAreaObstacle(StaticObstacle):
    def __init__(
        self, geometry: VisionGeometry, side: FieldSide, padding: float = 90.0
    ):
        self.side = side
        self.top_left_point = None
        self.top_right_point = None
        self.bot_left_point = None
        self.bot_right_point = None
        self.padding = padding

        if self.side is FieldSide.LEFT:
            self.top_line = "LeftFieldRightPenaltyStretch"
            self.bot_line = "LeftFieldLeftPenaltyStretch"
        else:
            self.top_line = "RightFieldRightPenaltyStretch"
            self.bot_line = "RightFieldLeftPenaltyStretch"
            self.padding = -self.padding

        for line in geometry.field_lines:
            if line.name == self.top_line:
                # TODO CHECK ORDER OF x and y
                self.top_left_point = Vector2D(
                    line.x1 - self.padding, line.y1 + self.padding
                )
                self.top_right_point = Vector2D(
                    line.x2 + self.padding, line.y2 + self.padding
                )
            if line.name == self.bot_line:
                self.bot_left_point = Vector2D(
                    line.x1 - self.padding, line.y1 - self.padding
                )
                self.bot_right_point = Vector2D(
                    line.x2 + self.padding, line.y2 - self.padding
                )

        if not all(
            points is not None
            for points in [
                self.top_left_point,
                self.top_right_point,
                self.bot_left_point,
                self.bot_right_point,
            ]
        ):
            # Fallback heurístico: aproxima área de penalidade a partir de proporções padrão SSL
            fl = getattr(geometry, "field_length", 0) or 0
            fw = getattr(geometry, "field_width", 0) or 0
            if fl > 0 and fw > 0:
                # Proporções típicas (~0.133 da length de profundidade e 0.4 da width de largura)
                depth = fl * 0.133
                width = fw * 0.40
                half_w = width / 2.0
                half_l = fl / 2.0

                if self.side is FieldSide.LEFT:
                    x_inner = -half_l + depth  # borda interna (mais ao centro)
                    x_outer = -half_l  # linha de gol
                else:
                    x_inner = half_l - depth
                    x_outer = half_l

                # Aplica padding (já invertido se RIGHT)
                min_x = min(x_inner, x_outer)
                max_x = max(x_inner, x_outer)
                min_y = -half_w
                max_y = half_w

                self.top_left_point = self.top_left_point or Vector2D(min_x - self.padding, max_y + abs(self.padding))
                self.top_right_point = self.top_right_point or Vector2D(max_x + self.padding, max_y + abs(self.padding))
                self.bot_left_point = self.bot_left_point or Vector2D(min_x - self.padding, min_y - abs(self.padding))
                self.bot_right_point = self.bot_right_point or Vector2D(max_x + self.padding, min_y - abs(self.padding))
            else:
                # Sem dimensões: define área degenerada fora do campo para não interferir
                off = 99999.0
                self.top_left_point = self.top_left_point or Vector2D(-off, off)
                self.top_right_point = self.top_right_point or Vector2D(-off + 10, off)
                self.bot_left_point = self.bot_left_point or Vector2D(-off, off - 10)
                self.bot_right_point = self.bot_right_point or Vector2D(-off + 10, off - 10)
            self._fallback = True
        else:
            self._fallback = False

    def distanceTo(self, curPosition: Vector2D) -> float:
        min_x = min(self.top_left_point.x, self.top_right_point.x)
        max_x = max(self.top_left_point.x, self.top_right_point.x)
        min_y = min(self.top_right_point.y, self.bot_right_point.y)
        max_y = max(self.top_right_point.y, self.bot_right_point.y)

        x, y = curPosition.x, curPosition.y
        dx = max(min_x - x, 0, x - max_x)
        dy = max(min_y - y, 0, y - max_y)
        if dx or dy:
            return Vector2D(dx, dy).size()
        return -min(x - min_x, max_x - x, y - min_y, max_y - y)

    def isCollidingAt(self, curPosition: Vector2D) -> bool:
        min_y = min(self.top_right_point.y, self.bot_right_point.y)
        max_y = max(self.top_right_point.y, self.bot_right_point.y)
        min_x = min(self.top_left_point.x, self.top_right_point.x)
        max_x = max(self.top_left_point.x, self.top_right_point.x)
        if min_y < curPosition.y < max_y:
            if min_x < curPosition.x < max_x:
                return True
        return False

    def adaptDestination(self, tarPosition: Vector2D, margin: float = 30) -> Vector2D:
        if not self.isCollidingAt(tarPosition):
            return tarPosition

        closest_corner = self._findClosestCorner(tarPosition)
        new_destination = Vector2D(tarPosition.x, tarPosition.y)

        min_x, max_x, _, _ = self._bounds()
        # Leave by the edge facing midfield. The other one is the goal line, which
        # coincides with the field border, so escaping through it puts the robot off the
        # field and the border pushes it straight back in here.
        x_ref = min_x if self.side is FieldSide.RIGHT else max_x

        dx = abs(x_ref - tarPosition.x)
        dy = abs(closest_corner.y - tarPosition.y)

        if (
            dx <= dy
        ):  # Maybe give a preference to be close to the center, so give dx more importance of dy?
            new_destination.x = x_ref
        else:
            new_destination.y = closest_corner.y

        return new_destination

    def _findClosestCorner(self, curPosition: Vector2D) -> Vector2D:
        closest_corner = None
        for corner in [
            self.top_left_point,
            self.top_right_point,
            self.bot_left_point,
            self.bot_right_point,
        ]:
            if closest_corner is None or corner.distance(
                curPosition
            ) < closest_corner.distance(curPosition):
                closest_corner = corner

        return closest_corner

    def _bounds(self) -> tuple:
        return (
            min(self.top_left_point.x, self.top_right_point.x),
            max(self.top_left_point.x, self.top_right_point.x),
            min(self.top_right_point.y, self.bot_right_point.y),
            max(self.top_right_point.y, self.bot_right_point.y),
        )

    def bounds(self) -> tuple:
        min_x, max_x, min_y, max_y = self._bounds()
        return (min_x, min_y, max_x, max_y)

    def _check_positions(self, positions: np.ndarray) -> bool:
        min_x, max_x, min_y, max_y = self._bounds()

        xs, ys = positions[:, 0], positions[:, 1]
        inside = (xs > min_x) & (xs < max_x) & (ys > min_y) & (ys < max_y)
        return bool(np.any(inside))

    def _check_segments(self, starts: np.ndarray, ends: np.ndarray) -> bool:
        """
        Exact segment-versus-rectangle test by the slab method: clip the segment against
        the x band and the y band; it enters the area when the two surviving parameter
        intervals overlap.
        """
        min_x, max_x, min_y, max_y = self._bounds()
        direction = ends - starts

        enter = np.zeros(len(starts))
        exit_ = np.ones(len(starts))
        misses = np.zeros(len(starts), dtype=bool)

        for axis, low, high in ((0, min_x, max_x), (1, min_y, max_y)):
            origin = starts[:, axis]
            delta = direction[:, axis]
            parallel = delta == 0.0

            # Parallel to this slab: it can only ever enter if it already lies within it.
            misses |= parallel & ((origin <= low) | (origin >= high))

            safe = np.where(parallel, 1.0, delta)
            t_low = (low - origin) / safe
            t_high = (high - origin) / safe
            near = np.where(parallel, 0.0, np.minimum(t_low, t_high))
            far = np.where(parallel, 1.0, np.maximum(t_low, t_high))

            enter = np.maximum(enter, near)
            exit_ = np.minimum(exit_, far)

        return bool(np.any(~misses & (enter < exit_)))

