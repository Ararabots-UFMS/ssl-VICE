from new_movement.entities.obstacles import Obstacle, ObstaclePriority
from new_movement.entities.States import Vector2D
from math import copysign
from dataclasses import dataclass
from system_interfaces.msg import VisionGeometry


@dataclass
class FieldSide:
    RIGHT = 0
    LEFT = 1


class StaticObstacle(Obstacle):
    def velocity(self) -> float:
        return 0.0


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

    def getPriority(self) -> ObstaclePriority:
        return ObstaclePriority.HIGHEST


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

        x_ref = self.top_right_point.x

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

    def getPriority(self) -> ObstaclePriority:
        return ObstaclePriority.LOW


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

    def getPriority(self) -> ObstaclePriority:
        return ObstaclePriority.LOWEST
