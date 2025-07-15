from .obstacles import Obstacle, ObstaclePriority
from .States import Vector2D
from system_interfaces.msg import VisionGeometry


class StaticObstacle(Obstacle):
    def velocity(self) -> float:
        return 0.0
    
class FieldBorderObstacle(StaticObstacle):
    def __init__(self, geometry: VisionGeometry, padding: float = 90.0): # 90mm = robots radius
        self.top_left_point:  Vector2D = None
        self.top_right_point: Vector2D = None
        self.bot_left_point:  Vector2D = None
        self.bot_right_point: Vector2D = None

    
        for line in geometry.field_lines:
            if line.name == 'TopTouchLine':
                # TODO Check if padding signal is correct
                self.top_left_point = Vector2D(line.x1 + padding, line.y1 - padding) # Not sure if x1 is on the left or right side
                self.top_right_point = Vector2D(line.x2 - padding, line.y2 - padding)

            elif line.name == 'BottomTouchLine':
                self.bot_left_point = Vector2D(line.x1 + padding, line.y1 + padding)
                self.bot_right_point = Vector2D(line.x2 - padding, line.y2 + padding)

    def distanceTo(self, curPosition: Vector2D) -> float:
        closest_corner = self._findClosestCorner(curPosition)

        if closest_corner.x < closest_corner.y:
            return abs(curPosition.x - closest_corner.x)
        else:
            return abs(curPosition.y - closest_corner.y)

    def isCollidingAt(self, curPosition: Vector2D) -> bool:
        if(curPosition.x > self.top_left_point.x and curPosition.x < self.top_right_point):
            if(curPosition.y > self.bot_left_point.y and curPosition.y < self.top_left_point.y):
                return True
        
        return False

    def adaptDestination(self, tarPosition: Vector2D) -> Vector2D:
        closest_corner = self._findClosestCorner(tarPosition)

        new_destination = Vector2D(tarPosition.x , tarPosition.y)

        if(abs(tarPosition.x) > closest_corner.x):
            new_destination.x = closest_corner.x
        if(abs(tarPosition.y) > closest_corner.y):
            new_destination.y = closest_corner.y

        return new_destination

    def _findClosestCorner(self, curPosition: Vector2D) -> Vector2D:
        closest_corner = None
        for corner in [self.top_left_point, self.top_right_point, self.bot_left_point, self.bot_right_point]:
            if closest_corner is None or corner.distance(curPosition) < closest_corner:
                closest_corner = corner

        return closest_corner

    def getPriority(self) -> ObstaclePriority:
        return ObstaclePriority.HIGHEST

class PenaltyAreaObstacle(StaticObstacle):
    def __init__(self):
        # TODO
        pass

    def distanceTo(self, curPosition: Vector2D) -> float:
        # TODO
        pass

    def isCollidingAt(self, curPosition: Vector2D) -> bool:
        # TODO
        pass

    def adaptDestination(self, tarPosition: Vector2D) -> Vector2D:
        # TODO
        pass

    def getPriority(self) -> ObstaclePriority:
        return ObstaclePriority.LOW

class GenericCircleObstacle(StaticObstacle):
    def __init__(self, center: Vector2D, radius: float, padding: float = 90.0):
        self.center: Vector2D = center
        self.radius: float = radius + padding

    def distanceTo(self, curPosition: Vector2D) -> float:
        return self.center.distance(curPosition)

    def isCollidingAt(self, curPosition: Vector2D) -> bool:
        if self.distanceTo(curPosition) < self.radius:
            return True
        return False

    def adaptDestination(self, tarPosition: Vector2D) -> Vector2D:
        # Projects the inside target point into the outer edge of the circle
        # TODO Check if the order of subtraction is correct
        center_to_target = Vector2D(tarPosition.x - self.center.x, tarPosition.y - self.center.y)

        distance = self.center.distance(tarPosition)
        scalar = self.radius / distance

        center_to_target.multiplyByScalar(scalar)

        return Vector2D(tarPosition.x + center_to_target.x, tarPosition.y + center_to_target.y)

    def getPriority(self) -> ObstaclePriority:
        return ObstaclePriority.LOWEST