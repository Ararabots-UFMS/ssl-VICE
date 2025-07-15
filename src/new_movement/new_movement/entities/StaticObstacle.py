from .obstacles import Obstacle, ObstaclePriority
from .States import Vector2D
from system_interfaces.msg import VisionGeometry


class StaticObstacle(Obstacle):
    def velocity(self) -> float:
        return 0.0
    
class FieldBorderObstacle(StaticObstacle):
    def __init__(self, geometry: VisionGeometry, padding: float = 90): # 90mm = robots radius
        self.top_left_point:  Vector2D = None
        self.top_right_point: Vector2D = None
        self.bot_left_point:  Vector2D = None
        self.bot_right_point: Vector2D = None
    
        for line in geometry.field_lines:
            if line.name == 'TopTouchLine':
                self.top_left_point = Vector2D(line.x1, line.y1) # Not sure if x1 is on the left or right side
                self.top_right_point = Vector2D(line.x2, line.y2)

            elif line.name == 'BottomTouchLine':
                self.bot_left_point = Vector2D(line.x1, line.y1)
                self.bot_right_point = Vector2D(line.x2, line.y2)

    def distanceTo(self, curPosition: Vector2D) -> float:
        # Searching closest corner
        closest_corner = None
        for corner in [self.top_left_point, self.top_right_point, self.bot_left_point, self.bot_right_point]:
            if closest_corner is None or corner.distance(curPosition) < closest_corner:
                closest_corner = corner

        if closest_corner.x < closest_corner.y:
            return abs(curPosition.x - closest_corner.x)
        else:
            return abs(curPosition.y - closest_corner.y)

    def isCollidingAt(self, curPosition: Vector2D) -> bool:
        pass

    def adaptDestination(self, tarPosition: Vector2D) -> Vector2D:
        pass

    def getPriority(self) -> ObstaclePriority:
        return ObstaclePriority.HIGHEST

class PenaltyAreaObstacle(StaticObstacle):
    def __init__(self):
        pass
    
    def getPriority(self) -> ObstaclePriority:
        return ObstaclePriority.LOW

class GenericCircleObstacle(StaticObstacle):
    def __init__(self):
        pass
    
    def getPriority(self) -> ObstaclePriority:
        return ObstaclePriority.LOWEST