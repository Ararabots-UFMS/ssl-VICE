from new_movement.entities.obstacles import Obstacle, ObstaclePriority
from new_movement.entities.States import Vector2D
from dataclasses import dataclass
from system_interfaces.msg import VisionGeometry

@dataclass
class FieldSide():
    RIGHT = 0
    LEFT = 1

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

            elif line.name == 'BottonTouchLine':
                self.bot_left_point = Vector2D(line.x1 + padding, line.y1 + padding)
                self.bot_right_point = Vector2D(line.x2 - padding, line.y2 + padding)

    def distanceTo(self, curPosition: Vector2D) -> float:
        if(not self.isCollidingAt(curPosition)):
            closest_corner = self._findClosestCorner(curPosition)
            distance = min(abs(curPosition.x - closest_corner.x), abs(curPosition.y - closest_corner.y))
            return distance
        
        else:
            dx = max(self.top_left_point.x - curPosition.x, 0 , curPosition.x - self.top_right_point.x)
            dy = max(self.bot_right_point.y - curPosition.y, 0 , curPosition.y - self.top_right_point.y)
            distance = Vector2D(dx, dy).size()
        
            return -distance

    def isCollidingAt(self, curPosition: Vector2D) -> bool:
        if(curPosition.x > self.top_left_point.x and curPosition.x < self.top_right_point.x):
            if(curPosition.y > self.bot_left_point.y and curPosition.y < self.top_left_point.y):
                return False
        
        return True

    def adaptDestination(self, tarPosition: Vector2D) -> Vector2D:
        closest_corner = self._findClosestCorner(tarPosition)

        new_destination = Vector2D(tarPosition.x , tarPosition.y)

        if(abs(tarPosition.x) > abs(closest_corner.x)):
            new_destination.x = closest_corner.x
        if(abs(tarPosition.y) > abs(closest_corner.y)):
            new_destination.y = closest_corner.y

        return new_destination

    def _findClosestCorner(self, curPosition: Vector2D) -> Vector2D:
        closest_corner = None
        for corner in [self.top_left_point, self.top_right_point, self.bot_left_point, self.bot_right_point]:
            if closest_corner is None or corner.distance(curPosition) < closest_corner.distance(curPosition):
                closest_corner = corner

        return closest_corner

    def getPriority(self) -> ObstaclePriority:
        return ObstaclePriority.HIGHEST

class PenaltyAreaObstacle(StaticObstacle):
    def __init__(self, geometry: VisionGeometry, side: FieldSide, padding: float = 90.0):
        self.side = side
        self.top_left_point = None
        self.top_right_point = None
        self.bot_left_point = None
        self.bot_right_point = None
        self.padding = padding

        if self.side is FieldSide.LEFT:
            self.top_line = 'LeftFieldRightPenaltyStretch'
            self.bot_line = 'LeftFieldLeftPenaltyStretch'
        else:
            self.top_line = 'RightFieldRightPenaltyStretch'
            self.bot_line = 'RightFieldLeftPenaltyStretch'
            self.padding = -self.padding  

        for line in geometry.field_lines:
            if line.name == self.top_line:
                #TODO CHECK ORDER OF x and y
                self.top_left_point = Vector2D(line.x1 - self.padding, line.y1 + self.padding)
                self.top_right_point = Vector2D(line.x2 + self.padding, line.y2 + self.padding)
            if line.name == self.bot_line:
                self.bot_left_point = Vector2D(line.x1 - self.padding, line.y1 - self.padding)
                self.bot_right_point = Vector2D(line.x2 + self.padding, line.y2 - self.padding)
        
    def distanceTo(self, curPosition: Vector2D) -> float:
        if(not self.isCollidingAt(curPosition)):
            dx = max(self.top_left_point.x - curPosition.x, 0 , curPosition.x - self.top_right_point.x)
            dy = max(self.bot_right_point.y - curPosition.y, 0 , curPosition.y - self.top_right_point.y)
            distance = Vector2D(dx, dy).size()

            return distance
        else:
            closest_coner = self._findClosestCorner(curPosition)
            distance = min(abs(curPosition.x - closest_coner.x), abs(curPosition.y - closest_coner.y))

            return -distance

    def isCollidingAt(self, curPosition: Vector2D) -> bool:
        if (curPosition.y < self.top_right_point.y and curPosition.y > self.bot_right_point.y):
            if(curPosition.x > self.top_left_point.x and curPosition.x < self.top_right_point.x):
                return True

        return False

    def adaptDestination(self, tarPosition: Vector2D) -> Vector2D:
        if(not self.isCollidingAt(tarPosition)):
            return tarPosition

        closest_corner = self._findClosestCorner(tarPosition)
        new_destination = Vector2D(tarPosition.x, tarPosition.y)

        # This blocks a new destination on the side of the goal
        if(self.side is FieldSide.LEFT):
            x_ref = self.top_right_point.x
        else:
            x_ref = self.top_left_point.x

        dx = abs(x_ref - tarPosition.x)
        dy = abs(closest_corner.y - tarPosition.y)

        if(dx <= dy): #Maybe give a preference to be close to the center, so give dx more importance of dy?
            new_destination.x = x_ref
        else:
            new_destination.y = closest_corner.y
            
        return new_destination

    def _findClosestCorner(self, curPosition: Vector2D) -> Vector2D:
        closest_corner = None
        for corner in [self.top_left_point, self.top_right_point, self.bot_left_point, self.bot_right_point]:
            if closest_corner is None or corner.distance(curPosition) < closest_corner.distance(curPosition):
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
    
# --- Penalty Area Tests ---

geometry = VisionGeometry()
lines = []

from system_interfaces.msg import FieldLineSegment

# Define penalty area lines (example for LEFT side)
LeftFieldRightPenaltyStretch = FieldLineSegment()
LeftFieldRightPenaltyStretch.name = "LeftFieldRightPenaltyStretch"
LeftFieldRightPenaltyStretch.x1 = -2250.0
LeftFieldRightPenaltyStretch.y1 = 800.0
LeftFieldRightPenaltyStretch.x2 = -1650.0
LeftFieldRightPenaltyStretch.y2 = 800.0

LeftFieldLeftPenaltyStretch = FieldLineSegment()
LeftFieldLeftPenaltyStretch.name = "LeftFieldLeftPenaltyStretch"
LeftFieldLeftPenaltyStretch.x1 = -2250.0
LeftFieldLeftPenaltyStretch.y1 = -800.0
LeftFieldLeftPenaltyStretch.x2 = -1650.0
LeftFieldLeftPenaltyStretch.y2 = -800.0

lines.append(LeftFieldRightPenaltyStretch)
lines.append(LeftFieldLeftPenaltyStretch)

geometry.field_lines = lines

# Test Constructor
penalty = PenaltyAreaObstacle(geometry, FieldSide.LEFT, padding=90.0)

print(" --- PenaltyAreaObstacle Constructor ---")
print(f"top_left: {penalty.top_left_point}")
print(f"top_right: {penalty.top_right_point}")
print(f"bot_left: {penalty.bot_left_point}")
print(f"bot_right: {penalty.bot_right_point}")
print(" --- END ---\n")

# Test positions (adjust as needed for your field)
position_tests = [
    Vector2D(-2000.0, 0.0),
    Vector2D(-1700.0, 700.0),
    Vector2D(-1800.0, -700.0),
    Vector2D(-2250.0, 800.0),
    Vector2D(-1650.0, -800.0),
    Vector2D(-1600.0, 0.0),
    Vector2D(-2300.0, 0.0),
    Vector2D(-1950.0, 900.0),
    Vector2D(-1950.0, -900.0),
]

print(" --- TEST DISTANCE --- ")
for position in position_tests:
    print(f"Position: {position} Distance: {penalty.distanceTo(position)}")
print(" --- END ---\n")

print(" --- TEST COLLISION --- ")
for position in position_tests:
    print(f"Position: {position} Colliding: {penalty.isCollidingAt(position)}")
print(" --- END ---\n")

print(" --- TEST ADAPT DESTINATION --- ")
for position in position_tests:
    print(f"Position: {position} New_destination: {penalty.adaptDestination(position)}")
print(" --- END ---\n")

print(" --- TEST CLOSEST CORNER --- ")
for position in position_tests:
    print(f"Position: {position} ClosestCorner: {penalty._findClosestCorner(position)}")
print(" --- END ---\n")