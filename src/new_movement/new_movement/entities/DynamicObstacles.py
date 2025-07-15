from .obstacles import Obstacle, ObstaclePriority
from .States import Vector2D

class EnemyRobotObstacle(Obstacle):
    def __init__(self):
        pass

    def getPriority(self) -> ObstaclePriority:
        return ObstaclePriority.HIGH

class AllyRobotObstacle(Obstacle):
    def __init__(self):
        pass

    def getPriority(self) -> ObstaclePriority:
        return ObstaclePriority.MEDIUM

class MovingBallObstacle(Obstacle):
    def __init__(self):
        pass

    def getPriority(self) -> ObstaclePriority:
        return ObstaclePriority.LOW

