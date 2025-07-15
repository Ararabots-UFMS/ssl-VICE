from .obstacles import Obstacle, ObstaclePriority

class StaticObstacle(Obstacle):
    def velocity(self) -> float:
        return 0.0
    
class FieldBorderObstacle(StaticObstacle):
    pass

class PenaltyAreaObstacle(StaticObstacle):
    pass

class GenericCircleObstacle(StaticObstacle):
    pass