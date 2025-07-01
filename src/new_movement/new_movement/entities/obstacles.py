from .States import Vector2D, State
from .Trajectory import Trajectory
from typing import Optional


class Obstacle():
    def __init__(self) -> None:
        pass

    def distanceTo(self, point: Vector2D) -> float:
        '''Given a position (Vector2D), return the distance from the obstacle to the point'''
        pass

    def collisionAt(self, trajectory: Trajectory) -> Optional[float]:
        '''Given the trajectory, return the first time where a collision happens, if none, then return none'''
        pass

    def is_collision(self, point: Vector2D) -> bool:
        pass

class RobotObstacle(Obstacle):
    def __init__(self, state: State) -> None:
        pass

class AreaObstacle(Obstacle):
    def __init__(self) -> None:
        pass