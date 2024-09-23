from movement.obstacles.interfaces import DynamicObstacle

import numpy as np

class RobotObstacle(DynamicObstacle):
    def __init__(self, x: np.matrix, orientation: float, ally: bool):
        pass

    def is_collision(self, x: np.matrix, ignore: bool = False, padding: float = 90):
        pass

    def get_dynamic_range(self, delta: float):
        pass

    def update_state(self, x: np.matrix):
        pass

class BallObstacle(DynamicObstacle):
    def __init__(self, x: np.matrix, mode: str):
        self.mode = mode
        pass

    def is_colission(self, x: np.matrix, ignore: bool = True, padding: float = 90):
        pass

    def get_dynamic_range(self, delta: float):
        pass

    def update_state(self, x: np.matrix):
        pass

    def set_mode(mode: str):
        pass