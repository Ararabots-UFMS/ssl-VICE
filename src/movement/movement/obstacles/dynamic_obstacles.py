from movement.obstacles.interfaces import DynamicObstacle
from movement.path import steers

import numpy as np

class RobotObstacle(DynamicObstacle):
    def __init__(self, x_obs: np.matrix, orientation: float):
        self.x_obs = x_obs
        self.orientation = orientation

    def is_collision(self, x: np.matrix, ignore: bool = False, padding: float = 90, use_dynamic: bool = True, delta: float):
        if ignore:
            return False

        r = padding

        if use_dynamic:
            center, radius = self.get_dynamic_range(delta)
            distance = np.linalg.norm(self.x_obs[:2] - center)
            r += radius
        else:
            distance = np.linalg.norm(self.x_obs[:2] - x[:2])
            
        if distance < r:
            return True
        
        return False

    def get_dynamic_range(self, delta: float, max_delta: float = 0.5) -> np.matrix, radius:
        if delta > max_delta:
            delta = max_delta
        
        # MAMACO
        direction_vector = np.atleast_1d(np.linalg.norm(self.x[2:], 2, -1))
        direction_vector[direction_vector==0] = 1
        # MAMACO END

        if self.can_break(delta):
            # f_minus é igual à distance de freio
            pass
        else:
            # f_minus é igual à distance de maxima freagem
            pass
        
        max_acc = [[np.sign(self.x_obs[2]) + (self.x_obs[2] == 0), np.sign(self.x_obs[3]) + (self.x_obs[3] == 0)], delta]
        
        f_plus = np.linalg.norm(self.x_obs[:2] - (steer.time_optimal_steer_2d(self.x_obs, [max_acc]))[:2])

        radius = abs(f_plus - f_minus) / 2
        center = self.x_obs[:2] + direction_vector * (f_minus + radius)
        
        return center, radius
    
    def can_break(self, delta):
        pass

    def update_state(self, x_obs: np.matrix, orientation):
        self.x_obs = x_obs
        self.orientation = orientation

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