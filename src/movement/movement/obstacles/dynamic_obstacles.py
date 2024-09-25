from movement.obstacles.interfaces import DynamicObstacle
from movement.path.steer import integrate_control_2d

import numpy as np

class RobotObstacle(DynamicObstacle):
    def __init__(self, x_obs: np.matrix, orientation: float, u_max: float = 1.0, u_min: float = -1.0):
        self.x_obs = x_obs
        self.orientation = orientation
        self.u_max = u_max
        self.u_min = u_min

    def is_colission(self, x: np.matrix, delta: float, ignore: bool = False, padding: float = 90, use_dynamic: bool = True, max_delta: float = 0.5):
        if ignore:
            return False

        r = padding

        if use_dynamic:
            center, radius = self.get_dynamic_range(delta, max_delta)
            distance = np.linalg.norm(self.x_obs[:2] - center)
            r += radius
        else:
            distance = np.linalg.norm(self.x_obs[:2] - x[:2])
            
        if distance < r:
            return True
        
        return False

    def get_dynamic_range(self, delta: float, max_delta: float = 0.5) -> (np.matrix, float):
        if delta > max_delta:
            delta = max_delta
        
        # Normalizing velocity vector to unit movement direction
        direction_vector = self.x_obs[2:] / (np.linalg.norm(self.x_obs[2:] + 0.001))

        direc_acc = [np.sign(self.x_obs[2]) + (self.x_obs[2] == 0), np.sign(self.x_obs[3]) + (self.x_obs[3] == 0)]
        
        can_break, break_time = self.can_break(direc_acc, delta)

        max_acc = [[[direc_acc[0] * self.u_max, direc_acc[1] * self.u_max], delta]]
        min_acc = [[[direc_acc[0] * self.u_min, direc_acc[1] * self.u_min], break_time]]

        f_plus = np.linalg.norm(self.x_obs[:2] - (integrate_control_2d(self.x_obs, max_acc))[:2])

        f_minus = np.linalg.norm(self.x_obs[:2] - (integrate_control_2d(self.x_obs, min_acc))[:2])

        radius = abs(f_plus - f_minus) / 2

        center = self.x_obs[:2].T + (direction_vector * (f_minus + radius)).transpose()
        
        return center.T, radius

    def can_break(self, u, delta):
        velocity_mag = np.linalg.norm(self.x_obs[2:])
        acc_mag = np.linalg.norm(u)
        break_time = velocity_mag / acc_mag

        if break_time > delta:
            return False, delta

        return True, break_time

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


import matplotlib.pyplot as plt

fig, ax = plt.subplots()

plt.axis([0, 4500, 0, 3000])

x_obs = np.matrix([[3000], [0], [-3000], [0]]).astype(float)

max_delta = 0.5
deltas = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]

obs = RobotObstacle(x_obs, 0, 1000, -1000)

for i in deltas:
    center, radius = obs.get_dynamic_range(i)

    obstacle_plt = plt.Circle((float(x_obs[0]), (float(x_obs[1]))), 90, color ='red')
    padding_plt = plt.Circle(((float(center[0])), (float(center[1]))), radius + 90, color='red', fill=False)
    ax.add_patch( obstacle_plt )
    ax.add_patch( padding_plt )

plt.show()