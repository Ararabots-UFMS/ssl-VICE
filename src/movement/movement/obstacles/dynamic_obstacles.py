from movement.obstacles.interfaces import DynamicObstacle
from system_interfaces.msg import Robots

from typing import Tuple

from math import sqrt, sin, cos

from ruckig import InputParameter, OutputParameter, Result, Ruckig, Trajectory, ControlInterface


class RoboObstacle(DynamicObstacle):
    def __init__(self, state: Robots, radius: 90, max_delta: float = 0.5, max_acc = 1, max_vel = 3):
        self.state = state
        self.radius = radius

        self.max_delta = max_delta
        self.max_acc = max_acc
        self.max_vel = max_vel

    def is_colission(self, delta: float, ref_point: Tuple[float, float], ref_radius = 90, use_dynamic: bool = True) -> bool:
        dynamic_center, dynamic_radius = self.get_dynamic_range(delta) if use_dynamic else ref_point

        distance = sqrt((dynamic_center[0] - ref_point[0])**2 + (dynamic_center[1] - ref_point[1])**2)

        if distance > dynamic_radius + ref_radius:
            return True
        
        return False

    def get_dynamic_range(self, delta) -> Tuple[Tuple[float, float], float]:        
        # Using exact formulation of dynamic obstacle modelation from Tigers ETDP from 2024
        if delta < 0:
            delta = 0
        elif delta > self.max_delta:
            delta = self.max_delta

        f_plus, f_minus = self.bb_range(delta)

        # Distance between f_minus and f_plus...
        dynamic_radius = sqrt((f_plus[0] - f_minus[0]) ** 2 + (f_plus[1] - f_minus[1]) ** 2)

        velocity_mag = (self.state[1][0] ** 2 + self.state[1][1] ** 2)
        velocity_norm = self.state[1][0] / velocity_mag, self.state[1][1] / velocity_mag

        dynamic_center = self.state[0] + (velocity_norm[0] * (f_minus[0] + dynamic_radius)), self.state[1] + (velocity_norm[1] * (f_minus[1] + dynamic_radius))

        obs_radius = self.radius + dynamic_radius

        return dynamic_center, obs_radius
        
    def bb_range(self, delta) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        # Get f- and f+ 1d bang bang trajectorys given the delta time.

        # F plus
        inp = InputParameter(2)
        inp.control_interface = ControlInterface.Velocity

        inp.current_position = [self.state.position_x, self.state.position_y]
        inp.current_velocity = [self.state.velocity_x, self.state.velocity_y]

        inp.target_velocity = [self.max_vel, self.max_vel]
        inp.target_acceleration = [self.max_acc, self.max_acc]

        # F minus
        inp2 = InputParameter(2)
        inp2.control_interface = ControlInterface.Velocity

        inp2.current_position = [self.state.position_x, self.state.position_y]
        inp2.current_velocity = [self.state.velocity_x, self.state.velocity_y]

        inp2.target_velocity = [self.max_vel, self.max_vel]
        inp2.target_acceleration = [-self.max_acc, -self.max_acc]

        otg = Ruckig(2)
        f_plus = Trajectory(2)
        f_minus = Trajectory(2)

        otg.calculate(inp, f_plus)
        otg.calculate(inp2, f_minus)

        state_plus = f_plus.at_time(delta)
        state_minus = f_minus.at_time(delta)

        # (f+ x, f+ y), (f- x, f- y)
        return ((state_plus[0][0], state_plus[0][0]), (state_minus[0][0], state_minus[0][1]))

    def update_state(self, state: Robots) -> None:
        self.state = state


# from movement.obstacles.interfaces import DynamicObstacle
# from movement.control.steer import integrate_control_2d

# import numpy as np

# # TODO Refactor using ruckig, remove integrate_control_2d and numpy

# # (x, y, vx, vy) -> ([x ,y , theta], [vx, vy, v_theta])

# class RobotObstacle(DynamicObstacle):
#     def __init__(self, x_obs: np.matrix, orientation: float, u_max: float = 1.0, u_min: float = -1.0):
#         self.x_obs = x_obs
#         self.orientation = orientation
#         self.u_max = u_max
#         self.u_min = u_min

#     def is_colission(self, x: np.matrix, delta: float, ignore: bool = False, padding: float = 90, use_dynamic: bool = True, max_delta: float = 0.5):
#         if ignore:
#             return False

#         r = padding

#         if use_dynamic:
#             center, radius = self.get_dynamic_range(delta, max_delta)
#             distance = np.linalg.norm(self.x_obs[:2] - center)
#             r += radius
#         else:
#             distance = np.linalg.norm(self.x_obs[:2] - x[:2])
            
#         if distance < r:
#             return True
        
#         return False

#     def get_dynamic_range(self, delta: float, max_delta: float = 0.5) -> (np.matrix, float):
#         if delta > max_delta:
#             delta = max_delta
        
#         # Normalizing velocity vector to unit movement direction
#         direction_vector = self.x_obs[2:] / (np.linalg.norm(self.x_obs[2:] + 0.001))

#         direc_acc = [np.sign(self.x_obs[2]) + (self.x_obs[2] == 0), np.sign(self.x_obs[3]) + (self.x_obs[3] == 0)]
        
#         can_break, break_time = self.can_break(direc_acc, delta)

#         max_acc = [[[direc_acc[0] * self.u_max, direc_acc[1] * self.u_max], delta]]
#         min_acc = [[[direc_acc[0] * self.u_min, direc_acc[1] * self.u_min], break_time]]

#         f_plus = np.linalg.norm(self.x_obs[:2] - (integrate_control_2d(self.x_obs, max_acc))[:2])

#         f_minus = np.linalg.norm(self.x_obs[:2] - (integrate_control_2d(self.x_obs, min_acc))[:2])

#         radius = abs(f_plus - f_minus) / 2

#         center = self.x_obs[:2].T + (direction_vector * (f_minus + radius)).transpose()
        
#         return center.T, radius

#     def can_break(self, u, delta):
#         velocity_mag = np.linalg.norm(self.x_obs[2:])
#         acc_mag = np.linalg.norm(u)
#         break_time = velocity_mag / acc_mag

#         if break_time > delta:
#             return False, delta

#         return True, break_time

#     def update_state(self, x_obs: np.matrix, orientation):
#         self.x_obs = x_obs
#         self.orientation = orientation

# class BallObstacle(DynamicObstacle):
#     def __init__(self, x: np.matrix, mode: str):
#         self.mode = mode
#         pass

#     def is_colission(self, x: np.matrix, ignore: bool = True, padding: float = 90):
#         pass

#     def get_dynamic_range(self, delta: float):
#         pass

#     def update_state(self, x: np.matrix):
#         pass

#     def set_mode(mode: str):
#         pass


# # import matplotlib.pyplot as plt

# # fig, ax = plt.subplots()

# # plt.axis([0, 4500, 0, 3000])

# # x_obs = np.matrix([[3000], [0], [1000], [0]]).astype(float)

# # max_delta = 0.5
# # deltas = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]

# # obs = RobotObstacle(x_obs, 0, 1000, -1000)

# # for i in deltas:
# #     center, radius = obs.get_dynamic_range(i)

# #     obstacle_plt = plt.Circle((float(x_obs[0]), (float(x_obs[1]))), 90, color ='red')
# #     padding_plt = plt.Circle(((float(center[0])), (float(center[1]))), radius + 90, color='red', fill=False)
# #     ax.add_patch( obstacle_plt )
# #     ax.add_patch( padding_plt )

# # plt.show()