from movement.obstacles.interfaces import DynamicObstacle
from system_interfaces.msg import Robots

from typing import Tuple

from math import sqrt, copysign

from ruckig import InputParameter, OutputParameter, Result, Ruckig, Trajectory, ControlInterface


class RobotObstacle(DynamicObstacle):
    def __init__(self, state: Robots, radius: float = 90, max_delta: float = 0.5, max_acc = 1, max_vel = 3):
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

        velocity_mag = sqrt(self.state.velocity_x ** 2 + self.state.velocity_y ** 2)
        # In case the obstacle is stationary, adding a small number to avoid zero division.
        velocity_mag = 0.0001 if velocity_mag == 0 else velocity_mag

        velocity_norm = self.state.velocity_x / velocity_mag, self.state.velocity_y / velocity_mag


        f_plus, f_minus = self.bb_range(delta, velocity_norm)


        # Distance between f_minus and f_plus...
        dynamic_radius = sqrt((f_plus[0] - f_minus[0]) ** 2 + (f_plus[1] - f_minus[1]) ** 2)

        dynamic_center = self.state.position_x + (velocity_norm[0] * (f_minus[0] + dynamic_radius)), self.state.position_y + (velocity_norm[1] * (f_minus[1] + dynamic_radius))

        obs_radius = self.radius + dynamic_radius

        return dynamic_center, obs_radius
        
    def bb_range(self, delta, velocity_norm: Tuple[float, float]) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        # Get f- and f+ 1d bang bang trajectorys given the delta time.

        # F plus
        inp = InputParameter(2)
        inp.control_interface = ControlInterface.Velocity

        inp.max_velocity = [self.max_vel, self.max_vel]
        inp.max_acceleration = [self.max_acc, self.max_acc]

        inp.current_position = [self.state.position_x, self.state.position_y]
        inp.current_velocity = [self.state.velocity_x, self.state.velocity_y]

        inp.target_velocity = [copysign(self.max_vel, self.state.velocity_x), copysign(self.max_vel, self.state.velocity_y)]
        inp.target_acceleration = [copysign(self.max_acc, self.state.velocity_x), copysign(self.max_acc, self.state.velocity_y)]

        # F minus
        inp2 = InputParameter(2)
        inp2.control_interface = ControlInterface.Velocity

        inp2.max_velocity = [self.max_vel, self.max_vel]
        inp2.max_acceleration = [self.max_acc, self.max_acc]

        inp2.current_position = [self.state.position_x, self.state.position_y]
        inp2.current_velocity = [self.state.velocity_x, self.state.velocity_y]

        inp2.target_velocity = [copysign(self.max_vel, -1 * self.state.velocity_x), copysign(self.max_vel, -1 * self.state.velocity_y)]
        inp2.target_acceleration = [copysign(self.max_acc, -1 * self.state.velocity_x), copysign(self.max_acc, -1 * self.state.velocity_y)]

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


# import matplotlib.pyplot as plt

# fig, ax = plt.subplots()

# plt.axis([-4500, 4500, -3000, 3000])

# robot = Robots()

# robot.id = 0
# robot.position_x = 0.0
# robot.position_y = 0.0
# robot.velocity_x = 200.0
# robot.velocity_y = -200.0

# deltas = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]

# obs = RobotObstacle(robot, max_delta = 0.5, max_acc = 1000, max_vel = 3000)

# for i in deltas:
#     center, radius = obs.get_dynamic_range(i)

#     obstacle_plt = plt.Circle((robot.position_x, robot.position_y), 90, color ='red')
#     padding_plt = plt.Circle((center), radius, color='red', fill=False)
#     ax.add_patch( obstacle_plt )
#     ax.add_patch( padding_plt )

# plt.show()