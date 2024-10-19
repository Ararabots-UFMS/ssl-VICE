from movement.path.path_profiles import PathProfile, OrientationProfile
from ruckig import InputParameter, OutputParameter, Result, Ruckig, Trajectory

from typing import List, Optional, Tuple


class PathGenerator:
    def __init__(
        self,
        constrainsts: Tuple[List[float]] = ([3000, 3000, 3], [2000, 2000, 1]),
    ):
        self.vel_constrainst = constrainsts[0]
        self.acc_constrainst = constrainsts[1]

    def generate_input(
        self, init_state: Tuple[List[float]], path_profile: PathProfile, orientation_profile: OrientationProfile,  **kwargs
    ) -> Tuple[InputParameter, InputParameter]:
        
        # **kwargs need to be two different dicts of parameters. In this case, {path_kwargs} and {orientation_kwargs} inside kwargs.
        inp_path = InputParameter(2)
        inp_orientation = InputParameter(1)

        inp_path.current_position = init_state[0][:1]
        inp_path.current_velocity = init_state[1][:1]

        inp_orientation.current_position = init_state[0][2]
        inp_orientation.current_velocity = init_state[1][2]

        inp_path.max_velocity = self.vel_constrainst[:1]
        inp_path.max_acceleration = self.acc_constrainst[:1]

        kwargs["path_kwargs"]["inp"] = inp_path
        kwargs["orientation_kwargs"]["inp"] = inp_orientation

        path_profile.generate(kwargs["path_kwargs"])
        orientation_profile.generate(kwargs["orientation_kwargs"])

        return inp_path, inp_orientation


# from movement.path.path_profiles import *
# import matplotlib.pyplot as plt
# from math import pi

# gen = PathGenerator()
# inp = gen.generate_input(([0,0,0],[0,0,0]), BypassProfile)

# otg = Ruckig(3)
# trajectory = Trajectory(3)

# result = otg.calculate(inp, trajectory)
# print(f'Trajectory duration: {trajectory.duration:0.4f} [s]')

# x_pos = []
# y_pos = []

# time_unit = 0.01
# current_time = 0.0
# while current_time < 10:
#     pos, vel, acc = trajectory.at_time(current_time)

#     x_pos.append(pos[0])
#     y_pos.append(pos[1])

#     current_time += time_unit

# ax = plt.gca()
# ax.set_xlim([-4500, 4500])
# ax.set_ylim([-3000, 3000])


# plt.plot(x_pos, y_pos)
# plt.show()
