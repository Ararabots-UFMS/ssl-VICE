from movement.path.path_profiles import PathProfile
from ruckig import InputParameter, OutputParameter, Result, Ruckig, Trajectory

from typing import List, Optional, Tuple

class PathGenerator():
    def __init__(self, constrainsts: Tuple[List[float]] = ([100, 100, 1], [100, 100, 0.1])):
        self.vel_constrainst = constrainsts[0]
        self.acc_constrainst = constrainsts[1]

    def generate_input(self, init_state: Tuple[List[float]], profile: PathProfile, **kwargs):
        inp = InputParameter(3)

        inp.current_position = init_state[0]
        inp.current_velocity = init_state[1]

        inp.max_velocity = self.vel_constrainst
        inp.max_acceleration = self.acc_constrainst

        kwargs['inp'] = inp

        profile.generate(**kwargs)

        return inp

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