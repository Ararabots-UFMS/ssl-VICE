from movement.path.path_profiles import PathProfile
from movement.path.acceptor import PathAcceptor
from movement.obstacles.interfaces import Obstacle
from ruckig import InputParameter, OutputParameter, Result, Ruckig, Trajectory

from copy import copy
from typing import List, Optional, Tuple

class PathGenerator():
    def __init__(self, constrainsts: Tuple[List[float]]):

        self.Dofs = 3 # Degrees of freedom (x, y, theta)
        self.config = Ruckig(self.Dofs)
        self.inp = InputParameter(self.Dofs)

        self.inp.max_velocity = constrainsts[0]
        self.inp.max_acceleration = constrainsts[1]

    def generate(self, init_state: Tuple[List[float]],
                       goal_state: Optional[Tuple[List[float]]],
                       profile: PathProfile,
                       obstacles: List[Obstacle],
                       **kwargs) -> Trajectory:
        inp = self.inp

        path = Trajectory(self.Dofs)

        inp.current_position = init_state[0] # [pos_x, pox_y, orientation]
        inp.current_velocity = init_state[1] # [vel_x, vel_y, vel_angular]

        path = profile.generate(self.config, inp, goal_state, path, **kwargs)

        return path

class PathExecutor():
    def __init__(self):
        pass

    def execute(self, path: OutputParameter):
        pass