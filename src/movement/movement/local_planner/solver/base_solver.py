from typing import List, Optional

from movement.entities.trajectory.trajectory import Trajectory
from movement.entities.motion.motion_state import MotionState
from movement.entities.obstacle.obstacle import Obstacle
from movement.local_planner.trajectory_generator import TrajectoryGenerator

from abc import ABC, abstractmethod


class BaseSolver(ABC):
    @abstractmethod
    def solve(self, start: MotionState, goal: MotionState, obstacles: List[Obstacle], generator: TrajectoryGenerator) -> Optional[Trajectory]:
        raise NotImplementedError
