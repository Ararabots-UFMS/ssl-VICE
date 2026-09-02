from typing import List, Optional

from new_movement.entities.trajectory.trajectory import Trajectory
from new_movement.entities.motion.motion_state import MotionState
from new_movement.entities.obstacle.obstacle import Obstacle
from new_movement.local_planner.trajectory_generator import TrajectoryGenerator

from abc import ABC, abstractmethod


class BaseSolver(ABC):
    @abstractmethod
    def solve(self, start: MotionState, goal: MotionState, obstacles: List[Obstacle], generator: TrajectoryGenerator) -> Optional[Trajectory]:
        raise NotImplementedError
