from movement.obstacles.interfaces import Obstacle, StaticObstacle

from ruckig import InputParameter, OutputParameter, Result, Ruckig, Trajectory

from typing import List
from enum import Enum, auto


class PathStatus(Enum):
    ACCEPTED   = auto()
    COLISSION  = auto()
    INSIDEAREA = auto()

class PathAcceptor():
    def __init__(self, max_time: float, default_step: float = 0.1):
        self.max_time = max_time
        self.default_step = default_step

    def check(self, path: Trajectory, obstacles: List[Obstacle]):
        duration = path.duration
        
        current_time = 0.0
        # TODO Find closest obstacle and consider only him.
        
        while current_time < duration and current_time < self.max_time:
            current_position, _, _ = path.at_time(current_time)

            for obs in obstacles:
                if obs.is_colission(current_position):
                    if current_time == 0.0 and type(obs) == StaticObstacle:
                        return PathStatus.INSIDEAREA
                    else:
                        return PathStatus.COLISSION

                obs_distance = obs.distance(current_position)
                if obs_distance < min_distance:
                    min_distance = obs_distance

            current_time += dynamic_step(min_distance, duration)

        return PathStatus.ACCEPTED

    def dynamic_step(self, min_distance: float, duration: float):
        # TODO the exact formulation of the dynamic step over path is not presented in TIGERs ETDP 2024,
        # so we'll use the same concept but with out own formulation
        return self.default_step + self.default_step * min_distance