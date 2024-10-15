from movement.obstacles.interfaces import Obstacle, StaticObstacle

from ruckig import InputParameter, OutputParameter, Result, Ruckig, Trajectory

from typing import List, Tuple
from enum import Enum, auto


class AcceptorStatus(Enum):
    ACCEPTED = auto()
    COLLISION = auto()
    INSIDEAREA = auto()


class PathAcceptor:
    def __init__(self):
        pass

    def check(
        self,
        trajectory: Trajectory,
        obstacles: List[Obstacle],
        control_cycle: float = 0.01,
        max_lookahead: float = 3,
    ) -> Tuple[AcceptorStatus, Obstacle]:
        duration = trajectory.duration

        step_size = duration / control_cycle

        if duration < max_lookahead:
            max_lookahead = duration

        # TODO TIGERs considers only the closest obstacle only, maybe it's a good idea
        # Stepping over the path and check for colissions
        current_time = 0.0
        while current_time < max_lookahead:
            position, velocity, acceleration = trajectory.at_time(current_time)
            for obs in obstacles:
                if obs.is_colission(position):
                    if type(obs) == StaticObstacle:
                        return AcceptorStatus.INSIDEAREA, obs
                    return AcceptorStatus.COLLISION, obs

            # TODO Not using dynamic step size
            current_time += step_size

        return AcceptorStatus.ACCEPTED, None
