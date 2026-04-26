from typing import List, Optional
from enum import Enum, auto

from new_movement.entities.Trajectory import TrajectorySegment, Trajectory
from new_movement.entities.States import State, Vector2D
from new_movement.entities.obstacles import Obstacle
from new_movement.utilities.trajectory_generator.TrajGenerator import TrajectoryGenerator

# Relative imports
from .collision import CollisionEngine
from .sampler import InformedSampler

class PlanningStatus(Enum):
    SUCCESS = auto()
    DIRECT_PATH = auto()
    BYPASS_FOUND = auto()
    FAILED = auto()
    RECOVERY = auto()

class BaseSolver:
    """Interface for trajectory solvers."""
    def solve(self, start: State, goal: State, obstacles: List[Obstacle], generator: TrajectoryGenerator) -> Optional[Trajectory]:
        raise NotImplementedError

class BypassSolver(BaseSolver):
    """RRT-inspired solver for finding collision-free bypasses."""
    
    def __init__(self, max_iterations: int, sampler: InformedSampler, collision_time_step: float):
        self.max_iterations = max_iterations
        self.sampler = sampler
        self.collision_time_step = collision_time_step

    def solve(
        self, 
        start: State, 
        goal: State, 
        obstacles: List[Obstacle], 
        generator: TrajectoryGenerator
    ) -> Optional[Trajectory]:
        """Iteratively attempts to find a via-point that clears all obstacles."""
        for _ in range(self.max_iterations):
            via_state = State(
                self.sampler.sample_near_axis(start.position, goal.position),
                self.sampler.sample_velocity()
            )

            segment_1 = generator.generate(start, via_state)
            segment_2 = generator.generate(via_state, goal)

            if self._is_safe(segment_1, obstacles) and self._is_safe(segment_2, obstacles):
                segment_1.add_child(segment_2)
                return Trajectory(segment_1)
        return None

    def _is_safe(self, segment: TrajectorySegment, obstacles: List[Obstacle]) -> bool:
        return not CollisionEngine.is_collision(
            segment, obstacles, self.collision_time_step
        )
