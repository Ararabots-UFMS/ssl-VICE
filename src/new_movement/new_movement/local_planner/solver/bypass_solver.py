from typing import List, Optional

from new_movement.entities.trajectory.trajectory import Trajectory
from new_movement.entities.trajectory.trajectory_segment import TrajectorySegment
from new_movement.entities.motion.motion_state import MotionState
from new_movement.entities.obstacle.obstacle import Obstacle
from new_movement.local_planner.collision_engine import CollisionEngine
from new_movement.local_planner.informed_sampler import InformedSampler
from new_movement.local_planner.trajectory_generator import TrajectoryGenerator
from new_movement.local_planner.solver import BaseSolver


class BypassSolver(BaseSolver):
    """RRT-inspired solver for finding collision-free bypasses."""
    
    def __init__(self, max_iterations: int, sampler: InformedSampler, collision_time_step: float):
        self.max_iterations = max_iterations
        self.sampler = sampler
        self.collision_time_step = collision_time_step

    def solve(
        self, 
        start: MotionState, 
        goal: MotionState, 
        obstacles: List[Obstacle], 
        generator: TrajectoryGenerator
    ) -> Optional[Trajectory]:
        """Iteratively attempts to find a via-point that clears all obstacles."""
        found_trajectories = []
        for _ in range(self.max_iterations):
            via_state = MotionState(
                self.sampler.sample_near_axis(start.position, goal.position),
                self.sampler.sample_velocity()
            )

            segment_1 = generator.generate(start, via_state)
            segment_2 = generator.generate(via_state, goal)

            if self._is_safe(segment_1, obstacles) and self._is_safe(segment_2, obstacles):
                segment_1.add_child(segment_2)
                found_trajectories.append(Trajectory(segment_1))

        if not found_trajectories:
            return None
        
        fastest_trajectory = min(found_trajectories, key=lambda t: t.get_total_duration())
        return fastest_trajectory

    def _is_safe(self, segment: TrajectorySegment, obstacles: List[Obstacle]) -> bool:
        return not CollisionEngine.is_collision(
            segment, obstacles, self.collision_time_step
        )
