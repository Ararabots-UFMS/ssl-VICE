from typing import List, Optional
from enum import Enum, auto

from new_movement.entities.Trajectory import TrajectorySegment, Trajectory
from new_movement.entities.States import State, Vector2D
from new_movement.entities.obstacles import Obstacle
from new_movement.utilities.trajectory_generator.TrajGenerator import TrajectoryGenerator

from .collision import CollisionEngine
from .sampler import InformedSampler

# Perpendicular sampling sigma, as a fraction of the start-goal distance.
MIN_SPREAD = 0.03
MAX_SPREAD = 0.35


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
    
    def __init__(
        self,
        max_iterations: int,
        sampler: InformedSampler,
        collision_time_step: float,
        cost_margin: float = 0.15,
    ):
        self.max_iterations = max_iterations
        self.sampler = sampler
        self.collision_time_step = collision_time_step
        # How much faster a new bypass has to be before it replaces the previous one.
        self.cost_margin = cost_margin

    def solve(
        self,
        start: State,
        goal: State,
        obstacles: List[Obstacle],
        generator: TrajectoryGenerator,
        previous_via: Optional[State] = None,
    ) -> Optional[Trajectory]:
        """
        Attempts to find a via-point that clears all obstacles.

        The previous cycle's via point is re-solved from the current start and defended
        by cost_margin. Sampling alone returns a structurally different path every cycle
        for unchanged inputs, and the controller cannot track a reference that keeps
        changing its mind about which side of an obstacle to pass.
        """
        incumbent = None
        if previous_via is not None:
            incumbent = self._build(start, goal, obstacles, generator, previous_via)

        challenger = None
        for attempt in range(self.max_iterations):
            # Progressive widening: the tight offsets that keep the route close to the
            # direct line are tried first, and only widen when they keep colliding.
            spread = MIN_SPREAD + (MAX_SPREAD - MIN_SPREAD) * (
                attempt / max(1, self.max_iterations - 1)
            )
            via_position = self.sampler.sample_near_axis(
                start.position, goal.position, spread
            )
            via_state = State(
                via_position,
                self.sampler.sample_tangential_velocity(
                    start.position, via_position, goal.position
                ),
            )

            candidate = self._build(start, goal, obstacles, generator, via_state)
            if candidate is None:
                continue
            if challenger is None or candidate.get_total_duration() < challenger.get_total_duration():
                challenger = candidate

        if incumbent is None:
            return challenger
        if challenger is None:
            return incumbent

        margin = incumbent.get_total_duration() * (1.0 - self.cost_margin)
        return challenger if challenger.get_total_duration() < margin else incumbent

    def _build(
        self,
        start: State,
        goal: State,
        obstacles: List[Obstacle],
        generator: TrajectoryGenerator,
        via_state: State,
    ) -> Optional[Trajectory]:
        """Two segments through via_state, or None if either one collides."""
        segment_1 = generator.generate(start, via_state)
        segment_2 = generator.generate(via_state, goal)

        if not self._is_safe(segment_1, obstacles) or not self._is_safe(segment_2, obstacles):
            return None

        segment_1.add_child(segment_2)
        trajectory = Trajectory(segment_1)
        trajectory.via_state = via_state
        return trajectory

    def _is_safe(self, segment: TrajectorySegment, obstacles: List[Obstacle]) -> bool:
        return not CollisionEngine.is_collision(
            segment, obstacles, self.collision_time_step
        )
