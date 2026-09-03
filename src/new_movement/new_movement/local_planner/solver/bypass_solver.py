from typing import List, Optional

from new_movement.entities.trajectory.trajectory import Trajectory
from new_movement.entities.trajectory.trajectory_segment import TrajectorySegment
from new_movement.entities.motion.motion_state import MotionState
from new_movement.entities.obstacle.obstacle import Obstacle
from new_movement.local_planner.collision_engine import CollisionEngine
from new_movement.local_planner.informed_sampler import InformedSampler
from new_movement.local_planner.trajectory_generator import TrajectoryGenerator
from new_movement.local_planner.solver import BaseSolver

# Perpendicular sampling sigma, as a fraction of the start-goal distance.
MIN_SPREAD = 0.03
MAX_SPREAD = 0.35

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
        start: MotionState,
        goal: MotionState,
        obstacles: List[Obstacle],
        generator: TrajectoryGenerator,
        previous_via: Optional[MotionState] = None,
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
            via_state = MotionState(
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
        start: MotionState,
        goal: MotionState,
        obstacles: List[Obstacle],
        generator: TrajectoryGenerator,
        via_state: MotionState,
    ) -> Optional[Trajectory]:
        """Two segments through via_state, or None if either one collides."""
        segment_1 = generator.generate(start, via_state)
        segment_2 = generator.generate(via_state, goal)

        # The steering solver cannot produce a profile for every pair of states, and
        # when it gives up it returns a zero-duration path parked at its own start.
        # Chaining that raises out of add_child, and the exception travels all the way
        # up to the planner node, which logs it and leaves the robot with no plan for
        # the cycle. A via point we cannot actually steer to is just a candidate that
        # did not work out, so drop it and let the search try the next sample.
        if not self._reaches(segment_1, via_state):
            return None

        if not self._is_safe(segment_1, obstacles) or not self._is_safe(segment_2, obstacles):
            return None

        segment_1.add_child(segment_2)
        trajectory = Trajectory(segment_1)
        trajectory.via_state = via_state
        return trajectory

    @staticmethod
    def _reaches(segment: TrajectorySegment, target: MotionState, tolerance: float = 1e-3) -> bool:
        """
        Whether a generated segment actually ends on the state it was asked for.

        Deliberately the same comparison add_child makes, so a segment that passes here
        is one it will accept.
        """
        destination = segment.get_local_destination()
        return (
            destination.position.distance(target.position) < tolerance
            and destination.velocity.distance(target.velocity) < tolerance
        )

    def _is_safe(self, segment: TrajectorySegment, obstacles: List[Obstacle]) -> bool:
        return not CollisionEngine.is_collision(
            segment, obstacles, self.collision_time_step
        )
