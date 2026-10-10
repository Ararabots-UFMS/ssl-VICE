from typing import List, Optional

from movement.entities.trajectory.trajectory import Trajectory
from movement.entities.trajectory.trajectory_segment import TrajectorySegment
from movement.entities.motion.motion_state import MotionState
from movement.entities.obstacle.obstacle import Obstacle
from movement.entities.obstacle.penalty_area_obstacle import PenaltyAreaObstacle
from movement.local_planner.collision_engine import CollisionEngine
from movement.local_planner.informed_sampler import InformedSampler
from movement.local_planner.trajectory_generator import TrajectoryGenerator
from movement.local_planner.solver import BaseSolver
from utils.field_util import FieldSide
from utils.math_util import Vector2D

# Perpendicular sampling sigma, as a fraction of the start-goal distance.
MIN_SPREAD = 0.03
MAX_SPREAD = 0.6  # desvio lateral maior: contornar um inimigo no meio do caminho
# The penalty rectangle already includes the robot radius. This extra room keeps
# the planned trajectory off its edge despite tracking and vision noise.
PENALTY_CORNER_CLEARANCE = 50.0

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
        Attempts to find a route that clears all obstacles.

        Penalty areas get fixed corner routes. For other obstacles, the previous
        cycle's via point is re-solved from the current start and defended by
        cost_margin, so unchanged inputs keep returning the same route.
        """
        challenger = None
        direct_segment = generator.generate(start, goal)
        for via_states in self._penalty_corner_routes(
            start, goal, obstacles, direct_segment
        ):
            candidate = self._build_waypoints(
                start, goal, obstacles, generator, via_states
            )
            if candidate is not None and (
                challenger is None
                or candidate.get_total_duration() < challenger.get_total_duration()
            ):
                challenger = candidate

        # A valid corner route is deterministic and stays close to the padded
        # obstacle. Random candidates tend to wander farther, and can choose a
        # different side on successive cycles as the robot approaches the area.
        if challenger is not None:
            return challenger

        incumbent = None
        if previous_via is not None:
            incumbent = self._build(start, goal, obstacles, generator, previous_via)

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

    def _penalty_corner_routes(self, start, goal, obstacles, direct_segment):
        """Try the actual corners of a penalty area before random bypasses.

        A single sampled via point often cannot go around both corners of the
        rectangle. In that case every replan brakes at its edge. One or two
        nearby corner points provide a repeatable route along its boundary.
        """
        for obstacle in obstacles:
            if not isinstance(obstacle, PenaltyAreaObstacle):
                continue
            if not CollisionEngine.is_collision(
                direct_segment, [obstacle], self.collision_time_step
            ):
                continue
            min_x, max_x, min_y, max_y = obstacle._bounds()
            room = PENALTY_CORNER_CLEARANCE
            # The rear edge meets the field border, so only the two corners
            # facing midfield can provide a legal route around this area.
            front_x = max_x + room if obstacle.side is FieldSide.LEFT else min_x - room
            lower = Vector2D(front_x, min_y - room)
            upper = Vector2D(front_x, max_y + room)
            for corner in (lower, upper):
                yield [MotionState(corner, Vector2D(0.0, 0.0))]
            # When crossing from one side to the other, travel along the front
            # edge in the direction of the goal.
            first, second = (
                (lower, upper) if start.position.y < goal.position.y
                else (upper, lower)
            )
            yield [
                MotionState(first, Vector2D(0.0, 0.0)),
                MotionState(second, Vector2D(0.0, 0.0)),
            ]

    def _build_waypoints(self, start, goal, obstacles, generator, waypoints):
        states = [start, *waypoints, goal]
        segments = []
        for current, target in zip(states, states[1:]):
            segment = generator.generate(current, target)
            if not self._reaches(segment, target) or not self._is_safe(segment, obstacles):
                return None
            segments.append(segment)

        for current, following in zip(segments, segments[1:]):
            current.add_child(following)
        trajectory = Trajectory(segments[0])
        trajectory.via_state = waypoints[0]
        return trajectory

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

        # The steering solver returns a zero-duration path parked at its own start when
        # it cannot reach a state, and chaining that raises out of add_child. A via we
        # cannot steer to is just a candidate that did not work out.
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

        Deliberately the same comparison add_child makes.
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
