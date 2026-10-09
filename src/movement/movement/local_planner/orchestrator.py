
from math import hypot
from typing import List, Optional, Tuple

from movement.entities.trajectory.trajectory import Trajectory
from movement.entities.trajectory.trajectory_segment import TrajectorySegment
from movement.entities.motion.motion_state import MotionState
from movement.entities.motion.motion_constraints import MotionConstraints
from movement.entities.motion.motion_path import MotionPath
from movement.entities.motion.motion_primitive import MotionPrimitive
from movement.entities.obstacle.obstacle import Obstacle
from movement.entities.obstacle.static_obstacle import StaticObstacle
from movement.local_planner.collision_engine import CollisionEngine
from movement.local_planner.informed_sampler import InformedSampler
from movement.local_planner.trajectory_generator import TrajectoryGenerator
from movement.local_planner.solver import BypassSolver, PlanningStatus, SolverConfig

from utils.math_util import Vector2D

# How many times to re-ask every obstacle where a point should go before accepting that
# they cannot agree on one.
MAX_ESCAPE_PASSES = 6
# Search for how far a robot can run toward an occupied goal: tenths of the line first,
# then this many bisections of the tenth where it stops being free.
APPROACH_STEPS = 10
APPROACH_REFINEMENTS = 4


class Orchestrator:
    """High-level Orchestrator for Robot Planning."""

    def __init__(self, config: Optional[SolverConfig] = None):
        self.config = config or SolverConfig()
        self.generator = TrajectoryGenerator(MotionConstraints(self.config.max_velocity, self.config.max_acceleration))
        
        # Initialize Sampler and Solver
        self.sampler = InformedSampler(
            field_length=self.config.field_length,
            field_width=self.config.field_width,
            max_velocity=self.config.max_velocity.x
        )
        self.solver = BypassSolver(
            max_iterations=self.config.max_iterations,
            sampler=self.sampler,
            collision_time_step=self.config.collision_time_step,
            cost_margin=self.config.bypass_cost_margin
        )
        
        self.status = PlanningStatus.FAILED
        # Set when the robot sits inside obstacles with no point that satisfies them all.
        self.escape_failed = False

    def find(
        self,
        start: MotionState,
        goal: MotionState,
        obstacles: List[Obstacle],
        previous_via: Optional[MotionState] = None,
        speed_scale: float = 1.0,
    ) -> Trajectory:
        """
        Primary entry point for calculating a trajectory.

        previous_via is the via point of the last plan for this robot, if any. The caller
        owns that cache so this stays reentrant across the planner's worker threads.

        speed_scale caps this plan's top speed at that fraction of max_velocity. Escaping
        an obstacle and the recovery stop keep the full limits.
        """
        generator, solver, speed_limit = self._limited_to(speed_scale)
        goal = MotionState(goal.position, self._feasible_velocity(goal.velocity, speed_limit))
        if previous_via is not None:
            previous_via = MotionState(
                previous_via.position,
                self._feasible_velocity(previous_via.velocity, speed_limit),
            )
        start, goal, safety_trajectory = self._handle_start_and_goal_collisions(
            start, goal, obstacles
        )

        # Escaping and moving the goal used the obstacles' own edges; a route also keeps
        # whatever room they ask for.
        obstacles = [obs.for_route(start.position, goal.position) for obs in obstacles]

        # 1. Try direct path
        direct_seg = generator.generate(start, goal)
        # An unsolved steer is empty, and an empty path collides with nothing.
        if BypassSolver._reaches(direct_seg, goal) and not CollisionEngine.is_collision(
            direct_seg, obstacles, self.config.collision_time_step
        ):
            self.status = PlanningStatus.DIRECT_PATH
            safety_trajectory.status = PlanningStatus.DIRECT_PATH
            safety_trajectory.append(direct_seg)
            return safety_trajectory

        # 2. Try bypass solver
        bypass_traj = solver.solve(start, goal, obstacles, generator, previous_via)
        if bypass_traj and bypass_traj.root:
            self.status = PlanningStatus.BYPASS_FOUND
            safety_trajectory.status = PlanningStatus.BYPASS_FOUND
            safety_trajectory.append(bypass_traj.root)
            safety_trajectory.via_state = bypass_traj.via_state
            return safety_trajectory

        # 3. A robot is standing on the goal, so nothing can end there. Close in as far
        # as is free instead of stopping wherever we happen to be.
        if self._is_occupied(goal.position, obstacles, direct_seg.get_total_duration()):
            approach = self._approach(start, goal, obstacles, generator)
            if approach is not None:
                self.status = PlanningStatus.PARTIAL
                safety_trajectory.status = PlanningStatus.PARTIAL
                safety_trajectory.append(approach)
                return safety_trajectory

        # 4. Recovery fallback
        self.status = PlanningStatus.RECOVERY
        recovery = self._get_recovery_trajectory(start)
        recovery.status = PlanningStatus.RECOVERY
        return recovery

    def _limited_to(self, speed_scale: float):
        """The generator, bypass solver and speed limit for a plan capped at speed_scale."""
        if not 0.0 < speed_scale < 1.0:
            return self.generator, self.solver, self.config.max_velocity

        limit = self.config.max_velocity.multiplyByScalar(speed_scale)
        generator = TrajectoryGenerator(MotionConstraints(limit, self.config.max_acceleration))
        # Its own sampler, or the via points are sampled at speeds the cap forbids.
        solver = BypassSolver(
            max_iterations=self.config.max_iterations,
            sampler=InformedSampler(
                field_length=self.config.field_length,
                field_width=self.config.field_width,
                max_velocity=limit.x,
            ),
            collision_time_step=self.config.collision_time_step,
            cost_margin=self.config.bypass_cost_margin,
        )
        return generator, solver, limit

    def _feasible_velocity(self, velocity: Vector2D, limit: Optional[Vector2D] = None) -> Vector2D:
        """
        Scale a velocity down until its speed fits the limit, keeping its heading.

        The steering solver cannot end above the limit and returns an empty plan.
        """
        limit = limit or self.config.max_velocity
        excess = hypot(velocity.x / limit.x, velocity.y / limit.y)
        if excess <= 1.0:
            return velocity
        return Vector2D(velocity.x / excess, velocity.y / excess)

    def _collides_at(self, obs: Obstacle, position: Vector2D, t: float = 0.0) -> bool:
        """Whether this obstacle occupies a point at time t, static or dynamic alike."""
        if isinstance(obs, StaticObstacle):
            return obs.isCollidingAt(position)
        return obs.isCollidingAt(position, t)

    def _is_occupied(self, position: Vector2D, obstacles: List[Obstacle], t: float) -> bool:
        return any(self._collides_at(obs, position, t) for obs in obstacles)

    def _approach(
        self,
        start: MotionState,
        goal: MotionState,
        obstacles: List[Obstacle],
        generator: TrajectoryGenerator,
    ) -> Optional[TrajectorySegment]:
        """
        The longest clear run along the straight line to the goal, ending at rest.

        Coarse steps find roughly where the line stops being free and bisection
        tightens it; the run then ends approach_clearance short of that point.
        """
        span = goal.position.subtract(start.position)
        length = span.size()
        if length < 1e-6:
            return None

        def run_to(fraction: float) -> Optional[TrajectorySegment]:
            target = MotionState(
                start.position.add(span.multiplyByScalar(fraction)), Vector2D(0.0, 0.0)
            )
            segment = generator.generate(start, target)
            if not BypassSolver._reaches(segment, target):
                return None
            if CollisionEngine.is_collision(segment, obstacles, self.config.collision_time_step):
                return None
            return segment

        clear, blocked = None, 1.0
        for step in range(APPROACH_STEPS - 1, 0, -1):
            fraction = step / APPROACH_STEPS
            if run_to(fraction) is not None:
                clear = fraction
                break
            blocked = fraction
        if clear is None:
            return None

        for _ in range(APPROACH_REFINEMENTS):
            middle = 0.5 * (clear + blocked)
            if run_to(middle) is not None:
                clear = middle
            else:
                blocked = middle

        fraction = clear - self.config.approach_clearance / length
        if fraction <= 0.0:
            # Already as close as it gets; recovery holds the robot where it is.
            return None
        return run_to(fraction)

    def _push_clear(self, obs: Obstacle, position: Vector2D) -> Vector2D:
        """
        Where one obstacle wants a point moved to, a margin past its boundary.

        adaptDestination returns the boundary itself, where isCollidingAt is still true,
        so landing exactly on it leaves the next plan starting in a collision.
        """
        if isinstance(obs, StaticObstacle):
            boundary = obs.adaptDestination(position)
        else:
            boundary = obs.adaptDestination(position, 0.0)

        outward = boundary.subtract(position)
        if outward.size() < 1e-6:
            return boundary
        return boundary.add(outward.norm().multiplyByScalar(self.config.escape_margin))

    def _clear_point(
        self, position: Vector2D, obstacles: List[Obstacle]
    ) -> Tuple[Vector2D, bool]:
        """
        Move a point until no obstacle occupies it, or report that none of them agree.

        Escaping one obstacle at a time deadlocks where they overlap: the penalty area
        sends a robot past the field border, and the border sends it straight back. The
        answer only counts once every obstacle accepts it, so this reports failure
        rather than committing to one obstacle's opinion.
        """
        current = position
        for _ in range(MAX_ESCAPE_PASSES):
            blocking = [obs for obs in obstacles if self._collides_at(obs, current)]
            if not blocking:
                return current, True
            for obs in blocking:
                current = self._push_clear(obs, current)

        if any(self._collides_at(obs, current) for obs in obstacles):
            return position, False
        return current, True

    def _handle_start_and_goal_collisions(
        self, start: MotionState, goal: MotionState, obstacles: List[Obstacle]
    ) -> Tuple[MotionState, MotionState, Trajectory]:
        traj = Trajectory()

        # Only static obstacles move the goal: where a robot will be by the time we
        # arrive is a different question from where it is now.
        static = [obs for obs in obstacles if isinstance(obs, StaticObstacle)]
        goal_position, _ = self._clear_point(goal.position, static)
        goal = MotionState(goal_position, goal.velocity)

        # Escaping applies to every obstacle: gated to static ones, a robot pressed
        # against another robot collides at t=0 on every candidate path and stays stuck.
        exit_point, reachable = self._clear_point(start.position, obstacles)
        if not reachable:
            # Nowhere satisfies every obstacle at once, so stay put and let the
            # collision check fall through to a stop.
            self.escape_failed = True
            return start, goal, traj

        self.escape_failed = False
        if exit_point is not start.position and not exit_point.distance(start.position) < 1e-9:
            exit_state = MotionState(exit_point, start.velocity)
            escape = self.generator.generate(start, exit_state)
            traj.append(escape)
            # Carry on from where the escape actually ended, not from where it was
            # aimed: a short hop at the robot's current velocity is often unsolvable and
            # the steering solver returns its nearest attempt instead. Chaining onto the
            # requested state leaves a gap that Trajectory.append rejects.
            start = escape.get_local_destination()

        return start, goal, traj

    def _get_recovery_trajectory(self, current_state: MotionState) -> Trajectory:
        """
        Brake to a stop wherever that lands, rather than back at the position the robot
        held when this was planned — asking a robot at 2000mm/s to end where it already
        is means overshooting and driving back.

        One primitive straight against the velocity, at the acceleration limit.
        """
        velocity = current_state.velocity
        acceleration = self.config.max_acceleration
        stop_time = hypot(velocity.x / acceleration.x, velocity.y / acceleration.y)
        braking = []
        if stop_time > 0.0:
            braking.append(
                MotionPrimitive(
                    Vector2D(-velocity.x / stop_time, -velocity.y / stop_time), stop_time
                )
            )
        return Trajectory(
            TrajectorySegment(current_state.position, current_state.velocity, MotionPath(braking))
        )

    def validate_continuity(self, trajectory: Trajectory) -> bool:
        if not trajectory.root:
            return True
        current = trajectory.root
        while current and current.child:
            dest = current.get_local_destination()
            child_start = current.child.initial_state
            if dest.position.distance(child_start.position) > self.config.continuity_threshold or \
               dest.velocity.distance(child_start.velocity) > self.config.continuity_threshold:
                return False
            current = current.child
        return True
