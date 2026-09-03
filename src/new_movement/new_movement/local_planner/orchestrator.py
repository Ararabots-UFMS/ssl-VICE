
from typing import List, Optional, Tuple

from new_movement.entities.trajectory.trajectory import Trajectory
from new_movement.entities.motion.motion_state import MotionState
from new_movement.entities.motion.motion_constraints import MotionConstraints
from new_movement.entities.obstacle.obstacle import Obstacle
from new_movement.entities.obstacle.static_obstacle import StaticObstacle
from new_movement.local_planner.collision_engine import CollisionEngine
from new_movement.local_planner.informed_sampler import InformedSampler
from new_movement.local_planner.trajectory_generator import TrajectoryGenerator
from new_movement.local_planner.solver import BypassSolver, PlanningStatus, SolverConfig

from utils.math_util import Vector2D

# How many times to re-ask every obstacle where a point should go before accepting that
# they cannot agree on one.
MAX_ESCAPE_PASSES = 6


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
        # Set when the robot sits inside obstacles with no point that satisfies them
        # all, so the caller can say so rather than reporting a generic planning miss.
        self.escape_failed = False

    def find(
        self,
        start: MotionState,
        goal: MotionState,
        obstacles: List[Obstacle],
        previous_via: Optional[MotionState] = None,
    ) -> Trajectory:
        """
        Primary entry point for calculating a trajectory.

        previous_via is the via point of the last plan for this robot, if any. The caller
        owns that cache so this stays reentrant across the planner's worker threads.
        """
        start, goal, safety_trajectory = self._handle_start_and_goal_collisions(
            start, goal, obstacles
        )


        # 1. Try direct path
        direct_seg = self.generator.generate(start, goal)
        if not CollisionEngine.is_collision(direct_seg, obstacles, self.config.collision_time_step):
            self.status = PlanningStatus.DIRECT_PATH
            safety_trajectory.status = PlanningStatus.DIRECT_PATH
            safety_trajectory.append(direct_seg)
            return safety_trajectory

        # 2. Try bypass solver
        bypass_traj = self.solver.solve(start, goal, obstacles, self.generator, previous_via)
        if bypass_traj and bypass_traj.root:
            self.status = PlanningStatus.BYPASS_FOUND
            safety_trajectory.status = PlanningStatus.BYPASS_FOUND
            safety_trajectory.append(bypass_traj.root)
            safety_trajectory.via_state = bypass_traj.via_state
            return safety_trajectory

        # 3. Recovery fallback
        self.status = PlanningStatus.RECOVERY
        recovery = self._get_recovery_trajectory(start)
        recovery.status = PlanningStatus.RECOVERY
        return recovery

    def _collides_at(self, obs: Obstacle, position: Vector2D) -> bool:
        """Whether this obstacle occupies a point, static or dynamic alike."""
        if isinstance(obs, StaticObstacle):
            return obs.isCollidingAt(position)
        return obs.isCollidingAt(position, 0.0)

    def _push_clear(self, obs: Obstacle, position: Vector2D) -> Vector2D:
        """
        Where one obstacle wants a point moved to, a margin past its boundary.

        adaptDestination returns the boundary itself, and isCollidingAt is still true
        there (a distance of zero fails its <= 0 test), so landing exactly on it leaves
        the next plan starting in a collision.
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

        Obstacles used to be escaped one at a time, each one asked in turn where to go
        and the answer taken without checking it against the others. Overlapping
        obstacles then contradict each other: a robot inside the right penalty area is
        sent to its goal-line edge, which is past the field border, and the border sends
        it straight back inside the penalty area. That loop is stable, and its outward
        half is what drove the robot into the wall.

        Escaping is only worth doing if it lands somewhere every obstacle accepts, so
        this reports failure rather than committing to one obstacle's opinion.
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

        # Escaping applies to every obstacle. Gated to static ones, a robot pressed
        # against another robot had every candidate path collide at t=0, so the solver
        # fell through to a stop and it stayed stuck there.
        exit_point, reachable = self._clear_point(start.position, obstacles)
        if not reachable:
            # Nowhere satisfies every obstacle at once. Staying put is the safe answer:
            # driving to one obstacle's preferred exit is how the robot ended up past
            # the field border, and the collision check will fall through to a stop.
            self.escape_failed = True
            return start, goal, traj

        self.escape_failed = False
        if exit_point is not start.position and not exit_point.distance(start.position) < 1e-9:
            exit_state = MotionState(exit_point, start.velocity)
            escape = self.generator.generate(start, exit_state)
            traj.append(escape)
            # Carry on from where the escape actually ended, not from where it was
            # aimed. Reaching a point a few centimetres away while still carrying the
            # robot's current velocity is often not solvable, and the steering solver
            # returns its nearest attempt. Chaining the next segment onto the requested
            # state instead of the achieved one left a gap of a few centimetres, which
            # Trajectory.append rejected and the planner logged as "Solver error" while
            # dropping the plan for that cycle.
            start = escape.get_local_destination()

        return start, goal, traj

    def _get_recovery_trajectory(self, current_state: MotionState) -> Trajectory:
        """
        Brake to a stop wherever that lands, rather than returning to the position the
        robot held when this was planned. Asking a robot at 2000mm/s to end at its
        current position means overshooting and driving back, which is what the
        positive along-track excursions after every recovery were.
        """
        velocity = current_state.velocity
        acceleration = self.config.max_acceleration
        braking_offset = Vector2D(
            velocity.x * abs(velocity.x) / (2.0 * acceleration.x),
            velocity.y * abs(velocity.y) / (2.0 * acceleration.y),
        )
        stop_state = MotionState(current_state.position.add(braking_offset), Vector2D(0, 0))
        return Trajectory(self.generator.generate(current_state, stop_state))

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
