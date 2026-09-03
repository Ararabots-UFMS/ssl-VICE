
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

    def _escape_point(self, obs: Obstacle, position: Vector2D) -> Optional[Vector2D]:
        """
        Where to move to get clear of this obstacle, or None if already clear.

        Placed escape_margin beyond the boundary: adaptDestination returns the boundary
        itself, and isCollidingAt is true there (distance zero fails its <= 0 test), so
        escaping exactly onto it leaves the next plan starting in a collision.
        """
        if isinstance(obs, StaticObstacle):
            if not obs.isCollidingAt(position):
                return None
            boundary = obs.adaptDestination(position)
        else:
            if not obs.isCollidingAt(position, 0.0):
                return None
            boundary = obs.adaptDestination(position, 0.0)

        outward = boundary.subtract(position)
        if outward.size() < 1e-6:
            return None  # touching rather than penetrating; nothing to escape
        return boundary.add(outward.norm().multiplyByScalar(self.config.escape_margin))

    def _handle_start_and_goal_collisions(
        self, start: MotionState, goal: MotionState, obstacles: List[Obstacle]
    ) -> Tuple[MotionState, MotionState, Trajectory]:
        traj = Trajectory()
        for obs in obstacles:
            # Only static obstacles move the goal: where a robot will be by the time we
            # arrive is a different question from where it is now.
            if isinstance(obs, StaticObstacle) and obs.isCollidingAt(goal.position):
                goal = MotionState(obs.adaptDestination(goal.position), goal.velocity)

            # Escaping applies to every obstacle. Gated to static ones, a robot pressed
            # against another robot had every candidate path collide at t=0, so the
            # solver fell through to a stop and it stayed stuck there.
            exit_point = self._escape_point(obs, start.position)
            if exit_point is None:
                continue

            exit_state = MotionState(exit_point, start.velocity)
            traj.append(self.generator.generate(start, exit_state))
            start = exit_state
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
