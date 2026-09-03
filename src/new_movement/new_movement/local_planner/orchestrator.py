
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
            collision_time_step=self.config.collision_time_step
        )
        
        self.status = PlanningStatus.FAILED

    def find(self, start: MotionState, goal: MotionState, obstacles: List[Obstacle]) -> Trajectory:
        """Primary entry point for calculating a trajectory."""
        start, goal, safety_trajectory = self._handle_static_collisions(start, goal, obstacles)
        

        # 1. Try direct path
        direct_seg = self.generator.generate(start, goal)
        if not CollisionEngine.is_collision(direct_seg, obstacles, self.config.collision_time_step):
            self.status = PlanningStatus.DIRECT_PATH
            safety_trajectory.append(direct_seg)
            return safety_trajectory

        # 2. Try bypass solver
        bypass_traj = self.solver.solve(start, goal, obstacles, self.generator)
        if bypass_traj and bypass_traj.root:
            self.status = PlanningStatus.BYPASS_FOUND
            safety_trajectory.append(bypass_traj.root)
            return safety_trajectory

        # 3. Recovery fallback
        self.status = PlanningStatus.RECOVERY
        return self._get_recovery_trajectory(start)

    def _handle_static_collisions(
        self, start: MotionState, goal: MotionState, obstacles: List[Obstacle]
    ) -> Tuple[MotionState, MotionState, Trajectory]:
        traj = Trajectory()
        for obs in obstacles:
            if not isinstance(obs, StaticObstacle):
                continue
            if obs.isCollidingAt(goal.position):
                goal = MotionState(obs.adaptDestination(goal.position), goal.velocity)
            if obs.isCollidingAt(start.position):
                exit_point = obs.adaptDestination(start.position)
                exit_state = MotionState(exit_point, start.velocity)
                traj.append(self.generator.generate(start, exit_state))
                start = exit_state
        return start, goal, traj

    def _get_recovery_trajectory(self, current_state: MotionState) -> Trajectory:
        stop_state = MotionState(current_state.position, Vector2D(0, 0))
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
