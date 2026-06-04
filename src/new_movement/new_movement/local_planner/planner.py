from dataclasses import dataclass
from typing import List, Optional, Tuple

from new_movement.entities.Trajectory import Trajectory
from new_movement.entities.States import State, Vector2D
from new_movement.entities.obstacles import Obstacle
from new_movement.entities.StaticObstacle import StaticObstacle
from new_movement.utilities.trajectory_generator.TrajGenerator import TrajectoryGenerator

from .collision import CollisionEngine
from .solvers import BypassSolver, PlanningStatus
from .sampler import InformedSampler

@dataclass
class SolverConfig:
    """Configuration for planning algorithms."""
    max_iterations: int = 10
    field_length: float = 12000.0
    field_width: float = 9000.0
    max_velocity: float = 3000.0  # mm/s
    safety_buffer: float = 190.0   # mm
    continuity_threshold: float = 1e-3
    collision_time_step: float = 0.2

class Planner:
    """High-level Orchestrator for Robot Planning."""

    def __init__(self, config: Optional[SolverConfig] = None):
        self.config = config or SolverConfig()
        self.generator = TrajectoryGenerator()
        
        # Initialize Sampler and Solver
        self.sampler = InformedSampler(
            field_length=self.config.field_length,
            field_width=self.config.field_width,
            max_velocity=self.config.max_velocity
        )
        self.solver = BypassSolver(
            max_iterations=self.config.max_iterations,
            sampler=self.sampler,
            collision_time_step=self.config.collision_time_step
        )
        
        self.status = PlanningStatus.FAILED

    def find(self, start: State, goal: State, obstacles: List[Obstacle]) -> Trajectory:
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
        self, start: State, goal: State, obstacles: List[Obstacle]
    ) -> Tuple[State, State, Trajectory]:
        traj = Trajectory()
        for obs in obstacles:
            if not isinstance(obs, StaticObstacle):
                continue
            if obs.isCollidingAt(goal.position):
                goal = State(obs.adaptDestination(goal.position), goal.velocity)
            if obs.isCollidingAt(start.position):
                exit_point = obs.adaptDestination(start.position)
                exit_state = State(exit_point, start.velocity)
                traj.append(self.generator.generate(start, exit_state))
                start = exit_state
        return start, goal, traj

    def _get_recovery_trajectory(self, current_state: State) -> Trajectory:
        stop_state = State(current_state.position, Vector2D(0, 0))
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
