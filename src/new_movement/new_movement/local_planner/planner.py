from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from new_movement.entities.Trajectory import Trajectory
from new_movement.entities.States import State, Vector2D, MoveConstraints
from new_movement.entities.obstacles import Obstacle
from new_movement.entities.StaticObstacle import StaticObstacle
from new_movement.utilities.trajectory_generator.TrajGenerator import TrajectoryGenerator

from .collision import CollisionEngine
from .solvers import BypassSolver, PlanningStatus
from .sampler import InformedSampler

@dataclass
class SolverConfig:
    """Configuration for planning algorithms."""
    max_iterations: int = 20
    field_length: float = 12000.0
    field_width: float = 9000.0
    # Bounds are magnitudes: MoveConstraints derives min = -max from them. Negative
    # values invert the bounds and break the trapezoidal steer (braking accelerates,
    # the velocity cap is never enforced), so __post_init__ rejects them.
    max_velocity: Vector2D = field(default_factory=lambda: Vector2D(2000.0, 2000.0))  # mm/s
    max_acceleration: Vector2D = field(default_factory=lambda: Vector2D(1500.0, 1500.0)) # mm/s²
    continuity_threshold: float = 1e-3
    # How much faster a newly sampled bypass must be to replace the previous one.
    bypass_cost_margin: float = 0.15
    # Collision sampling step. At 2000 mm/s a 0.2 s step advances 400 mm between
    # samples — wider than an obstacle diameter, so the check can tunnel through it.
    # 0.04 s is ~80 mm of travel.
    collision_time_step: float = 0.04

    def __post_init__(self):
        if self.max_velocity.x <= 0 or self.max_velocity.y <= 0:
            raise ValueError(
                f"max_velocity must be positive on both axes, got {self.max_velocity}"
            )
        if self.max_acceleration.x <= 0 or self.max_acceleration.y <= 0:
            raise ValueError(
                f"max_acceleration must be positive on both axes, got {self.max_acceleration}"
            )

class Planner:
    """High-level Orchestrator for Robot Planning."""

    def __init__(self, config: Optional[SolverConfig] = None):
        self.config = config or SolverConfig()
        self.generator = TrajectoryGenerator(MoveConstraints(self.config.max_velocity, self.config.max_acceleration))
        
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
        start: State,
        goal: State,
        obstacles: List[Obstacle],
        previous_via: Optional[State] = None,
    ) -> Trajectory:
        """
        Primary entry point for calculating a trajectory.

        previous_via is the via point of the last plan for this robot, if any. The caller
        owns that cache so this stays reentrant across the planner's worker threads.
        """
        start, goal, safety_trajectory = self._handle_static_collisions(start, goal, obstacles)


        # 1. Try direct path
        direct_seg = self.generator.generate(start, goal)
        if not CollisionEngine.is_collision(direct_seg, obstacles, self.config.collision_time_step):
            self.status = PlanningStatus.DIRECT_PATH
            safety_trajectory.append(direct_seg)
            return safety_trajectory

        # 2. Try bypass solver
        bypass_traj = self.solver.solve(start, goal, obstacles, self.generator, previous_via)
        if bypass_traj and bypass_traj.root:
            self.status = PlanningStatus.BYPASS_FOUND
            safety_trajectory.append(bypass_traj.root)
            safety_trajectory.via_state = bypass_traj.via_state
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
