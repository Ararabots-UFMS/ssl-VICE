from new_movement.entities.Trajectory import TrajectorySegment, Trajectory
from new_movement.entities.States import State, Vector2D
from new_movement.entities.obstacles import Obstacle
from new_movement.entities.StaticObstacle import StaticObstacle
from new_movement.utilities.trajectory_generator.TrajGenerator import TrajectoryGenerator
from new_movement.utilities.collision_engine import CollisionEngine

from typing import List, Optional
from random import uniform


class CollisionSolver:
    def __init__(self, trys: int = 50, field_length_default: int = 12000, field_width_default: int = 9000):
        self.trys = trys
        self._field_length_default = field_length_default
        self._field_width_default = field_width_default

    def solve(
        self,
        curState: State,
        tarState: State,
        obstacles: List[Obstacle],
        generator: TrajectoryGenerator,
    ) -> Trajectory:
        """Finds a new trajectory without collisions using random sampling (bypass)."""
        for i in range(self.trys):
            # random point to attempt bypass
            random_point: Vector2D = self.generate_random_point()
            random_velocity: Vector2D = self.generate_random_velocity(
                velocity_constraints=Vector2D(2000, 2000)
            )
            bypassState = State(random_point, random_velocity)

            to_point: TrajectorySegment = generator.generate(curState, bypassState)
            from_point: TrajectorySegment = generator.generate(bypassState, tarState)
            
            if not CollisionEngine.is_collision(from_point, obstacles) and \
               not CollisionEngine.is_collision(to_point, obstacles):
                to_point.add_child(from_point)
                return Trajectory(to_point)

            # Recursive-ish check for a second bypass point if needed
            # (Keeping existing logic but using CollisionEngine)
            elif CollisionEngine.is_collision(from_point, obstacles) and \
                 not CollisionEngine.is_collision(to_point, obstacles):
                
                random_point2: Vector2D = self.generate_random_point()
                random_velocity2: Vector2D = self.generate_random_velocity(
                    velocity_constraints=Vector2D(2000, 2000)
                )
                new_bypassState = State(random_point2, random_velocity2)

                to_second_point: TrajectorySegment = generator.generate(
                    bypassState, new_bypassState
                )
                if CollisionEngine.is_collision(to_second_point, obstacles):
                    continue

                from_point_final: TrajectorySegment = generator.generate(
                    new_bypassState, tarState
                )
                
                if not CollisionEngine.is_collision(from_point_final, obstacles):
                    to_second_point.add_child(from_point_final)
                    to_point.add_child(to_second_point)
                    return Trajectory(to_point)
                    
        return Trajectory(None)

    def generate_random_velocity(self, velocity_constraints: Vector2D) -> Vector2D:
        return Vector2D(
            uniform(-velocity_constraints.x, velocity_constraints.x),
            uniform(-velocity_constraints.y, velocity_constraints.y),
        )

    def generate_random_point(self) -> Vector2D:
        field_length = self._field_length_default
        field_width = self._field_width_default
        return Vector2D(
            uniform(-field_length / 2, field_length / 2),
            uniform(-field_width / 2, field_width / 2),
        )


class Planner:
    """Responsible for finding a collision-free path generator."""

    def __init__(self, bypass_trys: int = 50):
        self.generator = TrajectoryGenerator()
        self.solver = CollisionSolver(trys=bypass_trys)

    def find(
        self, curState: State, tarState: State, obstacles: List[Obstacle]
    ) -> Trajectory:
        final_trajectory = Trajectory()
        
        # 1. Handle starting/ending inside static obstacles (Safety)
        for obs in obstacles:
            if isinstance(obs, StaticObstacle):
                if obs.isCollidingAt(tarState.position):
                    tarState = State(
                        obs.adaptDestination(tarState.position), tarState.velocity
                    )
                if obs.isCollidingAt(curState.position):
                    out_obs = State(
                        obs.adaptDestination(curState.position), curState.velocity
                    )
                    final_trajectory.append(self.generator.generate(curState, out_obs))
                    curState = out_obs

        # 2. Try direct path
        best_trajectory_seg = self.generator.generate(curState, tarState)

        # 3. If direct path collides, solve for bypass
        if CollisionEngine.is_collision(best_trajectory_seg, obstacles):
            solved_trajectory = self.solver.solve(
                curState, tarState, obstacles, self.generator
            )
            if solved_trajectory and solved_trajectory.root:
                # We return the solved trajectory (Optimization happens in another node)
                return solved_trajectory
            else:
                # Fallback to empty/direct if solver failed
                return Trajectory(best_trajectory_seg)
        else:
            return Trajectory(best_trajectory_seg)
