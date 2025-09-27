from new_movement.entities.Trajectory import TrajectorySegment, Trajectory
from new_movement.entities.States import State, Vector2D
from new_movement.entities.obstacles import Obstacle
from new_movement.entities.StaticObstacle import StaticObstacle
from new_movement.utilities.trajectory_generator.TrajGenerator import (
    TrajectoryGenerator,
)

from typing import List, Optional
from random import uniform, randint


class CollisionSolver:
    def __init__(self, trys: int = 50, game_watcher=None):
        self.trys = trys
        self.game_watcher = game_watcher

    def is_collision(
        self,
        trajectory: TrajectorySegment,
        obstacles: List[Obstacle],
        time_step: float = 0.02,
    ) -> bool:
        """Checks if a trajectory has collision with any obstacles given, returns True is theres is, otherwise returns False"""
        total_time = 0.0
        duration = trajectory.get_total_duration()
        while total_time <= duration:
            pos = trajectory.get_state(total_time).position
            for obs in obstacles:
                # Only check obstacles close to the trajectory point
                if isinstance(obs, StaticObstacle):
                    if obs.isCollidingAt(pos):
                        return True
                else:  # Dynamic Obstacles
                    if obs.isCollidingAt(pos, total_time):
                        return True

            total_time += time_step
        return False

    def solve(
        self,
        curState: State,
        tarState: State,
        obstacles: List[Obstacle],
        generator: TrajectoryGenerator,
    ) -> Trajectory:
        """Finds a new trajectory without collisions based on a random sampling method similar to RRT"""
        for i in range(self.trys):
            # random point to attempt bypass
            random_point: Vector2D = self.generate_random_point()
            random_velocity: Vector2D = self.generate_random_velocity(
                velocity_constraints=Vector2D(2000, 2000)
            )  # max velocity
            bypassState = State(random_point, random_velocity)

            to_point: TrajectorySegment = generator.generate(curState, bypassState)
            from_point: TrajectorySegment = generator.generate(bypassState, tarState)
            if not self.is_collision(from_point, obstacles) and not self.is_collision(
                to_point, obstacles
            ):
                to_point.add_child(from_point)
                return Trajectory(to_point)

            elif self.is_collision(from_point, obstacles) and not self.is_collision(
                to_point, obstacles
            ):
                random_point: Vector2D = self.generate_random_point()
                random_velocity: Vector2D = self.generate_random_velocity(
                    velocity_constraints=Vector2D(2000, 2000)
                )  # max velocity
                new_bypassState = State(random_point, random_velocity)

                to_second_point: TrajectorySegment = generator.generate(
                    bypassState, new_bypassState
                )
                if self.is_collision(to_second_point, obstacles):
                    continue

                from_point: TrajectorySegment = generator.generate(
                    new_bypassState, tarState
                )

                to_second_point.add_child(from_point)
                to_point.add_child(to_second_point)
                return Trajectory(to_point)
        return Trajectory(None)

    def generate_random_velocity(self, velocity_constraints: Vector2D) -> Vector2D:
        """Generates a random velocity within the max and min velocity"""
        return Vector2D(
            uniform(-velocity_constraints.x, velocity_constraints.x),
            uniform(-velocity_constraints.y, velocity_constraints.y),
        )

    def generate_random_point(self) -> Vector2D:
        """Generates a random point inside game field"""
        if self.game_watcher:
            geometry = self.game_watcher.get_geometry()
            field_length = geometry.field_length
            field_width = geometry.field_width
        else:
            # Default field dimensions if no game_watcher
            field_length = 12000  # 12m
            field_width = 9000  # 9m

        return Vector2D(
            uniform(-field_length / 2, field_length / 2),
            uniform(-field_width / 2, field_width / 2),
        )


class TrajectoryOptimizer:
    def __init__(self, trys: int, early_stop: int = 20):
        self.trys = trys
        self.early_stop = early_stop

    def optimize(
        self,
        trajectory: Trajectory,
        generator: TrajectoryGenerator,
        collisionSolver: CollisionSolver,
        obstacles: List[Obstacle],
        initial_time: float = 0.0,
    ) -> Trajectory:
        """
        Optimizes a trajectory, by random sampling a range in the trajectory and finding a new collision free segment

        How it works:
            - A random mode is chosen, either head, normal or tail
            - if the mode is normal, then 2 points are randomly chosen and a new path is generated from point 1 to point 2
            - if the mode is head or tail, one point is randomly chosen and a new path is generated from the begining
            to the point (in head mode) or the point to the ending (tail mode)
            - a collision check is made, and if no collision is detected, then this new patch is stiched.

        This approach is discribed in the article "Bang Bang Boosting of RRTs"

        """
        total_time = trajectory.get_total_duration()

        if total_time <= 0:
            return trajectory

        early_stop_count = 0
        for _ in range(self.trys):
            if early_stop_count > self.early_stop:
                break

            mode = ["Head", "Normal", "Tail"][randint(0, 2)]

            total_time = trajectory.get_total_duration()
            before_time = total_time

            first_time = (
                uniform(initial_time, total_time)
                if (mode == "Normal" or mode == "Tail")
                else 0.0
            )
            second_time = (
                uniform(first_time, total_time)
                if (mode == "Normal" or mode == "Head")
                else total_time
            )

            firstState = trajectory.get_state(first_time)
            secondState = trajectory.get_state(second_time)

            optimized_segment = generator.generate(firstState, secondState)

            if not collisionSolver.is_collision(optimized_segment, obstacles):
                curSegment = trajectory.root
                curTime = second_time
                while (
                    curSegment.get_local_duration() < curTime
                    and curSegment.child is not None
                ):
                    curTime -= curSegment.get_local_duration()
                    curSegment = curSegment.child

                _, last_motion = curSegment.motion_path.split(curTime)
                last_segment = TrajectorySegment(
                    secondState.position, secondState.velocity, last_motion
                )

                if curSegment.child is not None:
                    last_segment.child = curSegment.child

                optimized_segment.add_child(last_segment)
                trajectory.connect(optimized_segment, first_time)

                if (
                    abs(before_time - trajectory.get_total_duration()) < 0.01
                ):  # if no improvement in path duration
                    early_stop_count += 1

        return trajectory


class Planner:
    """Its responsible for finding a collision free optimal trajectory to target state"""

    def __init__(
        self, bypass_trys: Optional[int] = None, optimizer_trys: Optional[int] = 50
    ):
        self.generator = TrajectoryGenerator()
        self.solver = (
            CollisionSolver()
            if bypass_trys is None
            else CollisionSolver(trys=bypass_trys)
        )
        self.optimizer = TrajectoryOptimizer(optimizer_trys)
        self.optimizer_trys = optimizer_trys

    def find(
        self, curState: State, tarState: State, obstacles: List[Obstacle]
    ) -> TrajectorySegment:
        final_trajectory = Trajectory()
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

        best_trajectory = self.generator.generate(curState, tarState)

        if self.solver.is_collision(best_trajectory, obstacles):
            solved_trajectory = self.solver.solve(
                curState, tarState, obstacles, self.generator
            )
            solved_trajectory = self.optimizer.optimize(
                solved_trajectory, self.generator, self.solver, obstacles
            )
            if solved_trajectory is None or solved_trajectory.root is None:
                return final_trajectory
            best_seg = solved_trajectory.root
        else:
            best_seg = best_trajectory

        final_trajectory.append(best_seg)

        return final_trajectory
