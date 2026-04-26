from typing import List, Optional
from random import uniform, randint
from new_movement.entities.Trajectory import Trajectory, TrajectorySegment
from new_movement.entities.obstacles import Obstacle
from new_movement.utilities.trajectory_generator.TrajGenerator import TrajectoryGenerator
from new_movement.utilities.collision_engine import CollisionEngine

class TrajectoryOptimizer:
    def __init__(self, trys: int = 50, early_stop: int = 20):
        self.trys = trys
        self.early_stop = early_stop

    def optimize(
        self,
        trajectory: Trajectory,
        generator: TrajectoryGenerator,
        obstacles: List[Obstacle],
        initial_time: float = 0.0,
    ) -> Trajectory:
        """
        Optimizes a trajectory by random sampling a range in the trajectory and finding a new collision-free segment.
        """
        total_time = trajectory.get_total_duration()

        if total_time <= 0 or not trajectory.root:
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

            # Use CollisionEngine instead of passing CollisionSolver
            if not CollisionEngine.is_collision(optimized_segment, obstacles):
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
