import numpy as np
from typing import List
from new_movement.entities.Trajectory import TrajectorySegment
from new_movement.entities.obstacles import Obstacle
from new_movement.entities.StaticObstacle import StaticObstacle

class CollisionEngine:
    @staticmethod
    def is_collision(
        trajectory: TrajectorySegment,
        obstacles: List[Obstacle],
        time_step: float = 0.02,
    ) -> bool:
        """
        Vectorized collision check using NumPy.
        Samples the trajectory and evaluates all points against obstacles.
        """
        duration = trajectory.get_total_duration()
        if duration <= 0:
            return False

        # 1. Sample all time steps at once
        times = np.arange(0, duration + time_step, time_step)
        
        # 2. Get positions for all time steps
        positions = np.array([[trajectory.get_position(t).x, trajectory.get_position(t).y] for t in times])

        for obs in obstacles:
            if isinstance(obs, StaticObstacle):
                for pos_vec in positions:
                    from new_movement.entities.States import Vector2D
                    if obs.isCollidingAt(Vector2D(pos_vec[0], pos_vec[1])):
                        return True
            else:
                # Dynamic Obstacles (Robots)
                for i, t in enumerate(times):
                    from new_movement.entities.States import Vector2D
                    if obs.isCollidingAt(Vector2D(positions[i, 0], positions[i, 1]), t):
                        return True
                
        return False
