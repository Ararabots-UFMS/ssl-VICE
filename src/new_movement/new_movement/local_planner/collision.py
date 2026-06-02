import numpy as np
from typing import List
from new_movement.entities.Trajectory import TrajectorySegment
from new_movement.entities.obstacles import Obstacle
from new_movement.entities.StaticObstacle import StaticObstacle
from new_movement.entities.States import Vector2D

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
        positions = CollisionEngine._sample_positions(trajectory, times) 

        return any(obs.batch_collides(positions, times) for obs in obstacles)

    @staticmethod
    def _sample_positions(trajectory, times):
        states = [trajectory.get_state(t) for t in times]
        return np.array([[s.position.x, s.position.y] for s in states])
