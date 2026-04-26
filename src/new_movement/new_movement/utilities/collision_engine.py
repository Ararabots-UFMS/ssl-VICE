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
        """Checks if a trajectory has collision with any obstacles given."""
        total_time = 0.0
        duration = trajectory.get_total_duration()
        while total_time <= duration:
            pos = trajectory.get_state(total_time).position
            for obs in obstacles:
                # Static obstacles don't care about time
                if isinstance(obs, StaticObstacle):
                    if obs.isCollidingAt(pos):
                        return True
                else:
                    # Dynamic obstacles (Robots, Ball) care about time t
                    if obs.isCollidingAt(pos, total_time):
                        return True

            total_time += time_step
            
        # Also check the exact end point to avoid sampling misses
        final_pos = trajectory.get_destination().position
        for obs in obstacles:
            if isinstance(obs, StaticObstacle):
                if obs.isCollidingAt(final_pos): return True
            else:
                if obs.isCollidingAt(final_pos, duration): return True
                
        return False
