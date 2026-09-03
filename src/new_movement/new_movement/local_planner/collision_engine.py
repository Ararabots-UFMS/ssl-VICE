import numpy as np
from typing import List

from new_movement.entities.trajectory.trajectory_segment import TrajectorySegment
from new_movement.entities.trajectory.trajectory_sampler import TrajectorySampler
from new_movement.entities.obstacle.obstacle import Obstacle


class CollisionEngine:
    @staticmethod
    def is_collision(
        trajectory: TrajectorySegment,
        obstacles: List[Obstacle],
        time_step: float = 0.04,
    ) -> bool:
        """
        Swept collision check against the sampled trajectory.

        The trajectory is reduced to a polyline and every obstacle is asked about the
        connecting segments rather than the sample points. Checking points alone lets a
        path clip an obstacle between two samples, which cost ~5% of grazing collisions
        at the step sizes this planner uses, and could only be improved by paying for a
        finer step.

        time_step now controls how closely the polyline follows the real curve rather
        than how far the robot may travel between samples. The remaining error is the
        chord-to-parabola gap, acceleration * dt^2 / 8. Loosening it past ~0.04 s is not
        worth it: measured cost per check is flat in the step count (the fixed numpy and
        flattening overhead dominates), while the chord error starts producing misses
        and false alarms in both directions.
        """
        sampler = TrajectorySampler(trajectory)
        duration = sampler.duration
        if duration <= 0:
            return False

        # Always land exactly on the end of the trajectory: a final partial step would
        # otherwise leave the last stretch of the path unchecked.
        steps = max(1, int(np.ceil(duration / time_step)))
        times = np.linspace(0.0, duration, steps + 1)
        positions = sampler.positions(times)

        starts, ends = positions[:-1], positions[1:]
        t_starts, t_ends = times[:-1], times[1:]

        return any(
            obs.batch_collides_segments(starts, ends, t_starts, t_ends)
            for obs in obstacles
        )

    @staticmethod
    def _sample_positions(trajectory, times):
        """Positions along ``trajectory`` at ``times``, as an (N, 2) array."""
        return TrajectorySampler(trajectory).positions(np.asarray(times, dtype=float))
