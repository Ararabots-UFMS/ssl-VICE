import numpy as np
from typing import List

from movement.entities.trajectory.trajectory_segment import TrajectorySegment
from movement.entities.trajectory.trajectory_sampler import TrajectorySampler
from movement.entities.obstacle.obstacle import Obstacle

# Slack on the broad-phase box comparison, in mm. A box and the sweep it stands in for
# are computed by different arithmetic and can disagree in the last bit on a grazing
# path, and a rejection is irreversible, so ties resolve toward running the sweep.
BROAD_PHASE_MARGIN = 1.0e-6


class CollisionEngine:
    @staticmethod
    def is_collision(
        trajectory: TrajectorySegment,
        obstacles: List[Obstacle],
        time_step: float = 0.04,
    ) -> bool:
        """
        Swept collision check against the sampled trajectory.

        The trajectory is reduced to a polyline and obstacles are asked about the
        connecting segments, time_step therefore sets how closely the polyline follows the real curve.
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

        # Broad phase: most obstacles on a full field are nowhere near a given path,
        # and two box comparisons reject them for a fraction of the swept test.
        path_min_x, path_min_y = positions.min(axis=0)
        path_max_x, path_max_y = positions.max(axis=0)

        for obs in obstacles:
            box = obs.bounds()
            if box is not None:
                min_x, min_y, max_x, max_y = box
                if (
                    max_x < path_min_x - BROAD_PHASE_MARGIN
                    or min_x > path_max_x + BROAD_PHASE_MARGIN
                    or max_y < path_min_y - BROAD_PHASE_MARGIN
                    or min_y > path_max_y + BROAD_PHASE_MARGIN
                ):
                    continue
            if obs.batch_collides_segments(starts, ends, t_starts, t_ends):
                return True
        return False
