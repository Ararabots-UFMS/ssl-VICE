from abc import abstractmethod
import numpy as np

from new_movement.entities.obstacle.obstacle import Obstacle


class StaticObstacle(Obstacle):
    def velocity(self) -> float:
        return 0.0

    def batch_collides(self, positions: np.ndarray, times: np.ndarray) -> bool:
        """Static obstacles don't change over time — times is ignored."""
        return self._check_positions(positions)

    def batch_collides_segments(
        self,
        starts: np.ndarray,
        ends: np.ndarray,
        t_starts: np.ndarray,
        t_ends: np.ndarray,
    ) -> bool:
        """Static obstacles don't change over time — the timestamps are ignored."""
        return self._check_segments(starts, ends)

    def _check_segments(self, starts: np.ndarray, ends: np.ndarray) -> bool:
        """
        Does any segment starts[i] -> ends[i] touch this obstacle?

        The default subdivides and reuses the point test; override with a closed-form
        sweep where the shape allows one.
        """
        divisions = self._FALLBACK_SUBDIVISIONS
        delta = ends - starts
        for i in range(divisions + 1):
            if self._check_positions(starts + (i / divisions) * delta):
                return True
        return False

    @abstractmethod
    def _check_positions(self, positions: np.ndarray) -> bool:
        raise NotImplementedError