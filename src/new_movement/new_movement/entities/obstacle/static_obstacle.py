from abc import abstractmethod
import numpy as np

from new_movement.entities.obstacle.obstacle import Obstacle


class StaticObstacle(Obstacle):
    def velocity(self) -> float:
        return 0.0

    def batch_collides(self, positions: np.ndarray, times: np.ndarray) -> bool:
        """Static obstacles don't change over time — times is ignored."""
        return self._check_positions(positions)

    @abstractmethod
    def _check_positions(self, positions: np.ndarray) -> bool:
        raise NotImplementedError