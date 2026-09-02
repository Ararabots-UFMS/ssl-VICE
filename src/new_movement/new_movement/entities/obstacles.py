from abc import ABC, abstractmethod
from new_movement.entities.States import Vector2D
import numpy as np


class Obstacle(ABC):
    @abstractmethod
    def distanceTo(self, curPosition: Vector2D) -> float:
        """Calculate shortest euclidian distance between a position to the obstacle."""
        pass

    @abstractmethod
    def isCollidingAt(self, curPosition: Vector2D) -> bool:
        """Check if a position collides with the obstacles and returns the if collision"""
        pass

    def batch_collides(self, positions: np.ndarray, times: np.ndarray) -> bool:
        """
        Default fallback: Python loop over the base isCollidingAt.
        Override in subclasses for vectorized performance.
        """
        for i, t in enumerate(times):
            from new_movement.entities.States import Vector2D
            if self.isCollidingAt(Vector2D(float(positions[i, 0]), float(positions[i, 1])), t):
                return True
        return False

    @abstractmethod
    def adaptDestination(self, tarPosition: Vector2D) -> Vector2D:
        """
        If a destination position is in the obstacles,
        then return a alternative destination outside obstacle,
        """
        pass

    @property
    @abstractmethod
    def velocity(self) -> float:
        """Returns the velocity of the obstacle"""
        pass
