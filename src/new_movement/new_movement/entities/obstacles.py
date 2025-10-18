from abc import ABC, abstractmethod
from dataclasses import dataclass
from new_movement.entities.States import Vector2D


@dataclass
class ObstaclePriority:
    HIGHEST = 40
    HIGH = 30
    MEDIUM = 20
    LOW = 10
    LOWEST = 0


class Obstacle(ABC):
    @abstractmethod
    def distanceTo(self, curPosition: Vector2D) -> float:
        """Calculate shortest euclidian distance between a position to the obstacle."""
        pass

    @abstractmethod
    def isCollidingAt(self, curPosition: Vector2D) -> bool:
        """Check if a position collides with the obstacles and returns the if collision"""
        pass

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

    @property
    @abstractmethod
    def getPriority(self) -> ObstaclePriority:
        """Get obstacles priority"""
        pass
