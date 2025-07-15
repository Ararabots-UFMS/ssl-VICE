from abc import ABC, abstractmethod
from .States import Vector2D

class Obstacle(ABC):
    @abstractmethod
    def distanceTo(self, curPosition: Vector2D) -> float:
        ''' Calculate euclidian distance between a position to the obstacle. '''
        pass

    @abstractmethod
    def isCollidingAt(self, curPosition: Vector2D) -> float:
        ''' Check if a position collides with the obstacles and returns the collision time '''
        pass

    @abstractmethod
    def adaptDestination(self, tarPosition: Vector2D) -> Vector2D:
        ''' 
        If a destination position is in the obstacles, 
        then return a alternative destination outside obstacle,
        '''
        pass

    @abstractmethod
    def velocity(self) -> float:
        ''' Returns the velocity of the obstacle '''
        pass

class StaticObstacle(Obstacle):
    def velocity(self):
        return 0.0