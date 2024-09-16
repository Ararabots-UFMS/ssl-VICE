from abc import ABC, abstractmethod

import numpy as np


class Obstacle(ABC):
    
    @abstractmethod
    def is_colission(self, x: np.matrix, ignore: bool):
        ''' Method to check collisions '''
        pass

class StaticObstacle(Obstacle):

    @abstractmethod
    def closest_outside_point(self, x: np.matrix):
        ''' Return the closest point outside the obstacle '''
        pass
    
class DynamicObstacle(Obstacle):

    @property
    @abstractmethod
    def mode(self):
        pass
    
    @abstractmethod
    def set_mode(self, mode: str = 'Default'):
        ''' Set behaviour mode for movimentation '''
        pass

    @abstractmethod
    def get_dynamic_range(self, delta: float):
        ''' Returns the center position and radius of dinamic range '''
        pass
        