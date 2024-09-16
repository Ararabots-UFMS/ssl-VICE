from abc import ABC, abstractmethod

import numpy as np


class Obstacle(ABC):
    
    @abstractmethod
    def is_colission(self, x: np.matrix, ignore: bool, padding: float = 90) -> bool:
        ''' Method to check collisions '''
        pass

class StaticObstacle(Obstacle):

    @abstractmethod
    def closest_outside_point(self, x: np.matrix, offset: float) -> np.array:
        ''' Return the closest point outside the obstacle '''
        pass
    
class DynamicObstacle(Obstacle):

    @property
    @abstractmethod
    def mode(self):
        pass
    
    @abstractmethod
    def set_mode(self, mode: str = 'Default') -> None:
        ''' Set behaviour mode for movimentation '''
        pass

    @abstractmethod
    def get_dynamic_range(self, delta: float) -> np.matrix:
        ''' Returns the center position and radius of dinamic range '''
        pass
    
    @abstractmethod
    def update_state(self, x: np.matrix) -> None:
        pass