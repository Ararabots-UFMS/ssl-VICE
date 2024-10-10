from abc import ABC, abstractmethod

import numpy as np

from typing import List, Tuple
import numpy as np

class Obstacle(ABC):
    
    @abstractmethod
    def is_colission(self, x: Tuple[List[float]], ignore: bool, padding: float = 90) -> bool:
        ''' Method to check collisions '''
        pass

class StaticObstacle(Obstacle):

    @abstractmethod
    def closest_outside_point(self, x: Tuple[List[float]], offset: float) -> np.array:
        ''' Return the closest point outside the obstacle '''
        pass
    
class DynamicObstacle(Obstacle):

    @abstractmethod
    def get_dynamic_range(self, delta: float) -> (np.matrix, float):
        ''' Returns the center position and radius of dinamic range '''
        pass

    @abstractmethod
    def update_state(self, x: Tuple[List[float]]) -> None:
        pass