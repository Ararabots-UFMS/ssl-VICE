from math import atan2
from typing import Optional, Union
from utils.linalg import Vec2D

class Repulsive:
    '''
    Repulsive field class, used to define avoid-obstacle field.

    Args:
        None.

    Reference: "Evolutionary Univector Field-based Navigation with Collision Avoidance for Mobile Robot"
    '''

    def __init__(self):
        self.origin = Vec2D.origin()

    def update_origin(self, newOrigin: Vec2D) -> None:
        '''
        Parameters update method.
        '''
        self.origin = newOrigin.copy()

    def fi_r(self,  p: Vec2D,  origin: Optional[Vec2D] = None,  theta: bool = True) -> Union[Vec2D, float]:
        '''
        Calculate fi_r method.

        Args:
            p [Vec2D]: position.
            origin Optional[Vec2D]: Field origin. If not None, then update object origin.
            theta [bool]: returns angle if true, otherwise returns Vec2D.
        '''
        if origin is not None:
            self.update_origin(origin)

        p = p - self.origin
        
        return atan2(p[1], p[0]) if theta else p
