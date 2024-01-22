from math import atan2
from utils.linalg import Vec2D

class Repulsive:

    def __init__(self):
        self.origin = Vec2D.origin()

    def update_origin(self, newOrigin: Vec2D) -> None:
        self.origin = newOrigin.copy()

    def fi_r(self,  _p,  _origin: Vec2D = None,  _theta: bool = True):
        '''
        REFACTOR
        '''
        if _origin is not None:
            self.update_origin(_origin)

        p = _p - self.origin

        if _theta:
            return atan2(p[1], p[0])
        else:
            return p