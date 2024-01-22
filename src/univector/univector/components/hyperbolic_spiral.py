from math import atan2, sin, cos, sqrt, pi
from utils.linalg import Vec2D
from .utils import wrap2pi

class HyperbolicSpiral:

    def __init__(self, _Kr, _radius):
        self.Kr = _Kr
        self.radius = _radius

    def update_params(self, _KR: float, _RADIUS: float) -> None:
        '''
        This is a method
        '''
        #Maybe allow to change only one parameter
        self.Kr = _KR
        self.radius = _RADIUS

    def fi_h(self, _p: Vec2D, radius: float = None, cw: bool = True) -> float:

        if radius is None:
            r = self.radius
        else:
            r = radius

        p = _p
        theta = atan2(p[1], p[0])
        ro = p.norm()

        if ro > r:
            a = (pi / 2.0) * (2.0 - (r + self.Kr) / (ro + self.Kr))
        else:
            a = (pi / 2.0) * sqrt(ro / r)

        if cw:
            _theta = wrap2pi(theta + a)
        else:
            _theta = wrap2pi(theta - a)

        #atan2 is unecessary, just return _theta
        return atan2(sin(_theta), cos(_theta))

    def n_h(self, _p: Vec2D, _radius: float = None, cw: bool = True) -> Vec2D:
        #### Unecesssary code, _radius already on fi_h####
        p = _p

        if _radius is None:
            radius = self.radius
        else:
            radius = _radius
        ##########################

        fi = self.fi_h(p, radius, cw)
        return Vec2D(cos(fi), sin(fi))