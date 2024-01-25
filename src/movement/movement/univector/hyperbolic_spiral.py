from math import atan2, sin, cos, sqrt, pi
from utils.linalg import Vec2D
from .utils import wrap2pi
from typing import Optional

class HyperbolicSpiral:
    '''
    Hyperbolic Spiral field class, used to define move2goal field.

    Args:
        Kr [float]: Adjustable parameter.
        radius [float]: Radius that decides the size of the spiral.

    Note* If Kr becomes larger, the spiral becomes smoother.

    Reference: "Evolutionary Univector Field-based Navigation with Collision Avoidance for Mobile Robot"
    '''

    def __init__(self, Kr: float, radius: float):
        self.Kr = Kr
        self.radius = radius

    def update_params(self, Kr: float, radius: float) -> None:
        '''
        Parameters update method.
        '''
        self.Kr = Kr
        self.radius = radius

    def fi_h(self, p: Vec2D, radius: Optional[float] = None, cw: Optional[bool] = True) -> float:
        '''
        Calculate fi_h method.

        Args:
            p [Vec2D]: position.
            radius [float]: Radius that decides the size of the spiral.
            cw [bool]: clockwise if true, otherwise counter clockwise.
        '''
        r = self.radius if radius is None else radius

        theta = atan2(p[1], p[0])
        ro = p.norm() #Distance between center and position p.

        if ro > r:
            a = (pi / 2.0) * (2.0 - (r + self.Kr) / (ro + self.Kr))
        else:
            a = (pi / 2.0) * sqrt(ro / r)

        theta_ = (theta + a) if cw else (theta - a)

        return wrap2pi(theta_)

    def n_h(self, p: Vec2D, radius: Optional[float] = None, cw: bool = True) -> Vec2D:
        '''
        Calculate n_h method.

        Args:
            p [Vec2D]: position.
            radius [float]: Radius that decides the size of the spiral.
            cw [bool]: clockwise if true, otherwise counter clockwise.
        '''
        fi = self.fi_h(p, radius, cw)
        return Vec2D(cos(fi), sin(fi))