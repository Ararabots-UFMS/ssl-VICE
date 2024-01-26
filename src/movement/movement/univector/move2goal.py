from math import atan2, cos, sin
from typing import Optional
from utils.linalg import Vec2D, Mat2D
from movement.univector.hyperbolic_spiral import HyperbolicSpiral

class Move2Goal:
    '''
    Move2Goal field class, uses HyperbolicSpiral field to form the resulting univector field.
    
    Args:
        Kr [float]: Adjustable parameter.
        radius [float]: Radius that decides the size of the spiral.

    Reference: "Evolutionary Univector Field-based Navigation with Collision Avoidance for Mobile Robot"
    '''

    def __init__(self, Kr: float, radius: float):
        self.Kr = Kr
        self.radius = radius
        self.hyperbolicSpiral = HyperbolicSpiral(self.Kr, self.radius)
        
        self.origin = Vec2D.origin()

        self.toUnivectorMatrix = None
        self.toCanonicalMatrix = None

    def update_params(self, Kr: float, radius: float) -> None:
        '''
        Parameters update method.
        '''
        self.Kr = Kr
        self.radius = radius
        self.hyperbolicSpiral.update_params(self.Kr, self.radius)

    def update_axis(self, origin: Vec2D, u_axis: Vec2D) -> None:
        '''
        Update axes base method.
        '''
        self.origin = origin
        self.build_axis(u_axis)

    def build_axis(self, u_axis: Vec2D) -> None:
        '''
        Build reference axes method.

        Args:
            u_axis [Vec2D]: axis witch will be the basis of CanonicalMatrix and the invert matrix.
        '''
        u = u_axis/-u_axis.norm()
        theta = atan2(u[1], u[0])
        v = Vec2D(-sin(theta), cos(theta))

        self.toCanonicalMatrix = Mat2D(u, v)
        self.toUnivectorMatrix = self.toCanonicalMatrix.invert()

    def fi_tuf(self, _p: Vec2D, radius: Optional[float] = None) -> float:
        '''
        Calculate fi_tuf method.

        Args:
            p [Vec2D]: position.
            radius Optional[float]: radius of Hyperbolic Spiral.
        '''
        p = _p - self.origin
        r = self.radius if radius is None else radius

        p = self.toUnivectorMatrix.transform(p)

        x, y = p

        # Parece que houve algum erro de digitacao no artigo
        # Pois quando pl e pr sao definidos dessa maneira o campo gerado
        # se parece mais com o resultado obtido no artigo
        pl = Vec2D(x, y - r)
        pr = Vec2D(x, y + r)

        # Este caso eh para quando o robo esta dentro do "circulo" da bola
        if -r <= y < r:
            nh_pl = self.hyperbolicSpiral.n_h(pl, cw=False)
            nh_pr = self.hyperbolicSpiral.n_h(pr, cw=True)

            # Apesar de no artigo nao ser utilizado o modulo, quando utilizado
            # na implementacao o resultado foi mais condizente com o artigo
            vec = (abs(y + r) * nh_pl + abs(y - r) * nh_pr) / (2.0 * r)
            vec = self.toCanonicalMatrix.transform(vec)
        else:
            if y < -r:
                theta = self.hyperbolicSpiral.fi_h(pl, cw=True)
            else:  # y >= r
                theta = self.hyperbolicSpiral.fi_h(pr, cw=False)

            vec = Vec2D(cos(theta), sin(theta))
            vec = self.toCanonicalMatrix.transform(vec)

        return atan2(vec[1], vec[0])