from math import atan2, cos, sin
from utils.linalg import Vec2D, Mat2D
from univector.components.hyperbolic_spiral import HyperbolicSpiral

class Move2Goal:
    '''
    Uses the hyperbolic spiral field to create the move2goal field used in the resulting univector field
    '''

    def __init__(self, _Kr: float, _radius: float):
        self.Kr = _Kr
        self.radius = _radius

        ###### variable namo should be hyperbolicSpiral ######
        self.hyperSpiral = HyperbolicSpiral(self.Kr, self.radius)
        ######################################################
        self.origin = Vec2D.origin()

        self.u = Vec2D.origin()
        self.v = Vec2D.origin()

        self.toUnivectorMatrix = None
        self.toCanonicalMatrix = None

    def update_params(self, _KR: float, _RADIUS: float) -> None:
        self.Kr = _KR
        self.radius = _RADIUS
        self.hyperSpiral.update_params(self.Kr, self.radius)

    def update_axis(self, new_origin: Vec2D, new_u_axis: Vec2D) -> None:
        self.origin = new_origin
        self.u = new_u_axis
        self.build_axis()

    def build_axis(self) -> None:
        '''
        should be before update_axis
        '''
        self.u /= -self.u.norm()
        theta = atan2(self.u[1], self.u[0])
        self.v = Vec2D(-sin(theta), cos(theta))

        self.toCanonicalMatrix = Mat2D(self.u, self.v)
        self.toUnivectorMatrix = self.toCanonicalMatrix.invert()

    def fi_tuf(self, _p: Vec2D) -> float:
        n_h = self.hyperSpiral.n_h
        p = _p - self.origin
        r = self.radius

        p = self.toUnivectorMatrix.transform(p)

        x, y = p
        yl = y + r
        yr = y - r

        # Parece que houve algum erro de digitacao no artigo
        # Pois quando pl e pr sao definidos dessa maneira o campo gerado
        # se parece mais com o resultado obtido no artigo
        pl = Vec2D(x, yr)
        pr = Vec2D(x, yl)

        # Este caso eh para quando o robo esta dentro do "circulo" da bola
        if -r <= y < r:
            nh_pl = n_h(pl, cw=False)
            nh_pr = n_h(pr, cw=True)

            # Apesar de no artigo nao ser utilizado o modulo, quando utilizado
            # na implementacao o resultado foi mais condizente com o artigo
            vec = (abs(yl) * nh_pl + abs(yr) * nh_pr) / (2.0 * r)
            vec = self.toCanonicalMatrix.transform(vec)
        else:
            if y < -r:
                theta = self.hyperSpiral.fi_h(pl, cw=True)
            else:  # y >= r
                theta = self.hyperSpiral.fi_h(pr, cw=False)

            vec = Vec2D(cos(theta), sin(theta))
            # TODO: MATRIZES
            vec = self.toCanonicalMatrix.transform(vec)

        return atan2(vec[1], vec[0])