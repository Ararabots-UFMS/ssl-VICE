from typing import Iterable, List
import math

class Vec2D:
    pass

class Mat2D:
    pass

class Vec2D:

    def __init__(self, x: float, y: float):

        self.x = x
        self.y = y

    def to_list(self) -> List[float]:
        return [self.x, self.y]

    def __add__(self, vec: Vec2D) -> Vec2D:
        return Vec2D(self.x + vec.x, self.y + vec.y)

    def __sub__(self, vec: Vec2D):
        __v = Vec2D(self.x - vec.x, self.y - vec.y)
        return __v

    def __eq__(self, vec: Vec2D) -> bool:
        if isinstance(vec, Vec2D):
            return self.x == vec.x and self.y == vec.y
        return False

    def __mul__(self, alpha: float) -> Vec2D:
        # Multiplicação por escalar
        return Vec2D(alpha*self.x, alpha*self.y)

    def __rmul__(self, alpha) -> Vec2D:
        # Multiplicação por escalar 
        return self.__mul__(alpha)

    def __neg__(self) -> Vec2D:
        # Inversão de sentido
        return Vec2D(-self.x, -self.y)

    def __truediv__(self, alpha) -> Vec2D:
        # Multiplicação por escalar menor que 1 (divisão)
        return Vec2D(self.x/alpha, self.y/alpha)

    def __repr__(self) -> str:
        # Retorna string para fácil consulta de valores
        return f"Vec2({self.x:.4f}, {self.y:.4f})"

    def __getitem__(self, index: int) -> float:
        # Indexando valor
        assert type(index) == int
        if index == 0:
            return self.x 
        elif index == 1:
            return self.y
        
        raise IndexError

    def __setitem__(self, index: int, value: float) -> None:
        assert type(index) == int
        if index == 0:
            self.x = value 
        elif index == 1:
            self.y = value
        else:        
            raise IndexError

    def norm(self) -> float:
        # Retorna a norma/comprimento do vetor 
        return math.sqrt(self.x**2 + self.y**2)

    def versor(self) -> Vec2D:
        # Retorna um novo Vec2D contendo um versor do vetor atual
        norm = self.norm()
        return (1 / norm) * Vec2D(self.x, self.y)

    def dot(self, vec: Vec2D) -> float: 
        # Retorna o produto escalar com o vetor passado como argumento
        x = self.x * vec.x
        y = self.y * vec.y

        return x + y

    def cross(self, vec) -> float:
        # Retorna o produto vetorial com o vetor passado como argumento
        return self.x * vec.y - self.y * vec.x

    def angle(self, degrees=False) -> float:
        # Retorna o angulo formado entre o vetor e o eixo horizontal 
        result = math.atan2(self.y, self.x) 
        if degrees: result *= 180 / math.pi

        return result
    
    def copy(self) -> Vec2D:
        # Retorna um novo vetor contendo uma copia do atual
        return Vec2D(self.x, self.y)

    @classmethod
    def left(cls):
        return cls(-1, 0)

    @classmethod
    def right(cls):
        return cls(1, 0)

    @classmethod
    def up(cls):
        return cls(0, 1)

    @classmethod
    def down(cls):
        return cls(0, -1)
    
    @classmethod
    def origin(cls):
        return cls(0, 0)

    @classmethod
    def from_array(cls, array: Iterable[float]):
        return cls(array[0], array[1])

class Mat2D:

    def __init__(self, u: Vec2D, v: Vec2D):

        self.u = u
        self.v = v

    @classmethod
    def identity(cls) -> Mat2D:
        # Retorna uma matriz identidade 2x2
        return cls(Vec2D(1, 0), Vec2D(0, 1))

    def __repr__(self) -> str:

        str_rep = "Mat2D (\n"
        str_rep += f"  [{self.u[0]}\t{self.v[0]}]\n"
        str_rep += f"  [{self.u[1]}\t{self.v[1]}]\n"
        str_rep += ")" 

        return str_rep
    
    def __getitem__(self, indexes):
        # Indexando um Mat2D m usando m[i,j] 

        assert isinstance(indexes, tuple)

        i, j = indexes

        assert j >= 0 and j < 2
        assert i >= 0 and i < 2

        column = self.u if j == 0 else self.v

        return column[i]

    def __setitem__(self, indexes, value):

        assert isinstance(indexes, tuple)

        i, j = indexes

        assert j >= 0 and j < 2
        assert i >= 0 and i < 2

        column = self.u if j == 0 else self.v

        column[i] = value

    def __mul__(self, alpha: float) -> Mat2D:
        # Multiplicação por escalar
        return Mat2D(alpha*self.u, alpha*self.v)

    def __rmul__(self, alpha) -> Mat2D:

        return self.__mul__(alpha)

    def __neg__(self) -> Mat2D:
        # Invertendo o sinal de todos os elementos da matriz
        return self.__mul__(-1)

    def transform(self, vec: Vec2D) -> Vec2D:
        # Aplica a transformação contida na matriz no vetor passado como argumento
        return vec[0] * self.u + vec[1] * self.v

    def det(self) -> float:
        # Retorna o determinante da matriz
        return self.u[0]*self.v[1] - self.u[1] * self.v[0]

    def invert(self) -> Mat2D:
        # Retorna a matriz inversa

        det = self.det()
        assert det != 0 

        new_u = Vec2D(self.v[1], -self.u[1])
        new_v = Vec2D(-self.v[0], self.u[0])

        return (1/det) * Mat2D(new_u, new_v)