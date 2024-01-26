import pytest
from utils.linalg import Vec2D, Mat2D

constructor_x = 1.0
constructor_y = 1.0

#########################################################
######################### Ved2D #########################
#########################################################

@pytest.fixture
def vec2D():
    return Vec2D(constructor_x, constructor_y)

######################### Constructor #########################
def test_constructor_x(vec2D):
    assert isinstance(vec2D.x, float)

def test_constructor_y(vec2D):
    assert isinstance(vec2D.y, float)

######################### Methods #########################
    
@pytest.mark.parametrize(
    #Testing __add__, __sub__, __eq__
    "x1,         y1,         x2,         y2,         add_expected, sub_expected, eq_expected", [
    (0, 0, 0, 0,                                                (0,0), (0,0), True),
    (1, 1, 1, 1,                                                (2,2), (0,0), True),
    (9, 2, 3, 1,                                                (12, 3), (6, 1), False),
    (-3, 1, -2, -8,                                             (-5, -7), (-1, 9), False),
    (6.32, 5.12, 7.45, 1.74,                                    (13.77, 6.86), (-1.13, 3.38), False),
    (5.55, -4.23, 6.99, -10.1,                                  (12.54, -14.33), (-1.44, 5.87), False),
    (0, 0, 7.88, -2.45,                                         (7.88, -2.45), (-7.88, 2.45), False),
    (2.23, -2.67, 0, 0,                                         (2.23, -2.67), (2.23, -2.67), False)
])
def test_vector_operators(x1, y1, x2, y2, add_expected, sub_expected, eq_expected):
    vec1 = Vec2D(x1, y1)
    vec2 = Vec2D(x2, y2)

    # Add operator.
    add_vec = vec1 + vec2
    assert abs(add_vec.x - add_expected[0]) < 1e-4
    assert abs(add_vec.y - add_expected[1]) < 1e-4

    # Sub operator.
    sub_vec = vec1 - vec2
    assert abs(sub_vec.x - sub_expected[0]) < 1e-4
    assert abs(sub_vec.y - sub_expected[1]) < 1e-4

    # Eq operator.
    eq_vec = vec1 == vec2
    assert eq_vec == eq_expected

@pytest.mark.parametrize(
    #Testing __mul__, __rmul__, __truediv__
    "x,          y,          alpha,      mul_expected, rmul_expected, truediv_expected", [
    (0, 0, 1,                                                   (0,0), (0,0), (0,0)),
    (1, 1, 1,                                                   (1,1), (1,1), (1,1)),
    (9, 2, 3,                                                   (27,6), (27,6), (3,0.6667)),
    (-3, 1, -2,                                                 (6, -2), (6, -2), (1.5, -0.5)),
    (6.32, 5.12, 7.45,                                          (47.084, 38.144), (47.084, 38.144), (0.8483, 0.6872)),
    (5.55, -4.23, 6.99,                                         (38.7945, -29.5677), (38.7945, -29.5677), (0.7939, -0.6051)),
    (0, 0, 7.88,                                                (0,0), (0,0), (0,0)),
    (2.23, -2.67, 1,                                            (2.23, -2.67), (2.23, -2.67), (2.23, -2.67))
])
def test_scalar_operator(x ,y, alpha, mul_expected, rmul_expected, truediv_expected):
    vec = Vec2D(x, y)

    # Mul operator.
    mul_vec = vec * alpha
    assert abs(mul_vec.x - mul_expected[0]) < 1e-4
    assert abs(mul_vec.y - mul_expected[1]) < 1e-4

    # Rmul operator.
    rmul_vec = alpha * vec
    assert abs(rmul_vec.x - rmul_expected[0]) < 1e-4
    assert abs(rmul_vec.y - rmul_expected[1]) < 1e-4

    # Truediv operator.
    truediv_vec = vec / alpha
    assert abs(truediv_vec.x - truediv_expected[0]) < 1e-4
    assert abs(truediv_vec.y - truediv_expected[1]) < 1e-4