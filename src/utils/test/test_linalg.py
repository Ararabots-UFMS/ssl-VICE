import pytest
from math import pi
from utils.linalg import Vec2D, Mat2D

constructor_x = 1.0
constructor_y = 1.0

###############################################################
############################ Ved2D ############################
###############################################################

@pytest.fixture
def vec2D():
    return Vec2D(constructor_x, constructor_y)

######################### Constructor #########################
def test_constructor_x(vec2D):
    assert isinstance(vec2D.x, float)

def test_constructor_y(vec2D):
    assert isinstance(vec2D.y, float)

######################## Magic methods ########################
    
@pytest.mark.parametrize(
    #Testing __add__, __sub__, __eq__
    "x1,         y1,         x2,         y2,         add_expected,    sub_expected,    eq_expected", [
    (0,          0,          0,          0,          (0,0),           (0,0),           True),
    (1,          1,          1,          1,          (2,2),           (0,0),           True),
    (9,          2,          3,          1,          (12, 3),         (6, 1),          False),
    (-3,         1,          -2,         -8,         (-5, -7),        (-1, 9),         False),
    (6.32,       5.12,       7.45,       1.74,       (13.77, 6.86),   (-1.13, 3.38),   False),
    (5.55,       -4.23,      6.99,       -10.1,      (12.54, -14.33), (-1.44, 5.87),   False),
    (0,          0,          7.88,       -2.45,      (7.88, -2.45),   (-7.88, 2.45),   False),
    (2.23,       -2.67,      0,          0,          (2.23, -2.67),   (2.23, -2.67),   False)
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
    #Testing __mul__, __rmul__, __truediv__, __neg__
    "x,          y,          alpha,      mul_expected,        rmul_expected,        truediv_expected,        neg_expected", [
    (0,          0,          1,          (0,0),               (0,0),                (0,0),                   (0,0)),
    (1,          1,          1,          (1,1),               (1,1),                (1,1),                   (-1,-1)),
    (9,          2,          3,          (27,6),              (27,6),               (3,0.6667),              (-9,-2)),
    (-3,         1,          -2,         (6, -2),             (6, -2),              (1.5, -0.5),             (3, -1)),
    (6.32,       5.12,       7.45,       (47.084, 38.144),    (47.084, 38.144),     (0.8483, 0.6872),        (-6.32, -5.12)),
    (5.55,       -4.23,      6.99,       (38.7945, -29.5677), (38.7945, -29.5677),  (0.7939, -0.6051),       (-5.55, 4.23)),
    (0,          0,          7.88,       (0,0),               (0,0),                (0,0),                   (0,0)),
    (2.23,       -2.67,      1,          (2.23, -2.67),       (2.23, -2.67),        (2.23, -2.67),           (-2.23, 2.67))
])
def test_scalar_operator(x ,y, alpha, mul_expected, rmul_expected, truediv_expected, neg_expected):
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

    # Neg operator.
    neg_vec = -vec
    assert neg_vec[0] == neg_expected[0]
    assert neg_vec[1] == neg_expected[1]

def test_list_properties():
    # Testing to_list, __getitem__, __setitem__ method.
    vec = Vec2D(10.11, -19.88)

    # To list.
    assert vec.to_list() == [10.11, -19.88]

    # Get item.
    assert vec[0] == 10.11
    assert vec[1] == -19.88

    # Set item.
    vec[0] = -15.0
    vec[1] = 11.0
    assert vec[0] == -15.0
    assert vec[1] == 11.0

###################### Vector Operations #######################
@pytest.mark.parametrize(                                      
    # Testing sigle vec operations: norm, versor, angle, copy.
    "x,          y,          norm_expected,          versor_expected,          angle_expected", [
    (1,          1,          1.4142,                 (0.7071, 0.7071),         0.7853981),
    (9,          2,          9.2195,                 (0.9761, 0.2169),         0.2186689),
    (-3,         1,          3.1623,                 (-0.9487, 0.3162),        2.819842),
    (6.32,       5.12,       8.1336,                 (0.7770, 0.6294),         0.6808852),
    (5.55,       -4.23,      6.9782,                 (0.7953, -0.6061),        -0.6512395),
    (2.23,       -2.67,      3.4787,                 (0.6410, -0.7675),        -0.8749538)
])
def test_sigleVec_operations(x, y, norm_expected, versor_expected, angle_expected):
    vec = Vec2D(x, y)

    # Norm.
    vec_norm = vec.norm()
    assert abs(vec_norm - norm_expected) < 1e-4

    # Versor.
    versor = vec.versor()
    assert abs(versor.x - versor_expected[0]) < 1e-4
    assert abs(versor.y - versor_expected[1]) < 1e-4

    # Angle.
    rad_angle = vec.angle()
    degrees_angle = vec.angle(degrees=True)

    degrees_angle_expected = angle_expected * 180 / pi
    assert abs(rad_angle - angle_expected) < 1e-4
    assert abs(degrees_angle - degrees_angle_expected) < 1e-4

    # Copy.
    vec_copy = vec.copy()
    assert vec_copy.x == vec.x
    assert vec_copy.y == vec.y

@pytest.mark.parametrize(
    # Testing 2 vec operations: dot, cross.
    "x1,         y1,         x2,         y2,         dot_expected, cross_expected", [
    (1,          1,          1,          1,          2,            0),
    (9,          2,          3,          1,          29,           3),
    (-3,         1,          -2,         -8,         -2,           26),
    (6.32,       5.12,       7.45,       1.74,       55.9928,      -27.1471),
    (5.55,       -4.23,      6.99,       -10.1,      81.5175,      -26.4872),
    (2.23,       -2.67,      0,          0,          0,            0)
])
def test_2vec_operations(x1, y1, x2, y2, dot_expected, cross_expected):
    vec1 = Vec2D(x1, y1)
    vec2 = Vec2D(x2, y2)

    # Dot.
    dot_product = vec1.dot(vec2)
    assert abs(dot_product - dot_expected) < 1e-4

    # Cross.
    cross_product = vec1.cross(vec2)
    assert abs(cross_product - cross_expected) < 1e-4

###################### Class Methods ######################
def test_left():
    vec = Vec2D.left()
    assert vec == Vec2D(-1, 0)

def test_right():
    vec = Vec2D.right()
    assert vec == Vec2D(1, 0)

def test_up():
    vec = Vec2D.up()
    assert vec == Vec2D(0, 1)

def test_down():
    vec = Vec2D.down()
    assert vec == Vec2D(0, -1)

def test_origin():
    vec = Vec2D.origin()
    assert vec == Vec2D(0, 0)

def test_from_array():
    vec = Vec2D.from_array([1, 2])
    assert vec == Vec2D(1, 2)

###########################################################
###########################################################
#########################################################
    
