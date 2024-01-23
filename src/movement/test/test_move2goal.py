import pytest
from math import atan2, cos, sin
from utils.linalg import Vec2D, Mat2D

from univector.components.move2goal import Move2Goal
from univector.components.hyperbolic_spiral import HyperbolicSpiral

constructor_kr = 5
constructor_radius = 5

@pytest.fixture
def move2goal():
    return Move2Goal(constructor_kr, constructor_radius)

######################### Constructor #########################

def test_constructor_kr(move2goal):
    assert constructor_kr == move2goal.Kr

def test_constructor_radius(move2goal):
    assert constructor_radius == move2goal.radius 

def test_constructor_hyperbolicSpiral(move2goal):
    assert isinstance(move2goal.hyperSpiral, HyperbolicSpiral)

def test_constructor_origin(move2goal):
    assert move2goal.origin == Vec2D(0, 0)

def test_constructor_vec_u(move2goal):
    assert move2goal.u == Vec2D(0, 0)

def test_constructor_vec_v(move2goal):
    assert move2goal.v == Vec2D(0, 0)

def test_constructor_toUnivectorMatrix(move2goal):
    assert move2goal.toUnivectorMatrix == None

def test_constructor_toCanonicalMatrix(move2goal):
    assert move2goal.toCanonicalMatrix == None

######################### update_params #########################
    
def test_update_params_kr(move2goal):
    new_kr = 4

    move2goal.update_params(new_kr, constructor_radius)

    assert new_kr == move2goal.Kr

def test_update_params_radius(move2goal):
    new_radius = 4

    move2goal.update_params(constructor_kr, new_radius)

    assert new_radius == move2goal.radius

def test_update_params_hyperbolicSpiral(move2goal):
    new_kr = 4
    new_radius = 4

    move2goal.update_params(new_kr, new_radius)

    assert new_kr == move2goal.hyperSpiral.Kr
    assert new_radius == move2goal.hyperSpiral.radius

######################### build_axis #########################

@pytest.mark.parametrize(
    "origin,            vec_u,", [
    (Vec2D(0,0),        Vec2D(1, 0)),
    (Vec2D(1, 1),       Vec2D(-1, 1)),
    (Vec2D(0, 1),       Vec2D(-1, 0)),
    (Vec2D(-1, 1),      Vec2D(-1, -1)),
    (Vec2D(-1, 0),      Vec2D(15, 15)),
    (Vec2D(-1, -1),     Vec2D(15, -15)),
])
def test_build_axis_updated_u(move2goal, origin,  vec_u):
    u = vec_u/-vec_u.norm()
    theta = atan2(u[1], u[0])
    v = Vec2D(-sin(theta), cos(theta))
    expected_canonical = Mat2D(u, v)

    move2goal.update_axis(origin, vec_u)

    assert expected_canonical.u == move2goal.toCanonicalMatrix.u
    assert expected_canonical.v == move2goal.toCanonicalMatrix.v

######################### update_axis #########################
    
def test_update_axis_origin(move2goal):
    origin = Vec2D(1, 1)
    u = Vec2D(1, 0)

    move2goal.update_axis(origin, u)

    assert origin == move2goal.origin

@pytest.mark.parametrize(
    "vec_u,", [
    (Vec2D(1, 0)),
    (Vec2D(-1, 1)),
    (Vec2D(-1, 0)),
    (Vec2D(-1, -1)),
    (Vec2D(15, 15)),
    (Vec2D(15, -15)),
])
def test_update_axis_u(move2goal, vec_u):
    origin = Vec2D(0, 0)

    move2goal.update_axis(origin, vec_u)

    u = vec_u/-vec_u.norm()

    assert u == move2goal.u

######################### fi_tuf #########################
@pytest.mark.parametrize(
        "radius,        origin,        u,              p,              expected_angle", [
        (5,             Vec2D(0,0),    Vec2D(1, 0),    Vec2D(1, 0),    0),
        (5,             Vec2D(0,0),    Vec2D(1, 0),    Vec2D(-1, 0),   0),
        (5,             Vec2D(0,0),    Vec2D(1, 0),    Vec2D(0, 1),    -0.15661999255049955),
        (1,             Vec2D(0,0),    Vec2D(1, 0),    Vec2D(0, 0),    0),
        (1,             Vec2D(0,0),    Vec2D(1, 0),    Vec2D(0, 2),    -2.7488935718910694),
        (1,             Vec2D(0,0),    Vec2D(1, 0),    Vec2D(0, -1),   2.9171931783333793),
])
def test_fi_tuf(move2goal, radius, origin, u, p, expected_angle):

    move2goal.update_params(constructor_kr, radius)

    move2goal.update_axis(origin, u)
    result_angle = move2goal.fi_tuf(p)

    assert abs(expected_angle - result_angle) < 1e-10
