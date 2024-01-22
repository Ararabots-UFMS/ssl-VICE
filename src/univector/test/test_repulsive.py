import pytest
from univector.components.repulsive import Repulsive
from utils.linalg import Vec2D
from math import pi

@pytest.fixture
def repulsive():
    return Repulsive()

######################### Constructor #########################

def test_contructor_origin(repulsive):
    assert isinstance(repulsive.origin, Vec2D)
    #Is this testing repulsive constructor or Vec2D origin?
    assert Vec2D(0, 0) == repulsive.origin

######################### update_origin #########################
    
def test_update_origin(repulsive):
    repulsive.update_origin(Vec2D(1, 1))
    assert Vec2D(1, 1) == repulsive.origin

######################### fi_r #########################
    
@pytest.mark.parametrize(
    "_p,            _origin,        expected", [
    (Vec2D(1, 1),   Vec2D(0, 0),    0.7853981633974483),    # test random positive integer values
    (Vec2D(1, 1),   Vec2D(1, 1),    0),                     # test random mixed integer values
    (Vec2D(1, 1),   Vec2D(-1, -1),  0.7853981633974483),     # edge case: radius is None
    (Vec2D(-1, 1),  Vec2D(1, -1),   2.356194490192345),    # edge case: cw is False
    (Vec2D(1, 0),   None,           0),                     # angle should be 0 when _p is on the x-axis
    (Vec2D(0, 1),   None,           pi/2),                  # angle should be pi/2 when _p is on the y-axis
])   
def test_fi_r(repulsive, _p, _origin, expected):
    result = repulsive.fi_r(_p, _origin)
    assert abs(expected - result) < 1e-10  # use a small tolerance because of floating point precision