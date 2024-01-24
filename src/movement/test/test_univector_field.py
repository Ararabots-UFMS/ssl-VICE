import pytest
from movement.univector.univector_field import UnivectorField
from movement.univector.utils import LEFT, RIGHT
from utils.linalg import Vec2D

@pytest.fixture
def univectorField():
    return UnivectorField()

######################### get_attack_goal_axis #########################

def test_get_attack_goal_axis_left(univectorField):
    expected_LEFT = Vec2D.left()

    result = univectorField.get_attack_goal_axis(LEFT)
    
    assert expected_LEFT == result

def test_get_attack_goal_axis_right(univectorField):
    expected_RIGHT = Vec2D.right()

    result = univectorField.get_attack_goal_axis(RIGHT)
    
    assert expected_RIGHT == result

######################### update_obstacles #########################

@pytest.mark.parametrize(
    "_obstacles,                        _obsSpeeds", [
    ([Vec2D(1, 2), Vec2D(3, 4)],        [Vec2D(0.5, 0.5), Vec2D(-0.5, -0.5)]),  # test random positive integer values
    ([Vec2D(5, 6), Vec2D(7, 8)],        [Vec2D(-0.5, -0.5), Vec2D(0.5, 0.5)]), # test random mixed integer values
])
def test_update_obstacles(univectorField, _obstacles, _obsSpeeds):
    univectorField.update_obstacles(_obstacles, _obsSpeeds)

    assert _obstacles == univectorField.obstacles
    assert _obsSpeeds == univectorField.obstaclesSpeed

######################### update_constants #########################
    
@pytest.mark.parametrize(
    "_RADIUS,   _KR,    _K0,    _DMIN,      _LDELTA", [
    (1.0,       2.0,    3.0,    4.0,        5.0),  # test with positive float values
    (0.5,       1.5,    2.5,    3.5,        4.5),  # test with positive float values
    (-1.0,      -2.0,   -3.0,   -4.0,       -5.0),  # test with negative float values
])
def test_update_constants(univectorField, _RADIUS, _KR, _K0, _DMIN, _LDELTA):
    univectorField.update_constants(_RADIUS, _KR, _K0, _DMIN, _LDELTA)

    assert univectorField.RADIUS == _RADIUS
    assert univectorField.KR == _KR
    assert univectorField.K0 == _K0
    assert univectorField.DMIN == _DMIN
    assert univectorField.LDELTA == _LDELTA

    assert univectorField.avdObsField.K0 == _K0
    assert univectorField.mv2Goal.Kr == _KR
    assert univectorField.mv2Goal.radius == _RADIUS
