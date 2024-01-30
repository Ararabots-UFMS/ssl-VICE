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

    assert _obstacles == univectorField.obstaclesPos
    assert _obsSpeeds == univectorField.obstaclesVelocity

######################### update_constants #########################
    
@pytest.mark.parametrize(
    "_RADIUS,   _KR,    _K0,    _DMIN,      _LDELTA", [
    (1.0,       2.0,    3.0,    4.0,        5.0),  # test with positive float values
    (0.5,       1.5,    2.5,    3.5,        4.5),  # test with positive float values
    (-1.0,      -2.0,   -3.0,   -4.0,       -5.0),  # test with negative float values
])
def test_update_constants(univectorField, _RADIUS, _KR, _K0, _DMIN, _LDELTA):
    univectorField.update_constants(_RADIUS, _KR, _K0, _DMIN, _LDELTA)

    assert univectorField.Radius == _RADIUS
    assert univectorField.Kr == _KR
    assert univectorField.K0 == _K0
    assert univectorField.Dmin == _DMIN
    assert univectorField.LDelta == _LDELTA

    assert univectorField.avoidObstacleField.K0 == _K0
    assert univectorField.move2Goal.Kr == _KR
    assert univectorField.move2Goal.radius == _RADIUS

######################### get_vec_with_ball #########################
@pytest.mark.parametrize(
    "_robotPos,     _vRobot,       _ball,         _attack_goal,    _RADIUS,  _KR,   _K0,   _DMIN,  _LDELTA,  expected", [
    (Vec2D(-1, 2),  Vec2D(5, 8),   Vec2D(1, 1),   RIGHT,         3.0,      1.1,   3.4,   0.1,    3.4,      Vec2D(-0.7982, 0.6024)), 
    (Vec2D(11, 4),  Vec2D(9, 2),   Vec2D(5, 2),   RIGHT,         4.5,      2.2,   2.7,   0.4,    -3.6,     Vec2D(-0.8963, 0.4435)),
    (Vec2D(7, 3),   Vec2D(6, 4),   Vec2D(2, 3),   RIGHT,         5.0,      3.3,   1.5,   0.7,    3.8,      Vec2D(-0.4180, 0.9084)),
    (Vec2D(-1, 2),  Vec2D(5, 8),   Vec2D(1, 4),   LEFT,          8.2,      4.4,   9.9,   -1.2,   4.0,      Vec2D(-0.7956, 0.6058)),
    (Vec2D(11, 4),  Vec2D(9, 2),   Vec2D(5, 5),   LEFT,          4.5,      5.5,   8.8,   1.5,    4.7,      Vec2D(-0.7250, 0.6888)),
    (Vec2D(7, 3),   Vec2D(6, 4),   Vec2D(2, 6),   LEFT,          7.1,      -6.6,  7.7,   1.8,    4.2,      Vec2D(0.1387, 0.9903)),
    (Vec2D(-1, 2),  Vec2D(5, 8),   Vec2D(1, 7),   RIGHT,         5.0,      7.7,   6.6,   2.3,    2.3,      Vec2D(-0.4927, 0.8702)), 
    (Vec2D(11, 4),  Vec2D(9, 2),   Vec2D(5, 8),   RIGHT,         2.5,      8.8,   5.5,   2.6,    1.9,      Vec2D(-0.0551, 0.9985)),
    (Vec2D(7, 3),   Vec2D(6, 4),   Vec2D(2, 9),   RIGHT,         -9.0,     9.9,   4.4,   2.9,    1.8,      Vec2D(0.8963, 0.4434)),
    (Vec2D(-1, 2),  Vec2D(5, 8),   Vec2D(1, 1),   LEFT,          7.3,      1.5,   3.3,   3.7,    1.7,      Vec2D(-0.4927, 0.8702)),
    (Vec2D(11, 4),  Vec2D(9, 2),   Vec2D(5, 2),   LEFT,          8.5,      2.7,   -2.2,  3.5,    5.6,      Vec2D(-0.1850, 0.9827)),
    (Vec2D(7, 3),   Vec2D(6, 4),   Vec2D(2, 3),   LEFT,          4.0,      3.4,   1.1,   3.3,    6.2,      Vec2D(0.7530, 0.6580)),
])

def test_get_vec_with_ball(_robotPos, _vRobot, _ball, _attack_goal, _RADIUS, _KR, _K0, _DMIN, _LDELTA, expected):
    univectorField = UnivectorField()
    univectorField.update_constants(_RADIUS, _KR, _K0, _DMIN, _LDELTA)
    _obstacles = [Vec2D(1, 2), Vec2D(3, 4)]
    _obsSpeeds = [Vec2D(0.5, 0.5), Vec2D(-0.5, -0.5)]
    univectorField.update_obstacles(_obstacles, _obsSpeeds)
    result = univectorField.get_vec_with_ball(_robotPos, _vRobot, _ball, _attack_goal)
    assert abs(expected[0] - result[0]) < 1e-4
    assert abs(expected[1] - result[1]) < 1e-4

######################### get_angle_with_ball #########################
@pytest.mark.parametrize(
    "_robotPos,     _vRobot,       _ball,         _attack_goal,  _RADIUS,  _KR,   _K0,   _DMIN,  _LDELTA,  expected", [
    (Vec2D(-1, 2),  Vec2D(5, 8),   Vec2D(1, 2),   RIGHT,         3.0,      1.1,   3.4,   0.1,    3.4,      2.3875772324196487), 
    (Vec2D(11, 4),  Vec2D(9, 2),   Vec2D(5, 6),   RIGHT,         4.5,      2.2,   2.7,   0.4,    -3.6,     1.951010598629771),
    (Vec2D(7, 3),   Vec2D(6, 4),   Vec2D(2, 2),   RIGHT,         5.0,      3.3,   1.5,   0.7,    3.8,      2.261460486322725),
    (Vec2D(-1, 2),  Vec2D(5, 8),   Vec2D(1, 2),   LEFT,          8.2,      4.4,   9.9,   -1.2,   4.0,      2.5075556055731703),
    (Vec2D(11, 4),  Vec2D(9, 2),   Vec2D(5, 6),   LEFT,          4.5,      5.5,   8.8,   1.5,    4.7,      2.0210505285968274),
    (Vec2D(7, 3),   Vec2D(6, 4),   Vec2D(2, 2),   LEFT,          7.1,      -6.6,  7.7,   1.8,    4.2,      2.0756472445267713),
    (Vec2D(-1, 2),  Vec2D(5, 8),   Vec2D(1, 2),   RIGHT,         5.0,      7.7,   6.6,   2.3,    2.3,      2.085984740057053), 
    (Vec2D(11, 4),  Vec2D(9, 2),   Vec2D(5, 6),   RIGHT,         2.5,      8.8,   5.5,   2.6,    1.9,      1.7715359137500177),
    (Vec2D(7, 3),   Vec2D(6, 4),   Vec2D(2, 2),   RIGHT,         -9.0,     9.9,   4.4,   2.9,    1.8,      -1.066254531448714),
    (Vec2D(-1, 2),  Vec2D(5, 8),   Vec2D(1, 2),   LEFT,          7.3,      1.5,   3.3,   3.7,    1.7,      2.085984740057053),
    (Vec2D(11, 4),  Vec2D(9, 2),   Vec2D(5, 6),   LEFT,          8.5,      2.7,   -2.2,  3.5,    5.6,      1.7568302062239565),
    (Vec2D(7, 3),   Vec2D(6, 4),   Vec2D(2, 2),   LEFT,          4.0,      3.4,   1.1,   3.3,    6.2,      0.7909853164078244),
])

def test_get_angle_with_ball(_robotPos, _vRobot, _ball, _attack_goal, _RADIUS, _KR, _K0, _DMIN, _LDELTA, expected):
    univectorField = UnivectorField()
    univectorField.update_constants(_RADIUS, _KR, _K0, _DMIN, _LDELTA)
    _obstacles = [Vec2D(1, 2), Vec2D(3, 4)]
    _obsSpeeds = [Vec2D(0.5, 0.5), Vec2D(-0.5, -0.5)]
    univectorField.update_obstacles(_obstacles, _obsSpeeds)
    result = univectorField.get_angle_with_ball(_robotPos, _vRobot, _ball, _attack_goal)
    assert expected == result

######################### get_angle_vec #########################
@pytest.mark.parametrize(
    "_robotPos,     _vRobot,       _goal_pos,     _goal_axis,    _RADIUS,  _KR,   _K0,   _DMIN,  _LDELTA,  expected", [
    (Vec2D(-1, 2),  Vec2D(5, 9),   Vec2D(1, 2),   Vec2D(1, 6),   3.0,      1.1,   3.4,   0.1,    3.4,      1.872685684042867), 
    (Vec2D(11, 4),  Vec2D(9, 8),   Vec2D(5, 6),   Vec2D(2, 5),   4.5,      2.2,   2.7,   0.4,    -3.6,     -2.678580887545356),
    (Vec2D(7, 3),   Vec2D(6, 7),   Vec2D(2, 2),   Vec2D(3, 4),   5.0,      3.3,   1.5,   0.7,    3.8,      0.141120791936209),
    (Vec2D(-1, 2),  Vec2D(5, 6),   Vec2D(1, 2),   Vec2D(4, 3),   8.2,      4.4,   9.9,   -1.2,   4.0,      1.6525481782655795),
    (Vec2D(11, 4),  Vec2D(9, 5),   Vec2D(5, 6),   Vec2D(5, 2),   4.5,      5.5,   8.8,   1.5,    4.7,      -1.1140343545383058),
    (Vec2D(7, 3),   Vec2D(6, 4),   Vec2D(2, 2),   Vec2D(6, 1),   7.1,      -6.6,  7.7,   1.8,    4.2,      2.005797871565939),
    (Vec2D(-1, 2),  Vec2D(5, 3),   Vec2D(1, 2),   Vec2D(7, 9),   5.0,      7.7,   6.6,   2.3,    2.3,      2.085984740057052), 
    (Vec2D(11, 4),  Vec2D(9, 2),   Vec2D(5, 6),   Vec2D(8, 8),   2.5,      8.8,   5.5,   2.6,    1.9,      -2.575045164831861),
    (Vec2D(7, 3),   Vec2D(6, 1),   Vec2D(2, 2),   Vec2D(9, 7),   -9.0,     9.9,   4.4,   2.9,    1.8,      2.5070110890403585),
    (Vec2D(-1, 2),  Vec2D(5, 9),   Vec2D(1, 2),   Vec2D(1, 6),   7.3,      1.5,   3.3,   3.7,    1.7,      2.1127448742867103),
    (Vec2D(11, 4),  Vec2D(9, 8),   Vec2D(5, 6),   Vec2D(2, 5),   8.5,      2.7,   -2.2,  3.5,    5.6,      -0.9648503043167926),
    (Vec2D(7, 3),   Vec2D(6, 7),   Vec2D(2, 2),   Vec2D(3, 4),   4.0,      3.4,   1.1,   3.3,    6.2,      0.1891065313304643),
])

def test_get_angle_vec(_robotPos, _vRobot, _goal_pos, _goal_axis, _RADIUS, _KR, _K0, _DMIN, _LDELTA, expected):
    univectorField = UnivectorField()
    univectorField.update_constants(_RADIUS, _KR, _K0, _DMIN, _LDELTA)
    _obstacles = [Vec2D(1, 2), Vec2D(3, 4)]
    _obsSpeeds = [Vec2D(0.5, 0.5), Vec2D(-0.5, -0.5)]
    univectorField.update_obstacles(_obstacles, _obsSpeeds)
    result = univectorField.get_angle_vec(_robotPos, _vRobot, _goal_pos, _goal_axis)
    assert expected == result
