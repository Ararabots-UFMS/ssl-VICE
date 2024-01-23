import pytest
from movement.univector.avoid_obstacle import AvoidObstacle
from math import pi
from utils.linalg import Vec2D

constructor_pObs = Vec2D(0, 0)
constructor_vObs = Vec2D(0, 0)
constructor_pRobot = Vec2D(0, 0)
constructor_vRobot = Vec2D(0, 0)
constructor_K0 = 5

@pytest.fixture
def avoidObstacle():
    return AvoidObstacle(constructor_pObs, constructor_vObs, constructor_pRobot, constructor_vRobot, constructor_K0)

######################### Constructor #########################
def test_constructor_pObs(avoidObstacle):
    assert isinstance(avoidObstacle.pObs, Vec2D)

def test_constructor_vObs(avoidObstacle):
    assert isinstance(avoidObstacle.vObs, Vec2D)

def test_constructor_pRobot(avoidObstacle):
    assert isinstance(avoidObstacle.pRobot, Vec2D)

def test_constructor_vRobot(avoidObstacle):
    assert isinstance(avoidObstacle.vRobot, Vec2D)

def test_constructor_K0(avoidObstacle):
    assert constructor_K0 == avoidObstacle.K0

######################### get_s #########################
    
@pytest.mark.parametrize(
    "K0,    vObs,           vRobot,         expected", [
    (1,     Vec2D(3, 2),    Vec2D(1, 1),    Vec2D(2, 1)),           # test random positive integer values
    (2,     Vec2D(-7, 15),  Vec2D(1, 1),    Vec2D(-16, 28)),        # test random mixed integer values
    (3,     Vec2D(1, 1),    Vec2D(1, 1),    Vec2D(0, 0)),           # edge case: vObs and vRobot are the same
    (4,     Vec2D(2, 2),    Vec2D(0, 0),    Vec2D(8, 8)),           # edge case: vRobot is the origin
    (5,     Vec2D(0, 0),    Vec2D(2, 2),    Vec2D(-10, -10)),       # edge case: vObs is the origin
])
def test_get_s(K0, vObs, vRobot, expected):
    avoidObstacle = AvoidObstacle(constructor_pObs, vObs, constructor_pRobot, vRobot, K0)
    result = avoidObstacle.get_s()
    assert expected == result

######################### get_virtual_pos #########################
@pytest.mark.parametrize(
    "K0,    pObs,           vObs,           pRobot,             vRobot,         expected", [
    (1,     Vec2D(3, 2),    Vec2D(8, 9),    Vec2D(2, 1),        Vec2D(6, 1),    Vec2D(3.3430, 3.3720)),
    (2,     Vec2D(-7, 15),  Vec2D(2, 0),    Vec2D(-16, 28),     Vec2D(2, 3),    Vec2D(-7.0000, 9.0000)),
    (3,     Vec2D(1, 1),    Vec2D(1, 5),    Vec2D(0, 0),        Vec2D(5, 7),    Vec2D(-0.2649, 0.3675)),
    (4,     Vec2D(2, 2),    Vec2D(0, 0),    Vec2D(8, 8),        Vec2D(4, 11),   Vec2D(-0.8998, -5.9744)),
    (5,     Vec2D(0, 0),    Vec2D(2, 2),    Vec2D(-10, -10),    Vec2D(2, 1),    Vec2D(0.0000, 5.0000)),
])

def test_get_virtual_pos(K0, pObs, vObs, pRobot, vRobot, expected):
    avoidObstacle = AvoidObstacle(pObs, vObs, pRobot, vRobot, K0)
    result = avoidObstacle.get_virtual_pos()
    assert abs(expected[0] - result[0]) < 1e-4
    assert abs(expected[1] - result[1]) < 1e-4

######################### fi_auf #########################
@pytest.mark.parametrize(
    "K0, pObs,           vObs,         pRobot,           vRobot,       _robotPos,    _vPos,        expected", [
    (1,  Vec2D(3, 2),    Vec2D(9, 1),  Vec2D(2, 1),      Vec2D(2, 5),  Vec2D(9, -1), Vec2D(8, 8),  -1.460139105621001),
    (2,  Vec2D(-7, 15),  Vec2D(11, 3), Vec2D(-16, 28),   Vec2D(14, 3), Vec2D(1, 1),  None,         -0.7853981633974483),
    (3,  Vec2D(1, 1),    Vec2D(6, 2),  Vec2D(0, 0),      Vec2D(3, 3),  Vec2D(8, 2),  Vec2D(5, 3),  -0.3217505543966422),
    (4,  Vec2D(2, 2),    Vec2D(0, 12), Vec2D(8, 8),      Vec2D(2, 1),  Vec2D(10, 7), None,         -0.3382791806021421),
    (5,  Vec2D(0, 0),    Vec2D(2, 7),  Vec2D(-10, -10),  Vec2D(5, 6),  Vec2D(4, -4), Vec2D(6, 7),  -1.750649826587375),
])

def test_get_virtual_pos(K0, pObs, vObs, pRobot, vRobot,_robotPos, _vPos, expected):
    avoidObstacle = AvoidObstacle(pObs, vObs, pRobot, vRobot, K0)
    result = avoidObstacle.fi_auf(_robotPos, _vPos)
    assert abs(expected - result) < 1e-10

######################### update_param #########################
def test_update_param(avoidObstacle):
    new_k0 = 4

    avoidObstacle.update_param(new_k0)

    assert new_k0 == avoidObstacle.K0

######################### update_obstacle #########################
def test_update_obstacle_pObs(avoidObstacle):
    new_pObs = 1.2

    avoidObstacle.update_obstacle(new_pObs, constructor_vObs)

    assert new_pObs == avoidObstacle.pObs

def test_update_obstacle_vObs(avoidObstacle):
    new_vObs = 5.4

    avoidObstacle.update_obstacle(constructor_pObs, new_vObs)

    assert new_vObs == avoidObstacle.vObs

######################### update_robot #########################
def test_update_robot_pRobot(avoidObstacle):
    new_pRobot = 2.5

    avoidObstacle.update_robot(new_pRobot, constructor_vRobot)

    assert new_pRobot == avoidObstacle.pRobot

def test_update_robot_vRobot(avoidObstacle):
    new_vRobot = 7.3

    avoidObstacle.update_robot(constructor_pRobot, new_vRobot)

    assert new_vRobot == avoidObstacle.vRobot