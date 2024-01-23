import pytest
from univector.components.avoid_obstacle import AvoidObstacle
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

######################### fi_auf #########################

######################### update_param #########################

######################### update_obstacle #########################

######################### update_robot #########################