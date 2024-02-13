import math
import pytest
from utils.linalg import Vec2D
from utils.math_utils import *

################## unitVector ###################
@pytest.mark.parametrize(
    "vector, expected_vector",[
        ([2,3],[0.5547002, 0.8320502]),
        ([1,1],[0.7071067, 0.7071067]),
        ([-3,5],[-0.5144957, 0.8574929]),
        ([4,-7],[0.4961389, -0.8682431]),
        ([-2,-5],[-0.3713906, -0.9284766])
    ])

def test_unitVector(vector, expected_vector):
    result = unitVector(vector)
    assert math.isclose(result[0], expected_vector[0], rel_tol=1e-6)
    assert math.isclose(result[1], expected_vector[1], rel_tol=1e-6)

################## angle_between ################
@pytest.mark.parametrize(
    "v1,    v2,     absol,   expected_angle",[
        (Vec2D(3.0,4.0),    Vec2D(1.0,5.0),     True,   0.4461055489), 
        (Vec2D(5.0,1.0),    Vec2D(10.0,2.0),    True,   0.0),          
        (Vec2D(1.0,0.0),    Vec2D(0.0,1.0),     True,   1.5707963267), 
        (Vec2D(1.0,0.0),    Vec2D(-1.0,0.0),    True,   3.1415926535), 
        (Vec2D(-1.0,5.0),   Vec2D(1.0,0.0),     True,   1.7681918866),  
    ])

def test_angle_between(v1, v2, absol, expected_angle):
    result = angle_between(v1,v2,absol)
    assert math.isclose(result, expected_angle, rel_tol=1e-9)

################## rotateVector #################
@pytest.mark.parametrize(
    "x, angle, expected_vector",[
        (Vec2D(3.0,7.0),    1.5708,     Vec2D(-7.0000110195680865, 2.999974287544038)),
        (Vec2D(3.0,7.0),    0.785398,     Vec2D(-2.828425969351715, 7.071068274023156)),
        (Vec2D(-2.0, 5.0),  1.5708,     Vec2D(-4.999992653556062, -2.000018366012024)),
        (Vec2D(6.0,-3.0),   0.523599,   Vec2D(6.696152332514082, 0.40192529127466914)),
        (Vec2D(-5.0,-2.0),  3.14159, Vec2D(5.000005307161983, 1.9999867320439917))
    ])

def test_rotateVector(x, angle, expected_vector):
    result = rotateVector(x, angle)
    assert result[0]==expected_vector[0] and result[1]==expected_vector[1]

################## rotatePoint ##################
@pytest.mark.parametrize(
    "oring, point, angle, expected_point", [
        ([0,0],     [3,7],      1.5708,     [-7,2]),
        ([2,3],    [3,7],  1.5708,  [-2,3]),
        ([-2,5],    [-3,1],  0.785398,  [0,1]),
        ([5,-3],    [2,-4],  0.523599,  [2,-5]),
        ([-5,-9],    [-2,-5],  3.14159, [-8, -12]),
    ])

def test_rotatePoint(oring, point, angle, expected_point):
    result = rotatePoint(oring, point, angle)
    assert result[0]==expected_point[0] and result[1]==expected_point[1]
################## distancePoints ###############
@pytest.mark.parametrize(
    "point_a, point_b, expected_distance",[
        ([0,0],    [1,1],  1.4142135623730951),
        ([1,2],    [4,3],  3.1622776601683795),
        ([-3,2],   [4,-7], 11.40175425099138),
        ([-4,-2],  [8,3],  13.0),
        ([-12,-9],  [-1,-3],    12.529964086141668)
    ])

def test_distancePoints(point_a, point_b, expected_distance):
    result = distancePoints(point_a,point_b)
    assert result==expected_distance

################## clamp ##################
@pytest.mark.parametrize(
    "value, min, max, expected_value",[
        (10, 5, 20, 10),
        (2, 5, 20, 5),
        (30, 5, 20, 20),
        (-10,-5,-7,-5),
        (-5,-10,-7,-7)
    ])

def test_clamp(value, min, max, expected_value):
    result = clamp(value, min, max)
    assert result==expected_value

################# maxAbs ##################
@pytest.mark.parametrize(
    "x, y, expected_max_absolute",[
        (10, -20, 20),
        (15, 5, 15),
        (-20, -30, 30),
        (2,-2,2),
        (-15,10,15)
    ])

def test_maxAbs(x, y, expected_max_absolute):
    result = maxAbs(x,y)
    assert result==expected_max_absolute

################# gaussian ################
@pytest.mark.parametrize(
    "m, v, expected_result",[
        (0, 1,  1),
        (2, 0.5,    0.00033546262790251185),
        (3, 1,  0.011108996538242306),
        (1, 0.5,    0.1353352832366127),
        (2, 3,  0.8007374029168081)
    ])

def test_gaussian(m, v, expected_result):
    result = gaussian(m, v)
    assert math.isclose(result, expected_result, rel_tol=1e-9)

################# opposite_vector ################
@pytest.mark.parametrize(
    "vec, expected_vec",[
        ([1,2], [-1,-2]),
        ([-3,4],    [3,-4]),
        ([-5,-6],   [5,6]),
        ([7,-9],    [-7,9]),
        ([-3,3], [3,-3])
    ])

def test_opposite_vector(vec,expected_vec):
    result = opposite_vector(vec)
    assert math.isclose(result[0],expected_vec[0],rel_tol=1e-9) and math.isclose(result[1],expected_vec[1],rel_tol=1e-9)

################# min_diff_vec_and_opposite ##########################
@pytest.mark.parametrize(
    "num, orientation, vec, goal, expected_boolean, expected_int",[
        (6,True,Vec2D(1,1),Vec2D(2,3),True, 7),
        (7, True, Vec2D(3,4), Vec2D(-2,5), True, 8),
        (5, False, Vec2D(-2,1), Vec2D(5,3), False, 6),
        (3, False, Vec2D(-4,-3), Vec2D(7,2), False, 4),
        (3, False, Vec2D(-7,-7), Vec2D(7,7), False, 4)
    ])

def test_min_diff_vec_and_opposite(num, orientation, vec: Vec2D, goal: Vec2D, expected_boolean, expected_int):
    result = min_diff_vec_and_opposite(num, orientation, vec, goal)
    assert result[0]==expected_boolean
    assert (result[1]==expected_int)or(result[1]==0)


################# forward_min_diff ###########################
@pytest.mark.parametrize(
    "num, orientation, vec, goal, only_forward, expected_float, expected_int",[
        ( 6, True, Vec2D(1,1), Vec2D(2,3), False, 0.19739555984, 7),
        ( 3, True, Vec2D(2,4), Vec2D(-3,2), False, 1.446441332248135, 4),
        ( 5, False, Vec2D(3,1), Vec2D(7,-3), False, 2.4149503129080676, 6),
        ( -7, True, Vec2D(-4,2), Vec2D(3,5), False, -1.6475682180646747, -6),
        ( -2, False, Vec2D(7,4), Vec2D(3,3), False, -2.875340604438868, -1)
    ])

def test_forward_min_diff(num, orientation, vec, goal, only_forward, expected_float, expected_int):
    result = forward_min_diff(num, orientation, vec, goal, only_forward)
    assert math.isclose(result[1],expected_float,rel_tol=1e-9)
    assert (result[2]==expected_int)or(result[2]==0)

################# raio_vetores #########################   
@pytest.mark.parametrize(
    "p1, v1, p2, v2, expected_ratio",[
        ([1,2], [3,2],  [5,1],  [2,3], 255.0),
        ([-2,-5], [3,1], [4,6], [7,2], 255),
        ([-1,-2], [-3,-2], [5,1],   [2,3], 255),
        ([-2,-5], [-3,-1], [-4,-6], [7,2], 255),
        ([-1,-2], [-3,-2], [-5,-1], [-2,-3], 255)
    ])

def test_raio_vetores(p1, v1, p2, v2, expected_ratio):
    speed_max=255
    upper_bound=800
    angle = 3
    k = 0.01
    result = raio_vetores(p1, v1, p2, v2, speed_max, upper_bound, angle, k)
    assert result==expected_ratio

################# get_orientation_and_angle #####################
@pytest.mark.parametrize(
        "orientation, robot_vector, goal_vector, do_nothing_angle, expected_bool, expected_theta",[
            (True, [1,1], [2,5], 30, True, 0.40489178628),
            (True, [2,3], [-4,1], 30, True, 1.9138202672156),
            (False, [-7,2], [-5,0], 30, True, 0.27829965900511133),
            (True, [1,4], [-3,3], 30, True, 1.0303768265243125),
            (False, [-5,-3], [7,5], 30, False, -3.061762667877556)
        ])

def test_get_orientation_and_angle(orientation, robot_vector, goal_vector, do_nothing_angle, expected_bool,expected_theta):
    result = get_orientation_and_angle(orientation, robot_vector, goal_vector, do_nothing_angle)
    assert result[0]==expected_bool
    assert math.isclose(result[1],expected_theta,rel_tol=1e-9)
################# predict_speed #######################
@pytest.mark.parametrize(
    "robot_position, robot_vector, ball_position, atack_goal, expected_result",[
        ([1,1],[1,1],[1,1],[1,1],41.646694505434496),
        ([3,2],[1,5],[-5,2],[6,-7], 26.638236759089267),
        ([4,6],[2,7],[9,3],[3,1], 255.0),
        ([-6,-3],[-3,-2],[6,1],[7,5], 29.352388661825767),
        ([9,11],[4,-2],[-7,3],[6,2], 255.0)
    ])

def test_predict_speed(robot_position, robot_vector, ball_position, atack_goal, expected_result):
    speed_max=255
    upper_bound=800
    k = 0.01
    range_limit = 10
    result = predict_speed(robot_position, robot_vector, ball_position, atack_goal, speed_max, upper_bound, k, range_limit)
    assert result==expected_result

################# min_angle ########################
@pytest.mark.parametrize(
    "ang1, ang2, rad, expected_float",[
        (90, 180, True, 2.0354056994857),
        (90, 90, True, 0.0),
        (180, 30, True, 0.7964473723),
        (45, 60, False, 15),
        (180, 30, False, -150)
    ])
def test_min_angle(ang1, ang2, rad, expected_float):
    result = min_angle(ang1, ang2, rad)
    assert math.isclose(result,expected_float, rel_tol=1e-9)
################# wrap2pi ######################
@pytest.mark.parametrize(
    "theta, expected_theta",[
        (30,23.7168146928),
        (45,38.7168146928),
        (60,53.7168146928),
        (90,83.7168146928),
        (180,173.7168146928)
    ])
def test_wrap2pi(theta, expected_theta):
    result = wrap2pi(theta)
    assert math.isclose(result,expected_theta,rel_tol=1e-9)