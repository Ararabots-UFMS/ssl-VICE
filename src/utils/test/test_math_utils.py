import math
import pytest
from utils.linalg import Vec2D
from utils.math_utils import unitVector, angle_between, rotateVector, rotatePoint, distancePoints, clamp, maxAbs, gaussian, opposite_vector, min_diff_vec_and_opposite, forward_min_diff, raio_vetores, get_orientation_and_angle, predict_speed, min_angle, wrap2pi

################## unitVector ###################
@pytest.mark.parametrize(
    "vector, expected_vector",[
        ([2,3],[0.5547002,0.83205029]),
    ])

def test_unitVector(vector, expected_vector):
    result = unitVector(vector)
    assert math.isclose(result[0], expected_vector[0], rel_tol=1e-7)
    assert math.isclose(result[1], expected_vector[1], rel_tol=1e-7)

################## angle_between ################
@pytest.mark.parametrize(
    "v1,    v2,     absol,   expected_angle",[
        (Vec2D(3.0,4.0),    Vec2D(1.0,5.0),     True,   0.4461055489), #test with random numbers
        (Vec2D(5.0,1.0),    Vec2D(10.0,2.0),    True,   0.0),                #angulo = 0
        (Vec2D(1.0,0.0),    Vec2D(0.0,1.0),     True,   1.5707963267), #angulo = 90
        (Vec2D(1.0,0.0),    Vec2D(-1.0,0.0),    True,   3.1415926535), #angulo = 180
        (Vec2D(-1.0,5.0),   Vec2D(1.0,0.0),     True,   1.7681918866), #angulo 
    ])

def test_angle_between(v1, v2, absol, expected_angle):
    result = angle_between(v1,v2,absol)
    assert math.isclose(result, expected_angle, rel_tol=1e-9)

################## rotateVector #################
#Resultados incompatíveis (degree ou radians)
# -> funciona com radianos
@pytest.mark.parametrize(
    "x, angle, expected_vector",[
        (Vec2D(3.0,7.0),    90,     Vec2D(-7.602197493591415,-0.4545253221025174)),
        (Vec2D(3.0,7.0),    45,     Vec2D(-4.380358705285641, 6.229964495326463)),
        (Vec2D(-2.0, 5.0),  90,     Vec2D(-3.5738360857444493, -4.028361407846966)),
    ])

def test_rotateVector(x, angle, expected_vector):
    result = rotateVector(x, angle)
    assert result[0]==expected_vector[0] and result[1]==expected_vector[1]

################## rotatePoint ##################
@pytest.mark.parametrize(
    "oring, point, angle, expected_point", [
        ([0,0],     [3,7],      1.5708,     [-7,2]),
    ])

def test_rotatePoint(oring, point, angle, expected_point):
    result = rotatePoint(oring, point, angle)
    assert result[0]==expected_point[0] and result[1]==expected_point[1]
################## distancePoints ###############
@pytest.mark.parametrize(
    "point_a, point_b, expected_distance",[
        ([0,0],[1,1],1.4142135623730951),
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
    ])

def test_maxAbs(x, y, expected_max_absolute):
    result = maxAbs(x,y)
    assert result==expected_max_absolute

################# gaussian ################
@pytest.mark.parametrize(
    "m, v, expected_result",[
        (0, 1,  1),
        (2, 0.5,    0.00033546262790251185),
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
    ])

def test_opposite_vector(vec,expected_vec):
    result = opposite_vector(vec)
    assert math.isclose(result[0],expected_vec[0],rel_tol=1e-9) and math.isclose(result[1],expected_vec[1],rel_tol=1e-9)

################# min_diff_vec_and_opposite ##########################
@pytest.mark.parametrize(
    "num, orientation, vec, goal, expected_boolean, expected_int",[
        (6,True,Vec2D(1,1),Vec2D(2,3),True,7),
    ])

def test_min_diff_vec_and_opposite(num, orientation, vec, goal, expected_boolean, expected_int):
    result = min_diff_vec_and_opposite(num, orientation, vec, goal)
    assert result[0]==expected_boolean
    assert result[1]==expected_int

################# forward_min_diff ###########################
@pytest.mark.parametrize(
    "num, orientation, vec, goal, expected_boolean, expected_float, expected_int",[
        (6,True,Vec2D(1,1),Vec2D(2,3),True,0.19739555984, 7)
    ])

def test_forward_min_diff(num, orientation, vec, goal, expected_boolean, expected_float, expected_int):
    only_forward = False
    result = forward_min_diff(num, orientation, vec, goal, only_forward)
    assert result[0]==expected_boolean
    assert math.isclose(result[1],expected_float,rel_tol=1e-9)
    assert result[2]==expected_int

################# raio_vetores #########################
    
@pytest.mark.parametrize(
    "p1, v1, p2, v2, expected_ratio",[
        ([1,2], [3,2],  [5,1],  [2,3], 255.0)
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
            (True, [1,1], [2,5], 30, True, 0.40489178628)
        ])

def test_get_orientation_and_angle(orientation, robot_vector, goal_vector, do_nothing_angle, expected_bool,expected_theta):
    result = get_orientation_and_angle(orientation, robot_vector, goal_vector, do_nothing_angle)
    assert result[0]==expected_bool
    assert math.isclose(result[1],expected_theta,rel_tol=1e-9)
################# predict_speed #######################
#Resultado não está batendo com o teste!!!!!!!!!!
@pytest.mark.parametrize(
    "robot_position, robot_vector, ball_position, atack_goal, expected_result",[
        ([1,1],[1,1],[1,1],[1,1],41.646694505434496)
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
        (90,180,True,2.0354056994857),
        (90,90,True,0.0),
        (180,30,True,0.7964473723)
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