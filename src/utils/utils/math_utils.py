from typing import Union
from utils.linalg import Vec2D
import numpy as np
import numpy.linalg as la
import math
import scipy.stats as stats
import random

MINCHANGE = 0.5

FORWARD = True
BACKWARDS = not FORWARD

RAD2DEG = 180.0/math.pi
DEG2RAD = 1.0/RAD2DEG

def unitVector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


# def angle_between(v1, v2, absol:bool= True) -> float:
#     """ Returns the angle in radians between vectors 'v1' and 'v2' """

#     # TODO: Refatorar esta função para usar o Vec2D... o que abs significa?
#     # Gambito por enquanto
#     if isinstance(v1, Vec2D): v1 = np.array(v1.to_list())
#     if isinstance(v2, Vec2D): v2 = np.array(v2.to_list())

#     cosang = np.dot(v1, v2)
#     sinang = np.cross(v1, v2)
#     if absol:
#         sinang = la.norm(np.cross(v1, v2))
#     return np.arctan2(sinang, cosang)  # atan2(y, x) or atan2(sin, cos)

def angle_between(v1: Vec2D, v2: Vec2D, absol: bool= True) -> float:
    """ Returns the angle in radians between vectors 'v1' and 'v2' """
    v1 = Vec2D(v1[0],v1[1])
    v2 = Vec2D(v2[0],v2[1])
    cosang = v1.dot(v2)
    sinang = v1.cross(v2)
    if absol:
        sinang = abs(sinang)
    return math.atan2(sinang, cosang)  # atan2(y, x) or atan2(sin, cos)



def rotateVector(x, angle):
    """Rotate vector x anticlockwise around the origin by angle radians, return angle in format [x, y]"""
    y1 = math.cos(angle)*x[0] - math.sin(angle)*x[1]
    y2 = math.sin(angle)*x[0] + math.cos(angle)*x[1]
    return [y1, y2]


def rotatePoint(origin, point, angle):
    """Rotate a point counterclockwise by a given angle around a given origin.
    The angle should be given in radians."""
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return (int(qx),int(qy))


def distancePoints(a, b):
    """Distance between two points"""
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5

def clamp(value, min, max):
    if value < min:
        return min
    elif value > max:
        return max
    
    return value


def maxAbs(x, y):
    """Return the maximum absolute value"""
    return max(abs(x), abs(y))


def gaussian(m, v):
    return math.exp(-(m**2) / (2 * (v**2)))


def opposite_vector(vec):
    """
    Return the opposite vector
    :param vec: [float, float]
    :return: [float, float]
    """
    return [-vec[0], -vec[1]]


def min_diff_vec_and_opposite(num, orientation, vec, goal):
    """
    Return true if the angle difference between vec and goal is less then opposite vec and goal
    :param num: int
    :param orientation: boolean
    :param vec: [float, float]
    :param goal: [float, float]
    :return: boolean, int
    """
    rand = random.random()
    gamma_value = stats.gamma.cdf(num, a=10, scale=0.8)
    if rand < gamma_value:
        if angle_between(vec, goal) <= angle_between(opposite_vector(vec), goal):
            return True, 0
        return False, 0
    else:
        return orientation, num+1


def forward_min_diff(num, orientation, vec, goal, only_forward=False):
    """
     Return True if forward and the min difference angle
    :param num: int
    :param orientation: boolean
    :param vec: [float, float]
    :param goal: [float, float]
    :param only_forward: boolean
    :return: boolean, float, int
    """
    tmp, new_gamma_count = min_diff_vec_and_opposite(num, orientation, vec, goal)
    if tmp or only_forward:
        return True, angle_between(vec, goal, absol=False), new_gamma_count
    return False, angle_between(opposite_vector(vec), goal, absol=False), new_gamma_count

def raio_vetores(p1, v1, p2, v2, speed_max=255, upper_bound=800, angle = 3,k = 0.01):
    p1 = np.array(p1)
    p2 = np.array(p2)
    v1 = np.array(v1)
    v2 = np.array(v2)
    ret = upper_bound

    cos = abs(np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))
    r1 = 2*(1-cos)
    d = np.linalg.norm(p1-p2)
    if (cos < 1 - angle):
        ret = 10/(np.sqrt(float(r1*k)))
    
    return (ret/upper_bound) * speed_max

def get_orientation_and_angle(orientation, robot_vector, goal_vector,do_nothing_angle = 30):
    cos = np.dot(robot_vector,goal_vector)/(np.linalg.norm(robot_vector)*np.linalg.norm(goal_vector))

    cos_do_nothing = np.cos((np.pi/180.0)*(90-do_nothing_angle))
    #(90-do_nothing_angle/2.0))
    #transforma as variáveis abaixo em Vec2D para usar em angle_between
    robot_vector = Vec2D(robot_vector[0], robot_vector[1])
    goal_vector = Vec2D(goal_vector[0], goal_vector[1])
    theta = angle_between(robot_vector, goal_vector, absol=False)

    if abs(cos) < cos_do_nothing:
        return orientation, theta# maintains the same
    else:
        if cos > 0 and  cos > cos_do_nothing:
            return True, theta # backward
        else:
            return False, theta # forward


def predict_speed(robot_position, robot_vector, ball_position, atack_goal,  speed_max=255, upper_bound=800,k = 0.01,range_limit = 10):
    ret = upper_bound
    robot_position = np.array(robot_position)
    robot_vector   = np.array(robot_vector)
    ball_position  = np.array(ball_position)
    atack_goal     = np.array(atack_goal)
    pup   = np.array([atack_goal[0],atack_goal[1]-range_limit])
    pdown = np.array([atack_goal[0],atack_goal[1]+range_limit])
    vup   = ball_position - pup
    vdown = ball_position - pdown
    direction = robot_vector
    cos_up    = abs(np.dot(direction,vup)/(np.linalg.norm(direction)*np.linalg.norm(vup)))
    cos_down  = abs(np.dot(direction,vdown)/(np.linalg.norm(direction)*np.linalg.norm(vdown)))
    cos_both  = abs(np.dot(vdown,vup)/(np.linalg.norm(vup)*np.linalg.norm(vdown)))
    r1 = 2*(1-min(cos_up,cos_down))
    if (r1>0):
        if (cos_up < cos_both or cos_down < cos_both ):
            ret = 10/(np.sqrt(float(r1*k)))
    return (ret/upper_bound) * speed_max


def min_angle(ang1, ang2, rad: bool = True) -> Union[int, float]:
    if rad == True:
        ang1 *= RAD2DEG
        ang2 *= RAD2DEG
    
    angle_degrees = (ang1 + 180 -  ang2) % 360 - 180

    return -angle_degrees*DEG2RAD if rad == True else -angle_degrees 
    
def wrap2pi(theta):
    if theta > math.pi:
        return theta - 2*math.pi
    if theta < -math.pi:
        return 2*math.pi + theta
    else:
        return theta