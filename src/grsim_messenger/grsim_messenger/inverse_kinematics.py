
import numpy as np
from math import pi, sin, cos

def apply_inverse_kinematics(vx, vy, theta, offset):
    velocity = np.matrix([vx, vy, theta]).T

    wheel_radius = 0.027
    robot_radius = 0.09

    wheel_angles = [pi/6, pi*5/6, pi*5/4, pi*7/4]

    jacobian = np.matrix([[cos(wheel_angles[0] + offset), sin(wheel_angles[0] + offset), robot_radius],
                          [cos(wheel_angles[1] + offset), sin(wheel_angles[1] + offset), robot_radius],
                          [cos(wheel_angles[2] + offset), sin(wheel_angles[2] + offset), robot_radius],
                          [cos(wheel_angles[3] + offset), sin(wheel_angles[3] + offset), robot_radius]])

    wheels_v = ((1/wheel_radius)*(jacobian*velocity)).T.tolist()

    return wheels_v[0][1], wheels_v[0][2], wheels_v[0][3], wheels_v[0][0]