import numpy as np
from math import pi, sin, cos


def apply_inverse_kinematics(vx_world, vy_world, theta, robot_orientation):
    wheel_radius = 0.027
    robot_radius = 0.09
    
    rotation_matrix = np.matrix([
        [cos(robot_orientation), sin(robot_orientation)],
        [-sin(robot_orientation), cos(robot_orientation)]
    ])
    
    # Transform world velocities to robot frame
    world_velocity = np.matrix([vx_world, vy_world]).T
    robot_velocity = rotation_matrix * world_velocity
    
    vx_robot = robot_velocity[0, 0]
    vy_robot = robot_velocity[1, 0]
    
    velocity = np.matrix([vx_robot, vy_robot, theta]).T
    
    wheel_angles = [pi * 5 / 6, pi * 5 / 4, pi * 7 / 4, pi / 6]
    
    jacobian = np.matrix([
        [cos(wheel_angles[0]), sin(wheel_angles[0]), robot_radius],
        [cos(wheel_angles[1]), sin(wheel_angles[1]), robot_radius],
        [cos(wheel_angles[2]), sin(wheel_angles[2]), robot_radius],
        [cos(wheel_angles[3]), sin(wheel_angles[3]), robot_radius],
    ])

    wheels_v = ((1 / wheel_radius) * (jacobian * velocity)).T.tolist()
    return wheels_v[0]


if __name__ == "__main__":
    print(apply_inverse_kinematics(3.0, 0.0, 0.0, 0.0))
