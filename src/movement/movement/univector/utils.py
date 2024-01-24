from math import pi, exp, sqrt
import sys

print(sys.path)

LEFT = 0
RIGHT = 1

def gaussian(x, u, v):
    '''
    Gaussian function
    '''
    return exp(-((x-u)**2) / (2 * (v**2)))

def wrap2pi(theta: float) -> float:
    '''
    Ensures that theta will always be between pi and -pi
    '''

    if theta > pi:
        return theta - 2 * pi
    if theta < -pi:
        return 2 * pi + theta
    else:
        return theta

def norm(vec):
    '''
    Never used because Vec2d has a norm method
    '''
    return sqrt(vec[0]**2 + vec[1]**2)