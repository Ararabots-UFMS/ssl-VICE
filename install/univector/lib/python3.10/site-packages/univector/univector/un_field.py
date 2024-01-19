from typing import List
import numpy as np
import math
from math import pi
from math import cos, sin, atan2, exp, sqrt
from utils.linalg import *

from strategy.arena_utils import ArenaSections, univector_pos_section, Axis, Offsets

LEFT = 0
RIGHT = 1

def gaussian(m, v):
    return exp(-(m**2) / (2 * (v**2)))

def wrap2pi(theta: float) -> float:
    if theta > pi:
        return theta - 2 * pi
    if theta < -pi:
        return 2 * pi + theta
    else:
        return theta

def norm(vec):
    return sqrt(vec[0]**2 + vec[1]**2)


class HyperbolicSpiral:

    def __init__(self, _Kr, _radius):
        self.Kr = _Kr
        self.radius = _radius

    def update_params(self, _KR: float, _RADIUS: float) -> None:
        self.Kr = _KR
        self.radius = _RADIUS

    def fi_h(self, _p: Vec2D, radius: float = None, cw: bool = True) -> float:

        if radius is None:
            r = self.radius
        else:
            r = radius

        p = _p
        theta = atan2(p[1], p[0])
        ro = p.norm()

        if ro > r:
            a = (pi / 2.0) * (2.0 - (r + self.Kr) / (ro + self.Kr))
        else:
            a = (pi / 2.0) * math.sqrt(ro / r)

        if cw:
            _theta = wrap2pi(theta + a)
        else:
            _theta = wrap2pi(theta - a)

        return atan2(sin(_theta), cos(_theta))

    def n_h(self, _p: Vec2D, _radius: float = None, cw: bool = True) -> Vec2D:
        p = _p
        if _radius is None:
            radius = self.radius
        else:
            radius = _radius

        fi = self.fi_h(p, radius, cw)
        return Vec2D(cos(fi), sin(fi))


class Repulsive:

    def __init__(self):
        self.origin = Vec2D.origin()

    def update_origin(self, newOrigin: Vec2D) -> None:
        self.origin = newOrigin.copy()

    def fi_r(self, _p, _origin: Vec2D = None, _theta: bool = True):
        if _origin is not None:
            self.update_origin(_origin)

        p = _p - self.origin

        if _theta:
            return atan2(p[1], p[0])
        else:
            return p


class Move2Goal:

    def __init__(self, _Kr: float, _radius: float):
        self.Kr = _Kr
        self.radius = _radius
        self.hyperSpiral = HyperbolicSpiral(self.Kr, self.radius)
        self.origin = Vec2D.origin()

        self.u = Vec2D.origin()
        self.v = Vec2D.origin()

        self.toUnivectorMatrix = None
        self.toCanonicalMatrix = None

    def update_params(self, _KR: float, _RADIUS: float) -> None:
        self.Kr = _KR
        self.radius = _RADIUS
        self.hyperSpiral.update_params(self.Kr, self.radius)

    def update_axis(self, new_origin: Vec2D, new_u_axis: Vec2D) -> None:
        self.origin = new_origin
        self.u = new_u_axis
        self.build_axis()

    def build_axis(self) -> None:
        self.u /= -self.u.norm()
        theta = math.atan2(self.u[1], self.u[0])
        self.v = Vec2D(-sin(theta), cos(theta))

        self.toCanonicalMatrix = Mat2D(self.u, self.v)
        self.toUnivectorMatrix = self.toCanonicalMatrix.invert()

    def fi_tuf(self, _p: Vec2D) -> float:
        n_h = self.hyperSpiral.n_h
        p = _p - self.origin
        r = self.radius

        p = self.toUnivectorMatrix.transform(p)

        x, y = p
        yl = y + r
        yr = y - r

        # Parece que houve algum erro de digitacao no artigo
        # Pois quando pl e pr sao definidos dessa maneira o campo gerado
        # se parece mais com o resultado obtido no artigo
        pl = Vec2D(x, yr)
        pr = Vec2D(x, yl)

        # Este caso eh para quando o robo esta dentro do "circulo" da bola
        if -r <= y < r:
            nh_pl = n_h(pl, cw=False)
            nh_pr = n_h(pr, cw=True)

            # Apesar de no artigo nao ser utilizado o modulo, quando utilizado
            # na implementacao o resultado foi mais condizente com o artigo
            vec = (abs(yl) * nh_pl + abs(yr) * nh_pr) / (2.0 * r)
            vec = self.toCanonicalMatrix.transform(vec)
        else:
            if y < -r:
                theta = self.hyperSpiral.fi_h(pl, cw=True)
            else:  # y >= r
                theta = self.hyperSpiral.fi_h(pr, cw=False)

            vec = Vec2D(cos(theta), sin(theta))
            # TODO: MATRIZES
            vec = self.toCanonicalMatrix.transform(vec)

        return atan2(vec[1], vec[0])


class AvoidObstacle:
    def __init__(self, _pObs: Vec2D, _vObs: Vec2D, _pRobot: Vec2D, _vRobot: Vec2D, _K0: float):
        self.pObs =_pObs.copy()
        self.vObs = _vObs.copy()
        self.pRobot = _pRobot.copy()
        self.vRobot = _vRobot.copy()
        self.K0 = _K0
        self.repField = Repulsive()

    def get_s(self) -> Vec2D:
        return self.K0 * (self.vObs - self.vRobot)

    def get_virtual_pos(self) -> Vec2D:
        s = self.get_s()
        s_norm = s.norm()
        d = (self.pObs - self.pRobot).norm()
        if d >= s_norm:
            v_pos = self.pObs + s
        else:
            v_pos = self.pObs + (d / s_norm) * s
        return v_pos

    def fi_auf(self, _robotPos: Vec2D, _vPos: Vec2D = None, _theta: bool = True) -> Vec2D:
        if _vPos is None:
            v_pos = self.get_virtual_pos()
        else:
            v_pos = _vPos
        vec = self.repField.fi_r(_robotPos, _origin=v_pos, _theta=_theta)
        return vec

    def update_param(self, _K0: float) -> None:
        self.K0 = _K0

    def update_obstacle(self, _pObs: Vec2D, _vObs: Vec2D) -> None:

        self.pObs = _pObs
        self.vObs = _vObs

    def update_robot(self, _pRobot: Vec2D, _vRobot: Vec2D) -> None:

        self.pRobot = _pRobot
        self.vRobot = _vRobot


class UnivectorField:
    def __init__(self):
        self.obstacles = [Vec2D.origin()]
        self.obstaclesSpeed = [Vec2D.origin()]
        self.ballPos = Vec2D.origin()
        self.robotPos = Vec2D.origin()
        self.vRobot = Vec2D.origin()
        # Field constants
        self.RADIUS = None
        self.KR = None
        self.K0 = None
        self.DMIN = None
        self.LDELTA = None

        # Subfields
        self.avdObsField = AvoidObstacle(Vec2D.origin(), Vec2D.origin(),
                                         Vec2D.origin(), Vec2D.origin(), self.K0)

        self.mv2Goal = Move2Goal(self.KR, self.RADIUS)

    @staticmethod
    def get_attack_goal_axis(attack_goal: bool) -> Vec2D:
        if attack_goal == LEFT:
            return Vec2D.left()
        else:
            return Vec2D.right()

    @staticmethod
    def get_attack_goal_position(attack_goal: bool) -> Vec2D:
        """
        Return the position of the goal, given attacking side  and section of the object
        :param team_side: int
        :return: np.array([x,y])
        """
        return Vec2D(attack_goal * 150, 65)

    def update_obstacles(self, _obstacles: List[Vec2D], _obsSpeeds: List[Vec2D]) -> None:
        self.obstacles = _obstacles
        self.obstaclesSpeed = _obsSpeeds

    def update_robot(self, _robotPos: Vec2D, _vRobot: Vec2D) -> None:

        self.robotPos = _robotPos
        self.vRobot = _vRobot
        self.avdObsField.update_robot(self.robotPos, self.vRobot)

    def update_constants(self, _RADIUS: float, _KR: float, _K0: float, _DMIN: float, _LDELTA: float) -> Vec2D:
        self.RADIUS = _RADIUS
        self.KR = _KR
        self.K0 = _K0
        self.DMIN = _DMIN
        self.LDELTA = _LDELTA

        self.avdObsField.update_param(self.K0)
        self.mv2Goal.update_params(self.KR, self.RADIUS)

    def get_angle_vec(self, _robotPos: Vec2D = None, _vRobot: Vec2D = None,
                      _goal_pos: Vec2D = None, _goal_axis=None) -> float:

        if _robotPos is not None and _vRobot is not None:
            # Just in case the user send lists
            robot_pos = _robotPos
            v_robot = _vRobot
            self.update_robot(robot_pos, v_robot)

        if _goal_pos is not None and _goal_axis is not None:
            goal_position = _goal_pos
            goal_axis = _goal_axis
            self.mv2Goal.update_axis(goal_position, goal_axis)

        closest_center = None  # array to store the closest center
        centers = []
        min_distance = self.DMIN + 1

        if len(self.obstacles) > 0:
            # get the Repulsive field centers
            a = len(self.obstacles)
            for i in range(len(self.obstacles)):
                try:
                    self.avdObsField.update_obstacle(self.obstacles[i], self.obstaclesSpeed[i])
                    center = self.avdObsField.get_virtual_pos()
                    centers.append(center)

                except Exception as e:
                    pass
                    # import rospy
                    # #TODO: bug -> número de obstáculos é alterado dentro do laço...
                    # # problema de concorrência?
                    # rospy.logfatal(f"obj antes: {a} // obj atuais: {len(self.obstacles)}")

            # centers = centers
            # dist_vec = np.linalg.norm(np.subtract(centers, self.robotPos), axis=1)
            dist_vec = [(center - self.robotPos).norm() for center in centers]
            index = np.argmin(dist_vec)  # index of closest center
            closest_center = centers[index]
            min_distance = dist_vec[index]

            fi_auf = self.avdObsField.fi_auf(self.robotPos, _vPos=closest_center, _theta=True)

        # the first case when the robot is to close from an obstacle
        if min_distance <= self.DMIN:
            return fi_auf
        else:
            fi_tuf = self.mv2Goal.fi_tuf(self.robotPos)
            # Checks if at least one obstacle exist
            if len(self.obstacles) > 0:
                g = gaussian(min_distance - self.DMIN, self.LDELTA)
                diff = wrap2pi(fi_auf - fi_tuf)
                return wrap2pi(g * diff + fi_tuf)
            else:  # if there is no obstacles
                return fi_tuf

    def get_vec_with_ball(self, _robotPos: Vec2D = None,
                          _vRobot: Vec2D = None,
                          _ball: Vec2D = None,
                          _attack_goal: bool = RIGHT) -> Vec2D:
        angle = self.get_angle_with_ball(_robotPos, _vRobot, _ball, _attack_goal)
        return Vec2D(cos(angle), sin(angle))

    def get_angle_with_ball(self, _robotPos: Vec2D = None,
                            _vRobot: Vec2D = None,
                            _ball: Vec2D = None,
                            _attack_goal: bool = RIGHT) -> float:

        section_num = univector_pos_section(_ball)

        if section_num == ArenaSections.CENTER:
            correct_axis = self.get_attack_goal_position(_attack_goal) - _ball
        else:
            if _attack_goal == RIGHT:
                if section_num == ArenaSections.RIGHT_DOWN_CORNER or section_num == ArenaSections.RIGHT_UP_CORNER:
                    correct_axis = Vec2D.right()
                else:
                    correct_axis = self.get_correct_axis(_ball, section_num, _attack_goal)                    
            else:
                if section_num == ArenaSections.LEFT_DOWN_CORNER or section_num == ArenaSections.LEFT_UP_CORNER:
                    correct_axis = Vec2D.left()
                else:
                    correct_axis = self.get_correct_axis(_ball, section_num, _attack_goal)                 
                

        offset = self.get_correct_offset(_ball, section_num)

        return self.get_angle_vec(_robotPos, _vRobot, _ball, correct_axis - offset)

    def get_correct_axis(self, position: Vec2D, section_num: ArenaSections,
                         attack_goal: bool = RIGHT) -> Vec2D:

        axis = Axis[section_num.value]

        if attack_goal == LEFT:
            axis = axis * -1

        return axis

    def get_correct_offset(self, position: Vec2D, section_num: ArenaSections) -> Vec2D:
        return Offsets[section_num.value]
