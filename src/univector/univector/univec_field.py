from typing import List
import numpy as np
from math import cos, sin
from utils.linalg import *
from .components.move2goal import Move2Goal
from components.avoid_obstacle import AvoidObstacle
from components.utils import wrap2pi, gaussian, LEFT, RIGHT

from strategy.arena_utils import ArenaSections, univector_pos_section, Axis, Offsets

class UnivectorField:
    '''
    Uses the move2goal and avoid obstacle fields to create the univector field
    '''

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
                g = gaussian(min_distance, self.DMIN, self.LDELTA)
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
