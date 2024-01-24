from movement.univector.repulsive import Repulsive
from utils.linalg import *

class AvoidObstacle:
    '''
    AvoidObstacle field class, uses Repulsive field to form the resulting univector field.
    
    Args:
        pObs [Vec2D]: Obstacle position.
        vObs [Vec2D]: Obstacle velocity vector.
        pRobots [Vec2D]: Robots position.
        vRobots [Vec2D]: Robots velocity vector.
        K0 [float]: virtual prediction costant.
    '''
    
    def __init__(self, pObs: Vec2D, vObs: Vec2D, pRobot: Vec2D, vRobot: Vec2D, K0: float):
        self.pObs = pObs.copy()
        self.vObs = vObs.copy()
        self.pRobot = pRobot.copy()
        self.vRobot = vRobot.copy()
        self.K0 = K0
        self.repField = Repulsive()

    def get_s(self) -> Vec2D:
        '''
        's' is the vector from the real obstacle position to the predicted (virtual) obstacle position.
        '''
        return self.K0 * (self.vObs - self.vRobot)

    def get_virtual_pos(self) -> Vec2D:
        s = self.get_s()
        s_norm = s.norm()
        d = (self.pObs - self.pRobot).norm()

        # To prevent a collision if the real obstacle position is in front of the robot
        # but the virtual position is behind.
        v_pos = self.pObs
        v_pos += s if d >= s_norm else (d / s_norm) * s
        
        return v_pos

    def fi_auf(self, _robotPos: Vec2D, _vPos: Vec2D = None) -> Vec2D:
        v_pos = _vPos if _vPos is not None else self.get_virtual_pos()
        
        vec = self.repField.fi_r(_robotPos, _origin = v_pos)
        return vec

    def update_param(self, K0: float) -> None:
        '''
        Update K0 parameter method
        '''
        self.K0 = K0

    def update_obstacle(self, pObs: Vec2D, vObs: Vec2D) -> None:
        '''
        Update obstacle position and velocity vector.
        '''
        self.pObs = pObs
        self.vObs = vObs

    def update_robot(self, pRobot: Vec2D, vRobot: Vec2D) -> None:
        '''
        Update robots position and velocity vector.
        '''
        self.pRobot = pRobot
        self.vRobot = vRobot