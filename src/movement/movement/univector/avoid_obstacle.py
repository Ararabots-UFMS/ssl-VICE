from movement.univector.repulsive import Repulsive
from utils.linalg import *

class AvoidObstacle:
    '''
    Uses the repulsive field to create the avoid obstacle field used in the resulting univector field
    '''
    
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

    def fi_auf(self, _robotPos: Vec2D, _vPos: Vec2D = None) -> Vec2D:
        if _vPos is None:
            v_pos = self.get_virtual_pos()
        else:
            v_pos = _vPos
        vec = self.repField.fi_r(_robotPos, _origin=v_pos)
        return vec

    def update_param(self, _K0: float) -> None:
        self.K0 = _K0

    def update_obstacle(self, _pObs: Vec2D, _vObs: Vec2D) -> None:

        self.pObs = _pObs
        self.vObs = _vObs

    def update_robot(self, _pRobot: Vec2D, _vRobot: Vec2D) -> None:

        self.pRobot = _pRobot
        self.vRobot = _vRobot