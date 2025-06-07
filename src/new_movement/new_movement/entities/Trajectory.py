from States import State, Vector2D, MoveConstraints
from Motion import MotionPath
from TrajGenerator import TrajectoryGenerator
from typing import Optional


class TrajectorySegment():
    def __init__(self, initPos: Vector2D, initVel: Vector2D, motionPath: MotionPath) -> None:
        self.initPos = initPos
        self.initVel = initVel
        self.motionPath = motionPath

    def get_state(self, t) -> State:
        pass

    def get_destination(self) -> State:
        pass

    def get_total_time(self) -> float:
        pass

class Trajectory():
    def __init__(self):
        pass

    def append(self, t) -> None:
        pass

    def connect(self, t) -> None:
        pass

    def relocate(self):
        pass

    def get_state(self, t) -> State:
        pass
    
    def get_position(self, t) -> Vector2D:
        pass

    def get_velocity(self, t) -> Vector2D:
        pass

    def get_acceleration(self, t) -> Vector2D:
        pass

    def get_total_time(selfE) -> float:
        pass