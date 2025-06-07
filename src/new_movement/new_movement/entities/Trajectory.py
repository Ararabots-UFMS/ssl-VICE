from States import State, Vector2D, MoveConstraints
from Motion import MotionPath
from TrajGenerator import TrajectoryGenerator
from typing import Optional
from utils.BB_steer import integrate_control_2d as integrate


class TrajectorySegment:
    def __init__(self, initPos: Vector2D, initVel: Vector2D, motionPath: MotionPath) -> None:
        self.initPos = initPos
        self.initVel = initVel
        self.motionPath = motionPath

        self.child: Optional["TrajectorySegment"] = None

    def add_child(self, child: "TrajectorySegment") -> None:
        """Add child segment"""
        self.child = child

    def get_state(self, t) -> State:
        """Get the State at a time t in path"""
        pass

    def get_destination(self) -> State:
        """Get final destination (Position Vector2D)"""
        if self.child == None:
            # bb_integrate is a State [x, y, vx, vy]
            bb_integrate = integrate(self.initPos + self.initVel, self.motionPath.motion_path)

            return State(
                position=Vector2D(bb_integrate[0], bb_integrate[1]),
                velocity=Vector2D(bb_integrate[2], bb_integrate[3]),
            )

        self.child.get_destination()

    def get_total_time(self) -> float:
        """Get total time of path"""
        total_time = 0
        for p in self.motionPath.motion_path:
            total_time += p.duration

        if self.child is not None:
            total_time += self.child.get_total_time()

        return total_time


class Trajectory:
    def __init__(self):
        pass

    def append(self) -> None:
        """Append a TrajectorySegment to the end"""
        pass

    def connect(self, t) -> None:
        """Connect a TrajectorySegment at a time t"""
        pass

    def relocate(self):
        """Append a TrajectorySegment to the beginning"""
        pass

    def get_state(self, t) -> State:
        """Get the State at a time t in path"""
        pass

    def get_position(self, t) -> Vector2D:
        pass

    def get_velocity(self, t) -> Vector2D:
        pass

    def get_acceleration(self, t) -> Vector2D:
        pass

    def get_total_time(selfE) -> float:
        """Get total time of path"""
        pass
