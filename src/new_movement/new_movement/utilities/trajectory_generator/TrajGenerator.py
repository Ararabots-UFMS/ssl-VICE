from utils.BB_steer import time_optimal_steer_2d as bb_steer
from entities.States import State, Vector2D, MoveConstraints
from entities.Motion import MotionPath, MotionPrimitive
from entities.Trajectory import TrajectorySegment
from typing import Optional
import numpy as np


DEFAULT_VELOCITY_CONSTRAINST = Vector2D(3000, 3000)
DEFAULT_ACCELERATION_CONSTRAINST = Vector2D(1000, 1000)


class TrajectoryGenerator:
    def __init__(self, constrainsts: Optional[MoveConstraints] = None):
        # If constrainsts are not specified, set to default constrainst
        if constrainsts is not None:
            self.constrainsts = constrainsts
        else:
            self.constrainsts = MoveConstraints(
                DEFAULT_VELOCITY_CONSTRAINST, DEFAULT_ACCELERATION_CONSTRAINST
            )

    def generate(self, curState: State, tarState: State) -> TrajectorySegment:
        """Generates a piecewise constant acceleration motion path using the BB_Steer"""

        bb_output = bb_steer(
            curState.position + curState.velocity,
            tarState.position + tarState.velocity,
            umin=self.constrainsts.min_acceleration,
            umax=self.constrainsts.max_acceleration,
        )

        # bb_output is a list of piecewise constant acceleration, is other words, ((ax, ay), d) where d is the duration.
        motion_path = MotionPath(
            [MotionPrimitive(Vector2D(out[0][0], out[0][1]), out[1]) for out in bb_output]
        )

        return TrajectorySegment(curPos, curVel, motion_path)
