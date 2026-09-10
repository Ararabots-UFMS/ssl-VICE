from typing import Optional

from movement.trapezoidal_steering import MultiAxisSolver
from movement.entities.motion.motion_state import MotionState
from movement.entities.motion.motion_constraints import MotionConstraints
from movement.entities.motion.motion_path import MotionPath
from movement.entities.motion.motion_primitive import MotionPrimitive
from movement.entities.trajectory.trajectory_segment import TrajectorySegment

from utils.math_util import Vector2D

DEFAULT_VELOCITY_CONSTRAINST = Vector2D(900, 900) # mm/s
DEFAULT_ACCELERATION_CONSTRAINST = Vector2D(450, 450) # mm/s²

NEAR_ACCELERATION_CONSTRAINST = Vector2D(900, 900) # #TODO Hardcoded, needs to get the max_output from control and take a little off

class TrajectoryGenerator:
    def __init__(self, constrainsts: Optional[MotionConstraints] = None):
        self.constrainsts = constrainsts or MotionConstraints(DEFAULT_VELOCITY_CONSTRAINST, DEFAULT_ACCELERATION_CONSTRAINST)
        self.steering = MultiAxisSolver()

    def generate(self, curState: MotionState, tarState: MotionState) -> TrajectorySegment:
        """Generates a piecewise constant acceleration motion path using the Trapezoidal Steer"""
        trap_output = self.steering.time_optimal_2d(
            curState.position + curState.velocity,
            tarState.position + tarState.velocity,
            umin=self.constrainsts.min_acceleration,
            umax=self.constrainsts.max_acceleration,
            vmin=self.constrainsts.min_velocity,
            vmax=self.constrainsts.max_velocity,
        )

        # trap_output is a list of piecewise constant acceleration, is other words, ((ax, ay), d) where d is the duration.
        motion_path = MotionPath(
            [MotionPrimitive(Vector2D(out[0][0], out[0][1]), out[1]) for out in trap_output]
        )

        return TrajectorySegment(curState.position, curState.velocity, motion_path)

    def update_constrainsts(self, constrainsts: MotionConstraints) -> None:
        self.constrainsts = constrainsts
