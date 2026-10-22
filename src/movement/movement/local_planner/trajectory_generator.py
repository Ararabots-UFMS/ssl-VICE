from math import acos, asin, cos, pi, sin
from typing import Optional, Tuple

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

# The limits are split between the axes as the cosine and sine of an angle, so together
# they never exceed them; given to each axis whole, a diagonal ran 41% over. The angle
# is bisected this many times toward the one where both axes take equally long.
SHARE_ITERATIONS = 8
# Neither axis is left with nothing: an idle one still has to brake a small drift.
MIN_SHARE_ANGLE = 0.02
WHOLE_LIMITS = (1.0, 1.0)
# How far (mm, mm/s) a segment may end from its target and still count as reaching it.
ARRIVAL_TOLERANCE = 1.0e-3

class TrajectoryGenerator:
    def __init__(self, constrainsts: Optional[MotionConstraints] = None):
        self.constrainsts = constrainsts or MotionConstraints(DEFAULT_VELOCITY_CONSTRAINST, DEFAULT_ACCELERATION_CONSTRAINST)
        self.steering = MultiAxisSolver()

    def generate(self, curState: MotionState, tarState: MotionState) -> TrajectorySegment:
        """Generates a piecewise constant acceleration motion path using the Trapezoidal Steer"""
        segment = self._steer(curState, tarState, self._axis_shares(curState, tarState))
        if self._arrives(segment, tarState):
            return segment
        # About one state in ten thousand cannot be synchronised under a share (an axis
        # starting past its part of the speed limit). Whole limits per axis always can.
        return self._steer(curState, tarState, WHOLE_LIMITS)

    def _steer(
        self, curState: MotionState, tarState: MotionState, share: Tuple[float, float]
    ) -> TrajectorySegment:
        limits = self.constrainsts
        trap_output = self.steering.time_optimal_2d(
            curState.position + curState.velocity,
            tarState.position + tarState.velocity,
            umin=self._shared(limits.min_acceleration, share),
            umax=self._shared(limits.max_acceleration, share),
            vmin=self._shared(limits.min_velocity, share),
            vmax=self._shared(limits.max_velocity, share),
        )

        # trap_output is a list of piecewise constant acceleration, is other words, ((ax, ay), d) where d is the duration.
        motion_path = MotionPath(
            [MotionPrimitive(Vector2D(out[0][0], out[0][1]), out[1]) for out in trap_output]
        )

        return TrajectorySegment(curState.position, curState.velocity, motion_path)

    @staticmethod
    def _arrives(segment: TrajectorySegment, tarState: MotionState) -> bool:
        """A steer that could not be solved comes back empty, parked on its own start."""
        end = segment.get_local_destination()
        return (
            end.position.distance(tarState.position) < ARRIVAL_TOLERANCE
            and end.velocity.distance(tarState.velocity) < ARRIVAL_TOLERANCE
        )

    @staticmethod
    def _shared(limit: Vector2D, share: Tuple[float, float]) -> Tuple[float, float]:
        return (limit.x * share[0], limit.y * share[1])

    def _axis_shares(self, curState: MotionState, tarState: MotionState) -> Tuple[float, float]:
        """The fraction of the limits each axis may use for this move, as (x, y)."""
        limits = self.constrainsts
        planner = self.steering.planner

        def duration(axis: int, share: float) -> float:
            return planner.duration(
                planner.optimal(
                    curState.position[axis],
                    curState.velocity[axis],
                    tarState.position[axis],
                    tarState.velocity[axis],
                    limits.min_acceleration[axis] * share,
                    limits.max_acceleration[axis] * share,
                    limits.min_velocity[axis] * share,
                    limits.max_velocity[axis] * share,
                )
            )

        low, high = MIN_SHARE_ANGLE, pi / 2 - MIN_SHARE_ANGLE
        # Stay where the velocity asked for at the end fits both shares: an axis cannot
        # finish faster than its part of the limit.
        fits_y = asin(min(1.0, abs(tarState.velocity.y) / limits.max_velocity.y))
        fits_x = acos(min(1.0, abs(tarState.velocity.x) / limits.max_velocity.x))
        if max(low, fits_y) <= min(high, fits_x):
            low, high = max(low, fits_y), min(high, fits_x)
        for _ in range(SHARE_ITERATIONS):
            angle = 0.5 * (low + high)
            # The slower axis is the one that needs more of the limits.
            if duration(0, cos(angle)) > duration(1, sin(angle)):
                high = angle
            else:
                low = angle
        angle = 0.5 * (low + high)
        return cos(angle), sin(angle)

    def update_constrainsts(self, constrainsts: MotionConstraints) -> None:
        self.constrainsts = constrainsts
