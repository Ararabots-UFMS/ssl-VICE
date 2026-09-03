"""Multi-axis trapezoidal steering."""

from math import fabs

from new_movement.trapezoidal_steering import ControlMerger
from new_movement.trapezoidal_steering import TrapezoidalSolver


class MultiAxisSolver:
    """Plans independent axes and synchronizes them to one duration."""

    def __init__(self, planner=None, merger=None, time_epsilon=1.0e-7):
        self.planner = planner or TrapezoidalSolver(time_epsilon=time_epsilon)
        self.merger = merger or ControlMerger(time_epsilon)
        self.time_epsilon = time_epsilon

    def time_optimal(self, xinit, xgoal, umin=0, umax=0, vmin=0, vmax=0):
        dimension = len(xinit) // 2
        umin = [-1.0] * dimension if umin == 0 else umin
        umax = [1.0] * dimension if umax == 0 else umax
        vmin = [-1.0] * dimension if vmin == 0 else vmin
        vmax = [1.0] * dimension if vmax == 0 else vmax
        controls = [
            self.planner.optimal(xinit[i], xinit[dimension + i], xgoal[i], xgoal[dimension + i], umin[i], umax[i], vmin[i], vmax[i])
            for i in range(dimension)
        ]
        target_time = max((self.merger.duration(control) for control in controls), default=0.0)
        for index, control in enumerate(controls):
            duration = self.merger.duration(control)
            if duration >= target_time - self.time_epsilon:
                continue
            arguments = (
                xinit[index],
                xinit[dimension + index],
                xgoal[index],
                xgoal[dimension + index],
                target_time,
                umin[index],
                umax[index],
                vmin[index],
                vmax[index],
            )
            stretched = self.planner.scaled(*arguments)
            if not stretched:
                stretched = self.planner.hard_stop_wait(*arguments)
            if not stretched:
                # Neither of those can lengthen a velocity-limited profile by a small
                # amount: one has to break the velocity cap to do it, the other brakes to
                # a standstill and cannot make the time back up. Easing off the
                # acceleration instead keeps the profile and costs only the milliseconds
                # asked for. Without this the axis ends up with no control at all, which
                # takes the merged path to zero duration and leaves the robot with
                # nothing to follow on ~8% of goals.
                stretched = self.planner.stretched(*arguments)
            controls[index] = stretched
        return self.merger.scalar_axes(controls)

    def time_optimal_2d(self, xinit, xgoal, umin=(-1, -1), umax=(1, 1), vmin=(-1, -1), vmax=(1, 1)):
        return self.time_optimal(xinit, xgoal, umin, umax, vmin, vmax)
