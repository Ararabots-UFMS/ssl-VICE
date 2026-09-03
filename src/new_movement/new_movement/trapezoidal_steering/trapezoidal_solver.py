"""Velocity-aware one-dimensional trapezoidal planning."""

from math import fabs
from new_movement.trapezoidal_steering import BangBangSolver


class TrapezoidalSolver:
    """Builds bounded profiles from bang-bang primitives."""

    # How far a stretched profile may sit from the duration it was asked for. The axes
    # are merged at their shared boundaries and clipped to the shortest, so anything
    # looser than this quietly trims the end off the longer axis. At 2000 mm/s this is
    # two microns of travel.
    duration_tolerance = 1.0e-6

    def __init__(self, bang_bang=None, time_epsilon=1.0e-7):
        self.bang_bang = bang_bang or BangBangSolver(time_epsilon)
        self.time_epsilon = time_epsilon

    def duration(self, control):
        return self.bang_bang.duration(control)

    def optimal(self, ix, iv, gx, gv, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0):
        bang_bang = self.bang_bang.optimal(ix, iv, gx, gv, umin, umax)
        if not bang_bang:
            return []
        peak = iv + bang_bang[0][0] * bang_bang[0][1]
        if vmin - self.time_epsilon <= peak <= vmax + self.time_epsilon:
            return bang_bang
        cruise = vmax if peak > vmax else vmin
        first_acceleration, third_acceleration = (umax, umin) if peak > vmax else (umin, umax)
        first_time = (cruise - iv) / first_acceleration if first_acceleration else 0
        first_distance = iv * first_time + 0.5 * first_acceleration * first_time ** 2
        third_time = (gv - cruise) / third_acceleration if third_acceleration else 0
        third_distance = cruise * third_time + 0.5 * third_acceleration * third_time ** 2
        second_time = ((gx - ix) - first_distance - third_distance) / cruise if cruise else 0
        if min(first_time, second_time, third_time) < -self.time_epsilon:
            return bang_bang
        result = []
        if first_time > self.time_epsilon:
            result.append([first_acceleration, first_time])
        if second_time > self.time_epsilon:
            result.append([0.0, second_time])
        if third_time > self.time_epsilon:
            result.append([third_acceleration, third_time])
        return result or bang_bang

    def hard_stop(self, ix, iv, gx, gv, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0):
        if fabs(iv) < self.time_epsilon:
            return self.optimal(ix, 0.0, gx, gv, umin, umax, vmin, vmax)
        acceleration = umin if iv > 0 else umax
        stop_time = -iv / acceleration
        stop_position = ix + iv * stop_time + 0.5 * acceleration * stop_time ** 2
        return ([[acceleration, stop_time]] if stop_time > self.time_epsilon else []) + self.optimal(stop_position, 0.0, gx, gv, umin, umax, vmin, vmax)

    def hard_stop_wait(self, ix, iv, gx, gv, final_time, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0):
        control = self.hard_stop(ix, iv, gx, gv, umin, umax, vmin, vmax)
        duration = self.duration(control)
        if duration > final_time + self.time_epsilon:
            return []
        if final_time > duration + self.time_epsilon:
            control.insert(0 if fabs(iv) < self.time_epsilon else 1, [0.0, final_time - duration])
        return control

    def scaled(self, ix, iv, gx, gv, final_time, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0):
        optimal = self.optimal(ix, iv, gx, gv, umin, umax, vmin, vmax)
        duration = self.duration(optimal)
        if duration > final_time + self.time_epsilon:
            return []
        if fabs(duration - final_time) <= self.time_epsilon:
            return optimal
        scaled = self.bang_bang.scaled(ix, iv, gx, gv, final_time, umin, umax)
        if scaled and vmin - self.time_epsilon <= iv + scaled[0][0] * scaled[0][1] <= vmax + self.time_epsilon:
            return scaled
        return self.hard_stop_wait(ix, iv, gx, gv, final_time, umin, umax, vmin, vmax)

    def stretched(self, ix, iv, gx, gv, final_time, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0, iterations=60):
        """
        Make a profile last exactly final_time by using less of the acceleration
        available, keeping its shape.

        Neither of the other two stretches can lengthen a velocity-limited profile by a
        small amount. A two-phase bang-bang that covers the same distance in the same
        time has to peak above the velocity cap, so scaled() throws it out; hard_stop_wait
        brakes to a standstill and re-accelerates from rest, which costs more time than
        was on offer. Both then return nothing, and an axis with no control at all
        collapses the whole merged path to zero duration.

        Duration falls as the acceleration authority rises, so bisecting on that
        authority converges on final_time. The profile is still built by optimal(), so
        the velocity bounds keep being enforced.

        Duration is not continuous in the authority everywhere: optimal() switches
        between a bounded trapezoid and a plain bang-bang, and the duration steps across
        that switch, so for some inputs no authority gives exactly final_time. Returning
        the closest one would be worse than returning nothing, because ControlMerger
        truncates every axis to the shortest and a profile that runs long silently loses
        its tail — the robot would be handed a path that stops short of the goal instead
        of no path at all. So an unconverged search reports failure and leaves the
        caller with the empty result it would have had anyway.
        """
        low, high = 0.0, 1.0
        best = []
        for _ in range(iterations):
            authority = 0.5 * (low + high)
            profile = self.optimal(ix, iv, gx, gv, umin * authority, umax * authority, vmin, vmax)
            if not profile:
                # Too little authority to reach the goal at all; the answer is above this.
                low = authority
                continue
            duration = self.duration(profile)
            if fabs(duration - final_time) <= self.duration_tolerance:
                return profile
            if duration >= final_time:
                # Slow enough, so it is a usable answer and more authority is affordable.
                best = profile
                low = authority
            else:
                high = authority

        if best and fabs(self.duration(best) - final_time) <= self.duration_tolerance:
            return best
        return []
