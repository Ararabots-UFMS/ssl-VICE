"""
Regression tests for stretching a velocity-limited profile.

When two axes finish at slightly different times, the shorter one is stretched to match.
Two strategies existed and neither covers a velocity-limited profile that is short by a
millisecond: a two-phase bang-bang over the same time has to peak above the velocity cap,
and braking to a standstill costs more time than the stretch was asking for. Both
returned [], and ControlMerger drops the whole path if any axis is empty — so the robot
was handed a zero-duration trajectory on about 8% of goals.
"""

import random

import pytest

from new_movement.trapezoidal_steering import MultiAxisSolver, TrapezoidalSolver

UMIN, UMAX = -1500.0, 1500.0
VMIN, VMAX = -2000.0, 2000.0


def _integrate(ix, iv, control):
    x, v = ix, iv
    for acceleration, duration in control:
        x += v * duration + 0.5 * acceleration * duration ** 2
        v += acceleration * duration
    return x, v


class TestStretchedFillsTheGap:
    """The exact axis that used to come back empty."""

    IX, IV = 1322.3314785179027, -1278.7283938812486
    GX, GV = -4145.591636237768, 0.0
    TARGET = 3.4891578801649876

    @pytest.fixture
    def solver(self):
        return TrapezoidalSolver()

    def test_the_optimal_profile_is_velocity_limited(self, solver):
        """The premise: a cruise phase means a two-phase stretch cannot stay in bounds."""
        profile = solver.optimal(self.IX, self.IV, self.GX, self.GV, UMIN, UMAX, VMIN, VMAX)

        assert any(acceleration == 0.0 for acceleration, _ in profile)
        assert solver.duration(profile) < self.TARGET

    def test_both_existing_stretches_decline(self, solver):
        bang_bang = solver.bang_bang.scaled(
            self.IX, self.IV, self.GX, self.GV, self.TARGET, UMIN, UMAX
        )
        peak = self.IV + bang_bang[0][0] * bang_bang[0][1]
        assert abs(peak) > VMAX, "the two-phase stretch is meant to breach the cap here"

        hard_stop = solver.hard_stop(self.IX, self.IV, self.GX, self.GV, UMIN, UMAX, VMIN, VMAX)
        assert solver.duration(hard_stop) > self.TARGET

    def test_stretched_hits_the_target_duration(self, solver):
        profile = solver.stretched(
            self.IX, self.IV, self.GX, self.GV, self.TARGET, UMIN, UMAX, VMIN, VMAX
        )

        assert profile
        assert solver.duration(profile) == pytest.approx(self.TARGET, abs=1e-6)

    def test_stretched_still_lands_on_the_goal(self, solver):
        profile = solver.stretched(
            self.IX, self.IV, self.GX, self.GV, self.TARGET, UMIN, UMAX, VMIN, VMAX
        )
        position, velocity = _integrate(self.IX, self.IV, profile)

        assert position == pytest.approx(self.GX, abs=1e-3)
        assert velocity == pytest.approx(self.GV, abs=1e-3)

    def test_stretched_respects_the_acceleration_bounds(self, solver):
        profile = solver.stretched(
            self.IX, self.IV, self.GX, self.GV, self.TARGET, UMIN, UMAX, VMIN, VMAX
        )

        assert all(UMIN <= acceleration <= UMAX for acceleration, _ in profile)

    def test_stretched_respects_the_velocity_cap(self, solver):
        profile = solver.stretched(
            self.IX, self.IV, self.GX, self.GV, self.TARGET, UMIN, UMAX, VMIN, VMAX
        )

        velocity = self.IV
        assert abs(velocity) <= VMAX + 1e-6
        for acceleration, duration in profile:
            velocity += acceleration * duration
            assert abs(velocity) <= VMAX + 1e-6

    def test_the_two_dimensional_solve_no_longer_comes_back_empty(self):
        """The failure as the planner meets it: one dead axis emptied the whole path."""
        solver = MultiAxisSolver()
        result = solver.time_optimal_2d(
            [-3350.0487442041917, 1322.3314785179027, -598.8241304138708, -1278.7283938812486],
            [43.63806251864662, -4145.591636237768, 0.0, 0.0],
            umin=[UMIN, UMIN],
            umax=[UMAX, UMAX],
            vmin=[VMIN, VMIN],
            vmax=[VMAX, VMAX],
        )

        assert result


class TestNoGoalIsLeftWithoutATrajectory:
    def test_random_goals_all_produce_a_path(self):
        """
        The population measure. 8.1% of these used to come back with nothing at all,
        which reached the planner as a zero-duration segment and then as an exception
        from add_child, costing the robot its plan for that cycle.
        """
        solver = MultiAxisSolver()
        rng = random.Random(0)
        empty = 0
        worst_position = 0.0
        worst_velocity = 0.0

        for _ in range(600):
            ix, iy = rng.uniform(-6000, 6000), rng.uniform(-4500, 4500)
            ivx, ivy = rng.uniform(-2000, 2000), rng.uniform(-2000, 2000)
            gx, gy = rng.uniform(-6000, 6000), rng.uniform(-4500, 4500)

            control = solver.time_optimal_2d(
                [ix, iy, ivx, ivy],
                [gx, gy, 0.0, 0.0],
                umin=[UMIN, UMIN],
                umax=[UMAX, UMAX],
                vmin=[VMIN, VMIN],
                vmax=[VMAX, VMAX],
            )
            if not control:
                empty += 1
                continue

            x, vx = _integrate(ix, ivx, [(a[0], t) for a, t in control])
            y, vy = _integrate(iy, ivy, [(a[1], t) for a, t in control])
            worst_position = max(worst_position, abs(x - gx), abs(y - gy))
            worst_velocity = max(worst_velocity, abs(vx), abs(vy))

        assert empty == 0, f"{empty}/600 goals produced no trajectory"
        assert worst_position < 1.0
        assert worst_velocity < 1.0
