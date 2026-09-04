import pytest

from new_movement.trapezoidal_steering import TrapezoidalSolver


def integrate_1d(ix, iv, control):
    x, v = ix, iv
    for accel, duration in control:
        x += v * duration + 0.5 * accel * duration ** 2
        v += accel * duration
    return x, v


def peak_velocities(iv, control):
    """Velocity at the end of each segment (where extrema occur for
    constant-acceleration segments)."""
    v = iv
    velocities = [v]
    for accel, duration in control:
        v += accel * duration
        velocities.append(v)
    return velocities


@pytest.fixture
def solver():
    return TrapezoidalSolver()


class TestOptimalWithinVelocityBounds:
    def test_short_move_stays_bang_bang(self, solver):
        # Distance small enough that pure bang-bang never exceeds vmax.
        control = solver.optimal(0.0, 0.0, 1.0, 0.0, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0)
        x, v = integrate_1d(0.0, 0.0, control)
        assert x == pytest.approx(1.0, abs=1e-6)
        assert v == pytest.approx(0.0, abs=1e-6)
        for velocity in peak_velocities(0.0, control):
            assert -1.0 - 1e-6 <= velocity <= 1.0 + 1e-6


class TestOptimalWithCruisePhase:
    def test_long_move_produces_trapezoid(self, solver):
        control = solver.optimal(0.0, 0.0, 10.0, 0.0, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0)
        assert len(control) == 3
        (a1, t1), (a2, t2), (a3, t3) = control
        assert a1 == pytest.approx(1.0)
        assert a2 == pytest.approx(0.0)
        assert a3 == pytest.approx(-1.0)
        assert t1 == pytest.approx(1.0)
        assert t2 == pytest.approx(9.0)
        assert t3 == pytest.approx(1.0)

    def test_long_move_reaches_target(self, solver):
        control = solver.optimal(0.0, 0.0, 10.0, 0.0, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0)
        x, v = integrate_1d(0.0, 0.0, control)
        assert x == pytest.approx(10.0, abs=1e-6)
        assert v == pytest.approx(0.0, abs=1e-6)

    def test_cruise_velocity_never_exceeds_vmax(self, solver):
        control = solver.optimal(0.0, 0.0, 10.0, 0.0, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0)
        for velocity in peak_velocities(0.0, control):
            assert velocity <= 1.0 + 1e-6

    def test_negative_direction_cruise_respects_vmin(self, solver):
        control = solver.optimal(0.0, 0.0, -10.0, 0.0, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0)
        x, v = integrate_1d(0.0, 0.0, control)
        assert x == pytest.approx(-10.0, abs=1e-6)
        assert v == pytest.approx(0.0, abs=1e-6)
        for velocity in peak_velocities(0.0, control):
            assert velocity >= -1.0 - 1e-6


class TestOptimalEdgeCases:
    def test_already_at_target_returns_empty(self, solver):
        assert solver.optimal(2.0, 0.5, 2.0, 0.5) == []

    def test_zero_distance_but_velocity_change(self, solver):
        control = solver.optimal(1.0, 0.0, 1.0, 0.0, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0)
        assert control == []


class TestHardStop:
    def test_decelerates_from_moving_state(self, solver):
        control = solver.hard_stop(5.0, 1.0, 0.0, 0.0, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0)
        assert control != []
        # first segment must brake (opposite sign of initial velocity)
        assert control[0][0] < 0
        x, v = integrate_1d(5.0, 1.0, control)
        assert x == pytest.approx(0.0, abs=1e-6)
        assert v == pytest.approx(0.0, abs=1e-6)

    def test_decelerates_negative_velocity(self, solver):
        control = solver.hard_stop(-5.0, -1.0, 0.0, 0.0, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0)
        assert control[0][0] > 0
        x, v = integrate_1d(-5.0, -1.0, control)
        assert x == pytest.approx(0.0, abs=1e-6)
        assert v == pytest.approx(0.0, abs=1e-6)

    def test_zero_initial_velocity_matches_optimal(self, solver):
        hard_stop = solver.hard_stop(0.0, 0.0, 3.0, 0.0, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0)
        optimal = solver.optimal(0.0, 0.0, 3.0, 0.0, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0)
        assert hard_stop == optimal


class TestHardStopWait:
    def test_inserts_wait_segment_for_longer_time(self, solver):
        base = solver.hard_stop(0.0, 0.0, 1.0, 0.0, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0)
        base_duration = solver.duration(base)
        final_time = base_duration + 5.0
        control = solver.hard_stop_wait(0.0, 0.0, 1.0, 0.0, final_time, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0)
        assert solver.duration(control) == pytest.approx(final_time, abs=1e-6)
        x, v = integrate_1d(0.0, 0.0, control)
        assert x == pytest.approx(1.0, abs=1e-6)
        assert v == pytest.approx(0.0, abs=1e-6)

    def test_infeasible_when_time_shorter_than_hard_stop(self, solver):
        base = solver.hard_stop(0.0, 2.0, 0.0, 0.0, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0)
        base_duration = solver.duration(base)
        control = solver.hard_stop_wait(0.0, 2.0, 0.0, 0.0, base_duration * 0.1, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0)
        assert control == []

    def test_wait_when_already_stopped_at_start(self, solver):
        # iv == 0, so wait is inserted before the optimal move.
        final_time = 20.0
        control = solver.hard_stop_wait(0.0, 0.0, 1.0, 0.0, final_time, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0)
        assert solver.duration(control) == pytest.approx(final_time, abs=1e-6)
        assert control[0][0] == pytest.approx(0.0)


class TestScaled:
    def test_matches_optimal_when_final_time_equals_optimal_duration(self, solver):
        optimal = solver.optimal(0.0, 0.0, 1.0, 0.0, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0)
        optimal_time = solver.duration(optimal)
        scaled = solver.scaled(0.0, 0.0, 1.0, 0.0, optimal_time, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0)
        assert scaled == optimal

    def test_stretched_duration_reaches_target(self, solver):
        optimal = solver.optimal(0.0, 0.0, 1.0, 0.0, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0)
        optimal_time = solver.duration(optimal)
        final_time = optimal_time * 2.0
        control = solver.scaled(0.0, 0.0, 1.0, 0.0, final_time, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0)
        assert control != []
        assert solver.duration(control) == pytest.approx(final_time, abs=1e-6)
        x, v = integrate_1d(0.0, 0.0, control)
        assert x == pytest.approx(1.0, abs=1e-6)
        assert v == pytest.approx(0.0, abs=1e-6)

    def test_infeasible_when_time_too_short(self, solver):
        optimal = solver.optimal(0.0, 0.0, 10.0, 0.0, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0)
        optimal_time = solver.duration(optimal)
        control = solver.scaled(0.0, 0.0, 10.0, 0.0, optimal_time * 0.1, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0)
        assert control == []

    def test_scaled_respects_velocity_bounds(self, solver):
        final_time = 20.0
        control = solver.scaled(0.0, 0.0, 1.0, 0.0, final_time, umin=-1.0, umax=1.0, vmin=-1.0, vmax=1.0)
        assert control != []
        assert solver.duration(control) == pytest.approx(final_time, abs=1e-6)
        for velocity in peak_velocities(0.0, control):
            assert -1.0 - 1e-6 <= velocity <= 1.0 + 1e-6


class TestStretched:
    """
    When two axes finish at slightly different times the shorter one is stretched to
    match. Neither existing strategy covers a velocity-limited profile short by a
    millisecond: a two-phase bang-bang over the same time has to peak above the velocity
    cap, and braking to a standstill costs more time than the stretch was asking for.
    Both returned [], and ControlMerger drops the whole path if any axis is empty.
    """

    # The exact axis that used to come back empty.
    IX, IV = 1322.3314785179027, -1278.7283938812486
    GX, GV = -4145.591636237768, 0.0
    TARGET = 3.4891578801649876
    UMIN, UMAX = -1500.0, 1500.0
    VMIN, VMAX = -2000.0, 2000.0

    @pytest.fixture
    def stretched(self):
        solver = TrapezoidalSolver()
        return solver, solver.stretched(
            self.IX, self.IV, self.GX, self.GV, self.TARGET,
            self.UMIN, self.UMAX, self.VMIN, self.VMAX,
        )

    def test_the_existing_stretches_cannot_cover_this_case(self):
        """The premise: a cruise phase means a two-phase stretch cannot stay in bounds."""
        solver = TrapezoidalSolver()
        optimal = solver.optimal(
            self.IX, self.IV, self.GX, self.GV, self.UMIN, self.UMAX, self.VMIN, self.VMAX
        )
        assert any(acceleration == 0.0 for acceleration, _ in optimal)
        assert solver.duration(optimal) < self.TARGET

        bang_bang = solver.bang_bang.scaled(
            self.IX, self.IV, self.GX, self.GV, self.TARGET, self.UMIN, self.UMAX
        )
        peak = self.IV + bang_bang[0][0] * bang_bang[0][1]
        assert abs(peak) > self.VMAX, "the two-phase stretch is meant to breach the cap"

        hard_stop = solver.hard_stop(
            self.IX, self.IV, self.GX, self.GV, self.UMIN, self.UMAX, self.VMIN, self.VMAX
        )
        assert solver.duration(hard_stop) > self.TARGET

    def test_it_hits_the_target_duration_and_lands_on_the_goal(self, stretched):
        solver, profile = stretched
        assert profile
        assert solver.duration(profile) == pytest.approx(self.TARGET, abs=1e-6)

        position, velocity = self.IX, self.IV
        for acceleration, duration in profile:
            position += velocity * duration + 0.5 * acceleration * duration ** 2
            velocity += acceleration * duration

        assert position == pytest.approx(self.GX, abs=1e-3)
        assert velocity == pytest.approx(self.GV, abs=1e-3)

    def test_it_stays_inside_both_bounds(self, stretched):
        _, profile = stretched

        velocity = self.IV
        assert abs(velocity) <= self.VMAX + 1e-6
        for acceleration, duration in profile:
            assert self.UMIN <= acceleration <= self.UMAX
            velocity += acceleration * duration
            assert abs(velocity) <= self.VMAX + 1e-6
