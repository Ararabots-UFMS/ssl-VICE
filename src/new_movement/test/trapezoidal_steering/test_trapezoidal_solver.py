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
