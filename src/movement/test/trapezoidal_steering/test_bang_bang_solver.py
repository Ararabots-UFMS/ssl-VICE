import math

import pytest

from movement.trapezoidal_steering import BangBangSolver


def integrate_1d(ix, iv, control):
    """Integrate a 1D bang-bang control sequence [[accel, duration], ...]."""
    x, v = ix, iv
    for accel, duration in control:
        x += v * duration + 0.5 * accel * duration ** 2
        v += accel * duration
    return x, v


def duration_of(control):
    return sum(segment[1] for segment in control)


@pytest.fixture
def solver():
    return BangBangSolver()


class TestAlreadyAtTarget:
    def test_zero_distance_zero_velocity_returns_empty(self, solver):
        assert solver.optimal(0.0, 0.0, 0.0, 0.0) == []

    def test_same_state_nonzero_returns_empty(self, solver):
        assert solver.optimal(3.5, 1.0, 3.5, 1.0) == []


class TestOptimalSymmetric:
    def test_rest_to_rest_reaches_target(self, solver):
        control = solver.optimal(0.0, 0.0, 1.0, 0.0, umin=-1.0, umax=1.0)
        assert control != []
        x, v = integrate_1d(0.0, 0.0, control)
        assert x == pytest.approx(1.0, abs=1e-6)
        assert v == pytest.approx(0.0, abs=1e-6)

    def test_rest_to_rest_symmetric_durations(self, solver):
        # Symmetric double-integrator: accelerate then decelerate with equal
        # magnitude bounds should produce equal-duration segments.
        control = solver.optimal(0.0, 0.0, 1.0, 0.0, umin=-1.0, umax=1.0)
        assert len(control) == 2
        (a1, t1), (a2, t2) = control
        assert a1 == pytest.approx(1.0)
        assert a2 == pytest.approx(-1.0)
        assert t1 == pytest.approx(t2, rel=1e-6)
        assert t1 == pytest.approx(1.0, rel=1e-6)

    def test_negative_direction(self, solver):
        control = solver.optimal(0.0, 0.0, -1.0, 0.0, umin=-1.0, umax=1.0)
        x, v = integrate_1d(0.0, 0.0, control)
        assert x == pytest.approx(-1.0, abs=1e-6)
        assert v == pytest.approx(0.0, abs=1e-6)
        # First segment should decelerate/accelerate towards negative x.
        assert control[0][0] == pytest.approx(-1.0)

    def test_all_segment_accelerations_within_bounds(self, solver):
        control = solver.optimal(0.0, 0.0, 5.0, 0.0, umin=-2.0, umax=1.5)
        for accel, duration in control:
            assert -2.0 - 1e-9 <= accel <= 1.5 + 1e-9
            assert duration > 0


class TestOptimalWithInitialVelocity:
    def test_moving_towards_target_reaches_goal(self, solver):
        control = solver.optimal(0.0, 0.5, 10.0, 0.0, umin=-1.0, umax=1.0)
        x, v = integrate_1d(0.0, 0.5, control)
        assert x == pytest.approx(10.0, abs=1e-6)
        assert v == pytest.approx(0.0, abs=1e-6)

    def test_moving_away_from_target_still_reaches_goal(self, solver):
        # Moving in the wrong direction initially: solver should still find
        # a bang-bang plan that reaches the goal exactly.
        control = solver.optimal(0.0, -1.0, 5.0, 0.0, umin=-1.0, umax=1.0)
        x, v = integrate_1d(0.0, -1.0, control)
        assert x == pytest.approx(5.0, abs=1e-6)
        assert v == pytest.approx(0.0, abs=1e-6)

    def test_reach_nonzero_goal_velocity(self, solver):
        control = solver.optimal(0.0, 0.0, 4.0, 1.0, umin=-1.0, umax=1.0)
        x, v = integrate_1d(0.0, 0.0, control)
        assert x == pytest.approx(4.0, abs=1e-6)
        assert v == pytest.approx(1.0, abs=1e-6)


class TestOptimalAsymmetricBounds:
    def test_asymmetric_accel_limits_still_reach_target(self, solver):
        control = solver.optimal(0.0, 0.0, 3.0, 0.0, umin=-0.5, umax=2.0)
        x, v = integrate_1d(0.0, 0.0, control)
        assert x == pytest.approx(3.0, abs=1e-6)
        assert v == pytest.approx(0.0, abs=1e-6)
        for accel, _ in control:
            assert -0.5 - 1e-9 <= accel <= 2.0 + 1e-9


class TestDuration:
    def test_duration_matches_sum_of_segments(self, solver):
        control = [[1.0, 2.0], [-1.0, 3.0]]
        assert solver.duration(control) == pytest.approx(5.0)

    def test_duration_empty_control(self, solver):
        assert solver.duration([]) == 0.0


class TestScaled:
    def test_scaled_matches_requested_duration(self, solver):
        optimal_control = solver.optimal(0.0, 0.0, 1.0, 0.0)
        optimal_time = duration_of(optimal_control)
        final_time = optimal_time * 2.0
        control = solver.scaled(0.0, 0.0, 1.0, 0.0, final_time)
        assert control != []
        assert duration_of(control) == pytest.approx(final_time, abs=1e-6)

    def test_scaled_reaches_target_state(self, solver):
        final_time = 5.0
        control = solver.scaled(0.0, 0.0, 2.0, 0.5, final_time, umin=-1.0, umax=1.0)
        assert control != []
        x, v = integrate_1d(0.0, 0.0, control)
        assert x == pytest.approx(2.0, abs=1e-6)
        assert v == pytest.approx(0.5, abs=1e-6)

    def test_scaled_infeasible_when_time_too_short(self, solver):
        # The optimal (fastest) time for this move is > 0; requesting a much
        # shorter final_time than optimal must be infeasible.
        optimal_control = solver.optimal(0.0, 0.0, 10.0, 0.0)
        optimal_time = duration_of(optimal_control)
        control = solver.scaled(0.0, 0.0, 10.0, 0.0, optimal_time * 0.1)
        assert control == []

    def test_scaled_accelerations_within_bounds(self, solver):
        final_time = 6.0
        control = solver.scaled(0.0, 0.0, 1.0, 0.0, final_time, umin=-1.0, umax=1.0)
        assert control != []
        for accel, _ in control:
            assert -1.0 - 1e-9 <= accel <= 1.0 + 1e-9
