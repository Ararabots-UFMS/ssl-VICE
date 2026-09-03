import pytest

from new_movement.trapezoidal_steering import ControlIntegrator, MultiAxisSolver


def total_duration(control):
    return sum(segment[1] for segment in control)


@pytest.fixture
def solver():
    return MultiAxisSolver()


@pytest.fixture
def integrator():
    return ControlIntegrator()


class TestTimeOptimalSingleAxisMoves:
    def test_only_x_axis_moves(self, solver, integrator):
        xinit = [0.0, 0.0, 0.0, 0.0]
        xgoal = [1.0, 0.0, 0.0, 0.0]
        control = solver.time_optimal(xinit, xgoal)
        assert control != []
        result = integrator.n_dimensional(xinit, control)
        assert result[0] == pytest.approx(1.0, abs=1e-6)
        assert result[1] == pytest.approx(0.0, abs=1e-6)
        assert result[2] == pytest.approx(0.0, abs=1e-6)
        assert result[3] == pytest.approx(0.0, abs=1e-6)

    def test_only_y_axis_moves(self, solver, integrator):
        xinit = [0.0, 0.0, 0.0, 0.0]
        xgoal = [0.0, 2.0, 0.0, 0.0]
        control = solver.time_optimal(xinit, xgoal)
        result = integrator.n_dimensional(xinit, control)
        assert result[0] == pytest.approx(0.0, abs=1e-6)
        assert result[1] == pytest.approx(2.0, abs=1e-6)
        assert result[2] == pytest.approx(0.0, abs=1e-6)
        assert result[3] == pytest.approx(0.0, abs=1e-6)


class TestTimeOptimalBothAxesMove:
    def test_both_axes_reach_goal(self, solver, integrator):
        xinit = [0.0, 0.0, 0.0, 0.0]
        xgoal = [3.0, 1.0, 0.0, 0.0]
        control = solver.time_optimal(xinit, xgoal)
        result = integrator.n_dimensional(xinit, control)
        assert result[0] == pytest.approx(3.0, abs=1e-6)
        assert result[1] == pytest.approx(1.0, abs=1e-6)
        assert result[2] == pytest.approx(0.0, abs=1e-6)
        assert result[3] == pytest.approx(0.0, abs=1e-6)

    def test_axes_are_synchronized_to_same_duration(self, solver, integrator):
        # x moves much farther than y, so y's raw bang-bang control would be
        # much shorter; MultiAxisSolver must stretch/wait y to match x.
        xinit = [0.0, 0.0, 0.0, 0.0]
        xgoal = [10.0, 0.5, 0.0, 0.0]
        control = solver.time_optimal(xinit, xgoal)
        # Every merged segment should have the same total duration when summed.
        assert total_duration(control) > 0
        result = integrator.n_dimensional(xinit, control)
        assert result[0] == pytest.approx(10.0, abs=1e-6)
        assert result[1] == pytest.approx(0.5, abs=1e-6)

    def test_result_accelerations_within_bounds(self, solver):
        xinit = [0.0, 0.0, 0.0, 0.0]
        xgoal = [5.0, -3.0, 0.0, 0.0]
        umax = [1.0, 1.0]
        umin = [-1.0, -1.0]
        control = solver.time_optimal(xinit, xgoal, umin=umin, umax=umax)
        for acceleration, duration in control:
            assert duration > 0 or duration == pytest.approx(0.0)
            for a, lo, hi in zip(acceleration, umin, umax):
                assert lo - 1e-6 <= a <= hi + 1e-6


class TestTimeOptimalEdgeCases:
    def test_already_at_goal_returns_empty(self, solver):
        state = [1.0, 2.0, 0.0, 0.0]
        control = solver.time_optimal(state, list(state))
        assert control == []

    def test_nonzero_goal_velocity(self, solver, integrator):
        xinit = [0.0, 0.0, 0.0, 0.0]
        xgoal = [4.0, 2.0, 0.5, 0.5]
        control = solver.time_optimal(xinit, xgoal, vmin=[-1.0, -1.0], vmax=[1.0, 1.0])
        result = integrator.n_dimensional(xinit, control)
        assert result[0] == pytest.approx(4.0, abs=1e-6)
        assert result[1] == pytest.approx(2.0, abs=1e-6)
        assert result[2] == pytest.approx(0.5, abs=1e-6)
        assert result[3] == pytest.approx(0.5, abs=1e-6)


class TestTimeOptimal2D:
    def test_default_bounds_used(self, solver, integrator):
        xinit = (0.0, 0.0, 0.0, 0.0)
        xgoal = (2.0, -1.0, 0.0, 0.0)
        control = solver.time_optimal_2d(xinit, xgoal)
        result = integrator.n_dimensional(list(xinit), control)
        assert result[0] == pytest.approx(2.0, abs=1e-6)
        assert result[1] == pytest.approx(-1.0, abs=1e-6)

    def test_matches_time_optimal_call(self, solver):
        xinit = (0.0, 0.0, 0.0, 0.0)
        xgoal = (2.0, -1.0, 0.0, 0.0)
        via_2d = solver.time_optimal_2d(xinit, xgoal)
        via_generic = solver.time_optimal(xinit, xgoal, (-1, -1), (1, 1), (-1, -1), (1, 1))
        assert via_2d == via_generic
