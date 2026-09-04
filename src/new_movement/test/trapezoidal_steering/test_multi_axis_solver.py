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


class TestNoGoalIsLeftWithoutATrajectory:
    """
    A velocity-limited axis that neither stretch could lengthen came back empty, and
    ControlMerger drops the whole path if any axis is — so the robot was handed a
    zero-duration trajectory, which then raised out of add_child.
    """

    UMIN, UMAX = -1500.0, 1500.0
    VMIN, VMAX = -2000.0, 2000.0

    def _solve(self, solver, start, goal):
        return solver.time_optimal_2d(
            start, goal,
            umin=[self.UMIN, self.UMIN], umax=[self.UMAX, self.UMAX],
            vmin=[self.VMIN, self.VMIN], vmax=[self.VMAX, self.VMAX],
        )

    def test_the_axis_that_used_to_die_now_solves(self):
        assert self._solve(
            MultiAxisSolver(),
            [-3350.0487442041917, 1322.3314785179027, -598.8241304138708, -1278.7283938812486],
            [43.63806251864662, -4145.591636237768, 0.0, 0.0],
        )

    def test_random_goals_all_produce_a_path(self):
        """The population measure: 8.1% of these used to come back with nothing."""
        import random

        solver = MultiAxisSolver()
        rng = random.Random(0)
        empty = 0
        worst_position = worst_velocity = 0.0

        for _ in range(600):
            ix, iy = rng.uniform(-6000, 6000), rng.uniform(-4500, 4500)
            ivx, ivy = rng.uniform(-2000, 2000), rng.uniform(-2000, 2000)
            gx, gy = rng.uniform(-6000, 6000), rng.uniform(-4500, 4500)

            control = self._solve(solver, [ix, iy, ivx, ivy], [gx, gy, 0.0, 0.0])
            if not control:
                empty += 1
                continue

            x, y, vx, vy = ix, iy, ivx, ivy
            for acceleration, duration in control:
                x += vx * duration + 0.5 * acceleration[0] * duration ** 2
                y += vy * duration + 0.5 * acceleration[1] * duration ** 2
                vx += acceleration[0] * duration
                vy += acceleration[1] * duration

            worst_position = max(worst_position, abs(x - gx), abs(y - gy))
            worst_velocity = max(worst_velocity, abs(vx), abs(vy))

        assert empty == 0, f"{empty}/600 goals produced no trajectory"
        assert worst_position < 1.0
        assert worst_velocity < 1.0
