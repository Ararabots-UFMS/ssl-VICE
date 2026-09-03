import pytest

from new_movement.trapezoidal_steering import ControlIntegrator


@pytest.fixture
def integrator():
    return ControlIntegrator()


class TestTwoDimensional:
    def test_single_segment_constant_acceleration(self, integrator):
        state = (0.0, 0.0, 1.0, 0.0)
        control = [([1.0, 0.0], 1.0)]
        x, y, vx, vy = integrator.two_dimensional(state, control)
        assert x == pytest.approx(1.5)  # 0 + 1*1 + 0.5*1*1^2
        assert y == pytest.approx(0.0)
        assert vx == pytest.approx(2.0)  # 1 + 1*1
        assert vy == pytest.approx(0.0)

    def test_zero_control_returns_same_state(self, integrator):
        state = (1.0, 2.0, 3.0, 4.0)
        assert integrator.two_dimensional(state, []) == state

    def test_multi_segment_cumulative(self, integrator):
        state = (0.0, 0.0, 0.0, 0.0)
        control = [([1.0, 0.0], 1.0), ([0.0, 0.0], 1.0), ([-1.0, 0.0], 1.0)]
        x, y, vx, vy = integrator.two_dimensional(state, control)
        # accelerate to v=1 over 1s (x: 0.5), cruise 1s at v=1 (x: +1),
        # decelerate to v=0 over 1s (x: +1 - 0.5 = 0.5)
        assert x == pytest.approx(2.0)
        assert vx == pytest.approx(0.0, abs=1e-9)

    def test_y_axis_independent_of_x(self, integrator):
        state = (0.0, 0.0, 0.0, 0.0)
        control = [([1.0, -1.0], 2.0)]
        x, y, vx, vy = integrator.two_dimensional(state, control)
        assert x == pytest.approx(2.0)  # 0.5 * 1 * 4
        assert y == pytest.approx(-2.0)  # 0.5 * -1 * 4
        assert vx == pytest.approx(2.0)
        assert vy == pytest.approx(-2.0)


class TestTwoDimensionalAtTime:
    def test_time_within_first_segment(self, integrator):
        state = (0.0, 0.0, 0.0, 0.0)
        control = [([1.0, 0.0], 2.0), ([0.0, 0.0], 2.0)]
        x, y, vx, vy = integrator.two_dimensional_at_time(state, control, 1.0)
        assert x == pytest.approx(0.5)
        assert vx == pytest.approx(1.0)

    def test_time_at_full_duration_matches_two_dimensional(self, integrator):
        state = (0.0, 0.0, 0.0, 0.0)
        control = [([1.0, 0.5], 1.0), ([-1.0, -0.5], 1.0)]
        total = integrator.two_dimensional(state, control)
        at_end = integrator.two_dimensional_at_time(state, control, 2.0)
        assert at_end == pytest.approx(total)

    def test_time_beyond_duration_clamps_to_final_state(self, integrator):
        state = (0.0, 0.0, 0.0, 0.0)
        control = [([1.0, 0.0], 1.0)]
        total = integrator.two_dimensional(state, control)
        beyond = integrator.two_dimensional_at_time(state, control, 100.0)
        assert beyond == pytest.approx(total)

    def test_time_zero_returns_initial_state(self, integrator):
        state = (1.0, 2.0, 3.0, 4.0)
        control = [([1.0, 1.0], 5.0)]
        result = integrator.two_dimensional_at_time(state, control, 0.0)
        assert result == pytest.approx(state)

    def test_time_spanning_second_segment(self, integrator):
        state = (0.0, 0.0, 0.0, 0.0)
        control = [([1.0, 0.0], 1.0), ([0.0, 0.0], 1.0)]
        # after first segment: x=0.5, vx=1; then 0.5s of cruise -> x=1.0
        x, y, vx, vy = integrator.two_dimensional_at_time(state, control, 1.5)
        assert x == pytest.approx(1.0)
        assert vx == pytest.approx(1.0)


class TestTwoDimensionalTimestep:
    def test_states_start_and_end_match(self, integrator):
        state = (0.0, 0.0, 0.0, 0.0)
        control = [([1.0, 0.0], 1.0)]
        states = integrator.two_dimensional_timestep(state, control, 0.25)
        assert states[0] == pytest.approx(state)
        expected_final = integrator.two_dimensional(state, control)
        assert states[-1] == pytest.approx(expected_final)

    def test_number_of_samples_matches_timestep(self, integrator):
        state = (0.0, 0.0, 0.0, 0.0)
        control = [([1.0, 0.0], 1.0)]
        states = integrator.two_dimensional_timestep(state, control, 0.25)
        # 1.0 / 0.25 = 4 steps + initial state
        assert len(states) == 5

    def test_uneven_final_step_is_partial(self, integrator):
        state = (0.0, 0.0, 0.0, 0.0)
        control = [([1.0, 0.0], 1.0)]
        states = integrator.two_dimensional_timestep(state, control, 0.3)
        # steps of 0.3, 0.3, 0.3, then a final partial 0.1
        assert len(states) == 5
        expected_final = integrator.two_dimensional(state, control)
        assert states[-1] == pytest.approx(expected_final)

    def test_multi_segment_timestep(self, integrator):
        state = (0.0, 0.0, 0.0, 0.0)
        control = [([1.0, 0.0], 1.0), ([-1.0, 0.0], 1.0)]
        states = integrator.two_dimensional_timestep(state, control, 0.5)
        expected_final = integrator.two_dimensional(state, control)
        assert states[-1] == pytest.approx(expected_final)
        # 2 steps per segment, 2 segments -> 4 steps + initial
        assert len(states) == 5


class TestNDimensional:
    def test_matches_two_dimensional_for_2d(self, integrator):
        state = [0.0, 0.0, 1.0, 0.0]
        control = [([1.0, 0.0], 1.0), ([0.0, -1.0], 1.0)]
        n_result = integrator.n_dimensional(state, control)
        two_d_result = integrator.two_dimensional(tuple(state), control)
        assert list(n_result) == pytest.approx(list(two_d_result))

    def test_one_dimensional(self, integrator):
        state = [0.0, 0.0]
        control = [([1.0], 2.0)]
        x, vx = integrator.n_dimensional(state, control)
        assert x == pytest.approx(2.0)  # 0.5 * 1 * 4
        assert vx == pytest.approx(2.0)

    def test_three_dimensional(self, integrator):
        state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        control = [([1.0, 2.0, -1.0], 1.0)]
        result = integrator.n_dimensional(state, control)
        assert result[0] == pytest.approx(0.5)
        assert result[1] == pytest.approx(1.0)
        assert result[2] == pytest.approx(-0.5)
        assert result[3] == pytest.approx(1.0)
        assert result[4] == pytest.approx(2.0)
        assert result[5] == pytest.approx(-1.0)

    def test_does_not_mutate_input_state(self, integrator):
        state = [0.0, 0.0]
        control = [([1.0], 1.0)]
        integrator.n_dimensional(state, control)
        assert state == [0.0, 0.0]
