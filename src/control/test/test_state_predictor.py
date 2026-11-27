import math

import numpy as np
import pytest

from control.state_predictor import RobotModel, RobotStatePredictor

DELAY, TAU, A_MAX = 0.04, 0.08, 3000.0


def _at_rest(**kw):
    p = RobotStatePredictor(DELAY, TAU, kw.pop("max_accel", A_MAX), **kw)
    p.correct(10.0, 0.0, 0.0)
    return p


class TestCommandModel:
    def test_nothing_happens_before_the_actuation_delay(self):
        p = _at_rest()
        p.record_command(10.0, 1000.0, 0.0)

        x, y, vx, vy = p.state_at(10.0 + DELAY)

        assert (x, vx) == (0.0, 0.0)
        assert p.state_at(10.0 + DELAY + 0.01)[2] > 0.0

    def test_velocity_reaches_63_percent_of_a_step_after_tau(self):
        # Small step, so the acceleration clamp stays out of it.
        p = _at_rest(max_accel=1e9)
        p.record_command(10.0, 100.0, 0.0)

        vx = p.state_at(10.0 + DELAY + TAU)[2]

        assert vx == pytest.approx(100.0 * (1 - math.exp(-1)), rel=0.02)

    def test_acceleration_is_clamped_on_the_vector_norm(self):
        p = _at_rest()
        p.record_command(10.0, 3000.0, 3000.0)

        _, _, vx, vy = p.state_at(10.0 + DELAY + 0.1)

        assert math.hypot(vx, vy) == pytest.approx(A_MAX * 0.1, rel=1e-6)
        assert vx == pytest.approx(vy)

    def test_a_command_holds_until_the_next_one(self):
        p = _at_rest(max_accel=1e9)
        p.record_command(10.0, 500.0, 0.0)
        p.record_command(10.5, 0.0, 0.0)

        # 0.45 s is 5.6 tau: within 0.5% of the command.
        assert p.state_at(10.0 + DELAY + 0.45)[2] == pytest.approx(500.0, rel=5e-3)
        assert p.state_at(10.5 + DELAY + 1.0)[2] == pytest.approx(0.0, abs=1.0)

    def test_position_integrates_velocity(self):
        p = _at_rest(max_accel=1e9)
        p.record_command(10.0, 1000.0, 0.0)

        # Long after the transient: lagging the command by tau, distance-wise.
        x = p.state_at(10.0 + DELAY + 1.0)[0]

        # Within the 2 ms integration step's error.
        assert x == pytest.approx(1000.0 * (1.0 - TAU), rel=2e-3)

    def test_the_model_rolls_many_parameter_sets_at_once(self):
        taus = np.array([[0.05], [0.1]])
        model = RobotModel(0.0, taus, np.array([[1e9], [1e9]]))
        model.record_command(0.0, 100.0, 0.0)

        state = model.roll(0.0, np.zeros((2, 4)), 0.05)

        single = RobotModel(0.0, 0.1, 1e9)
        single.record_command(0.0, 100.0, 0.0)
        assert state[1] == pytest.approx(single.roll(0.0, np.zeros(4), 0.05))
        assert state[0, 2] > state[1, 2]


class TestCorrection:
    def test_a_measurement_pulls_the_estimate_toward_it(self):
        p = _at_rest()
        p.correct(10.1, 100.0, -50.0)

        x, y, _, _ = p.state_at(10.1)

        assert 0.0 < x <= 100.0
        assert -50.0 <= y < 0.0

    def test_a_robot_slower_than_commanded_is_tracked_by_its_measurements(self):
        """Slip, a weak battery: the model says one thing, vision another."""
        p = _at_rest(max_accel=1e9)
        model_only = _at_rest(max_accel=1e9)
        for k in range(100):
            p.record_command(10.0 + 0.02 * k, 1000.0, 0.0)
            model_only.record_command(10.0 + 0.02 * k, 1000.0, 0.0)
        truth = RobotModel(DELAY, TAU, 1e9)
        truth.record_command(10.0, 700.0, 0.0)
        for k in range(1, 61):
            t = 10.0 + k / 60
            p.correct(t, truth.roll(10.0, np.zeros(4), t)[0], 0.0)

        t_end = 11.0
        true_x = truth.roll(10.0, np.zeros(4), t_end)[0]
        x = p.state_at(t_end)[0]

        assert abs(x - true_x) < 0.2 * abs(model_only.state_at(t_end)[0] - true_x)

    def test_a_repeated_stamp_is_ignored(self):
        p = _at_rest()
        p.correct(10.1, 100.0, 0.0)
        before = p.state_at(10.1)

        p.correct(10.1, 500.0, 0.0)

        assert p.state_at(10.1) == before

    def test_an_older_stamp_is_ignored(self):
        p = _at_rest()
        p.correct(10.1, 100.0, 0.0)
        before = p.state_at(10.1)

        p.correct(10.05, 500.0, 0.0)

        assert p.state_at(10.1) == before
        assert p.last_capture == 10.1

    def test_no_state_before_the_first_measurement(self):
        p = RobotStatePredictor(DELAY, TAU, A_MAX)
        p.record_command(10.0, 1000.0, 0.0)

        assert p.state_at(10.5) is None
        assert p.last_capture is None

    def test_the_first_measurement_starts_at_rest_there(self):
        p = RobotStatePredictor(DELAY, TAU, A_MAX)
        p.correct(10.0, 12.0, 34.0)

        assert p.state_at(10.0) == (12.0, 34.0, 0.0, 0.0)


class TestHistory:
    def test_old_commands_are_dropped_but_the_one_in_effect_stays(self):
        p = _at_rest(max_accel=1e9)
        p.record_command(10.0, 500.0, 0.0)
        p.correct(13.0, 1500.0, 0.0)
        p.record_command(13.0, 500.0, 0.0)

        assert len(p.model._commands) == 2
        assert p.state_at(13.0 + DELAY)[2] > 0.0
