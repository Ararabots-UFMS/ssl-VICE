import csv

import numpy as np
import pytest

from control.identify_robot import VISION_FRICTION, build_profile, fit, load_csv, profile_command
from control.state_predictor import RobotModel
from vision.kalman_filter import ExtendedKalmanFilterClass2D

TRUE = dict(actuation_delay=0.04, tau=0.08, max_accel=3.0)
# Small grids keep the test fast; the defaults are the same shape, only denser.
GRIDS = dict(delays=np.arange(0.0, 0.1001, 0.01),
             taus=np.arange(0.02, 0.2001, 0.02),
             accels=np.arange(1.0, 6.001, 0.5))


def _synthetic_run(actuation_delay, tau, max_accel, fps=60.0, noise=1.0, seed=0):
    """What identify_robot would log for a robot that is exactly RobotModel."""
    rng = np.random.default_rng(seed)
    truth = RobotModel(actuation_delay, tau, max_accel * 1000.0, history=None)
    profile = [(0.0, 0.5)] + build_profile([0.5, 1.0, 1.5], hold=0.8, rest=1.0)
    commands = []
    t = 0.0
    while (speed := profile_command(profile, t)) is not None:
        commands.append((t, speed * 1000.0, 0.0))
        truth.record_command(t, speed * 1000.0, 0.0)
        t += 0.02
    samples, state, t_prev = [], np.zeros(4), 0.0
    for ts in np.arange(0.01, t, 1.0 / fps):
        state = truth.roll(t_prev, state, ts)
        t_prev = ts
        samples.append((ts, state[0] + rng.normal(0, noise), state[1] + rng.normal(0, noise)))
    return np.array(commands), np.array(samples)


def _through_vision_filter(samples):
    """What game_state carries: the vision EKF's estimate, not the detection."""
    kf = ExtendedKalmanFilterClass2D(friction=VISION_FRICTION)
    kf.initialize(samples[0, 1], samples[0, 2])
    out, t = [], samples[0, 0]
    for ts, x, y in samples:
        if ts > t:
            kf.predict(ts - t)
        t = ts
        kf.update(np.matrix([[x], [y]]))
        out.append((ts, float(kf.x[0, 0]), float(kf.x[1, 0])))
    return np.array(out)


class TestProfile:
    def test_each_speed_goes_out_rests_comes_back_rests(self):
        assert build_profile([0.5], hold=0.8, rest=1.0) == [
            (0.5, 0.8), (0.0, 1.0), (-0.5, 0.8), (0.0, 1.0)]

    def test_the_command_follows_the_profile_then_ends(self):
        profile = build_profile([1.0], hold=0.8, rest=1.0)

        assert profile_command(profile, 0.1) == 1.0
        assert profile_command(profile, 1.0) == 0.0
        assert profile_command(profile, 2.0) == -1.0
        assert profile_command(profile, 3.7) is None


@pytest.fixture(scope="module")
def run():
    return _synthetic_run(**TRUE)


class TestFit:
    def test_it_recovers_the_parameters_of_a_known_robot(self, run):
        commands, samples = run

        result = fit(commands, samples, vision_filter=False, **GRIDS)

        assert result["actuation_delay"] == pytest.approx(TRUE["actuation_delay"], abs=0.004)
        assert result["tau"] == pytest.approx(TRUE["tau"], abs=0.008)
        assert result["max_accel"] == pytest.approx(TRUE["max_accel"], abs=0.2)
        assert result["at_boundary"] == []
        assert result["pulses"] == 6

    def test_a_fit_on_the_edge_of_the_grid_is_flagged(self, run):
        commands, samples = run

        # The true delay, 0.04 s, is below this grid.
        result = fit(commands, samples, delays=np.arange(0.06, 0.1001, 0.02),
                     taus=np.arange(0.04, 0.1201, 0.04), accels=np.arange(2.0, 4.001, 1.0),
                     vision_filter=False)

        assert "actuation_delay" in result["at_boundary"]

    def test_it_sees_through_the_vision_filter(self, run):
        """The EKF lags while the robot accelerates; fitted raw, that lag reads as delay."""
        commands, samples = run
        filtered = _through_vision_filter(samples)

        result = fit(commands, filtered, **GRIDS)
        raw = fit(commands, filtered, vision_filter=False, **GRIDS)

        assert result["actuation_delay"] == pytest.approx(TRUE["actuation_delay"], abs=0.005)
        assert result["tau"] == pytest.approx(TRUE["tau"], abs=0.01)
        assert result["max_accel"] == pytest.approx(TRUE["max_accel"], abs=0.25)
        assert abs(raw["tau"] - TRUE["tau"]) > 0.02

    def test_the_csv_round_trips(self, tmp_path):
        path = tmp_path / "run.csv"
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["kind", "t", "vx", "vy", "x", "y", "theta"])
            w.writerow(["vis", "1.5", "", "", "10.0", "20.0", "0.1"])
            w.writerow(["cmd", "1.0", "500.0", "0.0", "", "", ""])

        commands, samples = load_csv(path)

        assert commands.tolist() == [[1.0, 500.0, 0.0]]
        assert samples.tolist() == [[1.5, 10.0, 20.0]]
