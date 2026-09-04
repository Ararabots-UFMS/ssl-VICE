"""
Regression tests for TrajectorySampler.

The sampler is a performance rewrite of get_state, so the property that matters is that
it disagrees with get_state nowhere — including on the edges get_state handles specially
(times past the end, times before the start, empty paths).
"""

import numpy as np
import pytest

from new_movement.entities.motion.motion_path import MotionPath
from new_movement.entities.motion.motion_primitive import MotionPrimitive
from new_movement.entities.trajectory.trajectory_sampler import TrajectorySampler
from new_movement.entities.trajectory.trajectory_segment import TrajectorySegment

from utils.math_util import Vector2D


def _segment(init_pos, init_vel, primitives):
    return TrajectorySegment(init_pos, init_vel, MotionPath(list(primitives)))


def _chain(*segments):
    for parent, child in zip(segments, segments[1:]):
        parent.child = child          # bypass add_child: continuity is not under test
    return segments[0]


@pytest.fixture
def accelerating():
    return _segment(
        Vector2D(0.0, 0.0),
        Vector2D(0.0, 0.0),
        [
            MotionPrimitive(Vector2D(1000.0, 500.0), 1.0),
            MotionPrimitive(Vector2D(0.0, 0.0), 0.5),
            MotionPrimitive(Vector2D(-1000.0, -500.0), 1.0),
        ],
    )


class TestAgreesWithGetState:
    def test_matches_across_the_path(self, accelerating):
        times = np.linspace(0.0, accelerating.get_total_duration(), 41)
        sampler = TrajectorySampler(accelerating)
        positions = sampler.positions(times)
        velocities = sampler.velocities(times)

        for i, t in enumerate(times):
            expected = accelerating.get_state(float(t))
            assert positions[i, 0] == pytest.approx(expected.position.x, abs=1e-9)
            assert positions[i, 1] == pytest.approx(expected.position.y, abs=1e-9)
            assert velocities[i, 0] == pytest.approx(expected.velocity.x, abs=1e-9)
            assert velocities[i, 1] == pytest.approx(expected.velocity.y, abs=1e-9)

    def test_matches_across_a_segment_chain(self, accelerating):
        second = _segment(
            Vector2D(-500.0, 250.0),
            Vector2D(300.0, -100.0),
            [MotionPrimitive(Vector2D(200.0, 400.0), 0.75)],
        )
        root = _chain(accelerating, second)

        times = np.linspace(0.0, root.get_total_duration(), 33)
        positions = TrajectorySampler(root).positions(times)

        for i, t in enumerate(times):
            expected = root.get_state(float(t))
            assert positions[i, 0] == pytest.approx(expected.position.x, abs=1e-9)
            assert positions[i, 1] == pytest.approx(expected.position.y, abs=1e-9)

    def test_reports_the_same_duration(self, accelerating):
        assert TrajectorySampler(accelerating).duration == pytest.approx(
            accelerating.get_total_duration()
        )


class TestEdges:
    def test_times_past_the_end_clamp_to_the_destination(self, accelerating):
        duration = accelerating.get_total_duration()
        sampler = TrajectorySampler(accelerating)

        beyond = sampler.positions(np.array([duration * 2.0]))
        end = accelerating.get_state(duration)

        assert beyond[0, 0] == pytest.approx(end.position.x, abs=1e-9)
        assert beyond[0, 1] == pytest.approx(end.position.y, abs=1e-9)

    def test_negative_times_clamp_to_the_start(self, accelerating):
        start = TrajectorySampler(accelerating).positions(np.array([-1.0]))

        assert start[0, 0] == pytest.approx(accelerating.init_pos.x)
        assert start[0, 1] == pytest.approx(accelerating.init_pos.y)

    def test_a_path_with_no_primitives_holds_its_initial_state(self):
        """A start == goal plan has no primitives but still has to answer queries."""
        empty = _segment(Vector2D(120.0, -80.0), Vector2D(0.0, 0.0), [])
        sampler = TrajectorySampler(empty)

        assert sampler.duration == 0.0
        positions = sampler.positions(np.array([0.0, 1.0, 5.0]))
        assert np.allclose(positions, [[120.0, -80.0]] * 3)

    def test_an_empty_time_array_yields_no_samples(self, accelerating):
        assert TrajectorySampler(accelerating).positions(np.array([])).shape == (0, 2)


class TestPositionBounds:
    """
    The box the broad phase rejects against. Too tight and the engine silently stops
    seeing real collisions, which is the worst failure this planner has.
    """

    def _generated(self, start, goal):
        from new_movement.entities.motion.motion_state import MotionState
        from new_movement.local_planner.trajectory_generator import TrajectoryGenerator

        return TrajectorySampler(TrajectoryGenerator().generate(
            MotionState(*start), MotionState(*goal)
        ))

    def test_they_hold_a_densely_sampled_path(self):
        """
        The extremes of a parabola are not always at its endpoints, so a path that
        overshoots and comes back escapes a box built from samples alone.
        """
        import random

        rng = random.Random(17)
        for _ in range(120):
            sampler = self._generated(
                (Vector2D(rng.uniform(-3000, 3000), rng.uniform(-2000, 2000)),
                 Vector2D(rng.uniform(-3000, 3000), rng.uniform(-3000, 3000))),
                (Vector2D(rng.uniform(-3000, 3000), rng.uniform(-2000, 2000)),
                 Vector2D(0.0, 0.0)),
            )
            min_x, min_y, max_x, max_y = sampler.position_bounds()

            dense = sampler.positions(np.linspace(0.0, sampler.duration, 4000))
            assert dense[:, 0].min() >= min_x - 1e-6
            assert dense[:, 0].max() <= max_x + 1e-6
            assert dense[:, 1].min() >= min_y - 1e-6
            assert dense[:, 1].max() <= max_y + 1e-6

    def test_a_turning_path_is_bounded_past_its_endpoints(self):
        """Driving backwards from a forward start: the far point is mid-flight."""
        sampler = self._generated(
            (Vector2D(0.0, 0.0), Vector2D(2000.0, 0.0)),
            (Vector2D(-500.0, 0.0), Vector2D(0.0, 0.0)),
        )
        _, _, max_x, _ = sampler.position_bounds()

        dense = sampler.positions(np.linspace(0.0, sampler.duration, 4000))
        assert dense[:, 0].max() > 100.0  # it really does overshoot
        assert max_x >= dense[:, 0].max() - 1e-6

    def test_they_cover_times_past_the_end(self):
        """``positions`` clamps out-of-range times, so the box must hold those too."""
        sampler = self._generated(
            (Vector2D(-1000.0, 0.0), Vector2D(500.0, 0.0)),
            (Vector2D(1000.0, 500.0), Vector2D(0.0, 0.0)),
        )
        min_x, min_y, max_x, max_y = sampler.position_bounds()

        beyond = sampler.positions(np.array([-5.0, sampler.duration + 5.0]))
        assert np.all(beyond[:, 0] >= min_x - 1e-6)
        assert np.all(beyond[:, 0] <= max_x + 1e-6)
        assert np.all(beyond[:, 1] >= min_y - 1e-6)
        assert np.all(beyond[:, 1] <= max_y + 1e-6)
