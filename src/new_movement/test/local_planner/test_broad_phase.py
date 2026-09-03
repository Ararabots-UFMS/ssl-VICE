"""
Tests for the broad-phase obstacle reject in CollisionEngine.

Planning cost is dominated by the swept test, and on a full 11v11 field almost every
obstacle is far from any given candidate path. The engine now compares axis-aligned
boxes first and skips obstacles that cannot possibly be reached.

The whole optimisation rests on one property: an obstacle's box must contain every
point that obstacle considers a collision. If a box is too tight the engine silently
stops seeing real collisions, which is the worst failure this planner has. So the
tests below attack the boxes from that direction — they look for a colliding point
outside the box — and separately pin that the engine's verdicts are unchanged.
"""

import random

import numpy as np
import pytest

from new_movement.entities.motion import MotionConstraints, MotionState
from new_movement.entities.obstacle import (
    EnemyRobotObstacle,
    FieldBorderObstacle,
    GenericCircleObstacle,
    PenaltyAreaObstacle,
)
from new_movement.entities.obstacle.ally_robot_obstacle import AllyRobotObstacle
from new_movement.entities.trajectory.trajectory import Trajectory
from new_movement.entities.trajectory.trajectory_sampler import TrajectorySampler
from new_movement.local_planner.collision_engine import CollisionEngine
from new_movement.local_planner.solver import SolverConfig
from new_movement.local_planner.trajectory_generator import TrajectoryGenerator

from utils.field_util import FieldSide
from utils.math_util import Vector2D


def _geometry():
    return type(
        "G", (), {"field_lines": [], "field_length": 12000.0, "field_width": 9000.0}
    )()


@pytest.fixture
def generator():
    config = SolverConfig()
    return TrajectoryGenerator(
        MotionConstraints(config.max_velocity, config.max_acceleration)
    )


def _reference_is_collision(segment, obstacles, time_step):
    """The engine without its broad phase: every obstacle gets the swept test."""
    sampler = TrajectorySampler(segment)
    duration = sampler.duration
    if duration <= 0:
        return False

    steps = max(1, int(np.ceil(duration / time_step)))
    times = np.linspace(0.0, duration, steps + 1)
    positions = sampler.positions(times)
    return any(
        obs.batch_collides_segments(
            positions[:-1], positions[1:], times[:-1], times[1:]
        )
        for obs in obstacles
    )


class TestBoundsContainCollisions:
    """A box that misses a colliding point would make the engine skip a real hit."""

    def test_enemy_tube_box_holds_every_colliding_point(self):
        rng = random.Random(3)
        for _ in range(200):
            state = MotionState(
                Vector2D(rng.uniform(-3000, 3000), rng.uniform(-2000, 2000)),
                Vector2D(rng.uniform(-3000, 3000), rng.uniform(-3000, 3000)),
            )
            obstacle = EnemyRobotObstacle(state, radius=200)
            min_x, min_y, max_x, max_y = obstacle.bounds()

            for _ in range(40):
                point = Vector2D(rng.uniform(-6000, 6000), rng.uniform(-4500, 4500))
                t = rng.uniform(0.0, 2.0)
                if obstacle.isCollidingAt(point, t):
                    assert min_x <= point.x <= max_x
                    assert min_y <= point.y <= max_y

    def test_circle_box_holds_every_colliding_point(self):
        rng = random.Random(5)
        obstacle = GenericCircleObstacle(Vector2D(300.0, -450.0), 500.0)
        min_x, min_y, max_x, max_y = obstacle.bounds()

        for _ in range(2000):
            point = Vector2D(rng.uniform(-3000, 3000), rng.uniform(-3000, 3000))
            if obstacle.isCollidingAt(point):
                assert min_x <= point.x <= max_x
                assert min_y <= point.y <= max_y

    def test_penalty_area_box_holds_every_colliding_point(self):
        rng = random.Random(7)
        for side in (FieldSide.LEFT, FieldSide.RIGHT):
            obstacle = PenaltyAreaObstacle(_geometry(), side)
            min_x, min_y, max_x, max_y = obstacle.bounds()

            for _ in range(2000):
                point = Vector2D(rng.uniform(-7000, 7000), rng.uniform(-5000, 5000))
                if obstacle.isCollidingAt(point):
                    assert min_x <= point.x <= max_x
                    assert min_y <= point.y <= max_y

    def test_field_border_declines_to_bound_itself(self):
        """
        It occupies everything outside a rectangle, which no finite box contains.
        Returning a box here would be the one way to break the broad phase.
        """
        assert FieldBorderObstacle(_geometry()).bounds() is None

    def test_ally_box_holds_its_whole_trajectory(self, generator):
        rng = random.Random(13)
        for _ in range(60):
            start = MotionState(
                Vector2D(rng.uniform(-3000, 3000), rng.uniform(-2000, 2000)),
                Vector2D(rng.uniform(-2000, 2000), rng.uniform(-2000, 2000)),
            )
            goal = MotionState(
                Vector2D(rng.uniform(-3000, 3000), rng.uniform(-2000, 2000)),
                Vector2D(0.0, 0.0),
            )
            trajectory = Trajectory(generator.generate(start, goal))
            obstacle = AllyRobotObstacle(start, trajectory, radius=190)

            box = obstacle.bounds()
            assert box is not None
            min_x, min_y, max_x, max_y = box

            duration = TrajectorySampler(trajectory.root).duration
            for t in np.linspace(0.0, max(duration, 1e-9), 200):
                point = trajectory.get_position(float(t))
                assert min_x <= point.x - obstacle.radius + 1e-6
                assert max_x >= point.x + obstacle.radius - 1e-6
                assert min_y <= point.y - obstacle.radius + 1e-6
                assert max_y >= point.y + obstacle.radius - 1e-6

    def test_ally_without_a_real_trajectory_declines_to_bound_itself(self):
        """Anything that only offers get_position keeps being tested the long way."""

        class DuckTyped:
            def get_position(self, t):
                return Vector2D(0.0, 0.0)

        state = MotionState(Vector2D(0.0, 0.0), Vector2D(0.0, 0.0))
        assert AllyRobotObstacle(state, DuckTyped()).bounds() is None


class TestTrajectoryBounds:
    def test_bounds_hold_a_densely_sampled_path(self, generator):
        """
        The extremes of a parabola are not always at its endpoints. A path that
        overshoots and comes back leaves the box built from samples alone unless the
        turning point is accounted for.
        """
        rng = random.Random(17)
        for _ in range(120):
            start = MotionState(
                Vector2D(rng.uniform(-3000, 3000), rng.uniform(-2000, 2000)),
                Vector2D(rng.uniform(-3000, 3000), rng.uniform(-3000, 3000)),
            )
            goal = MotionState(
                Vector2D(rng.uniform(-3000, 3000), rng.uniform(-2000, 2000)),
                Vector2D(0.0, 0.0),
            )
            sampler = TrajectorySampler(generator.generate(start, goal))
            min_x, min_y, max_x, max_y = sampler.position_bounds()

            dense = sampler.positions(np.linspace(0.0, sampler.duration, 4000))
            assert dense[:, 0].min() >= min_x - 1e-6
            assert dense[:, 0].max() <= max_x + 1e-6
            assert dense[:, 1].min() >= min_y - 1e-6
            assert dense[:, 1].max() <= max_y + 1e-6

    def test_bounds_cover_times_past_the_end(self, generator):
        """``positions`` clamps out-of-range times, so the box must hold those too."""
        start = MotionState(Vector2D(-1000.0, 0.0), Vector2D(500.0, 0.0))
        goal = MotionState(Vector2D(1000.0, 500.0), Vector2D(0.0, 0.0))
        sampler = TrajectorySampler(generator.generate(start, goal))
        min_x, min_y, max_x, max_y = sampler.position_bounds()

        beyond = sampler.positions(np.array([-5.0, sampler.duration + 5.0]))
        assert np.all(beyond[:, 0] >= min_x - 1e-6)
        assert np.all(beyond[:, 0] <= max_x + 1e-6)
        assert np.all(beyond[:, 1] >= min_y - 1e-6)
        assert np.all(beyond[:, 1] <= max_y + 1e-6)

    def test_a_turning_path_is_bounded_past_its_endpoints(self, generator):
        """Driving backwards from a forward start: the far point is mid-flight."""
        start = MotionState(Vector2D(0.0, 0.0), Vector2D(2000.0, 0.0))
        goal = MotionState(Vector2D(-500.0, 0.0), Vector2D(0.0, 0.0))
        sampler = TrajectorySampler(generator.generate(start, goal))
        _, _, max_x, _ = sampler.position_bounds()

        dense = sampler.positions(np.linspace(0.0, sampler.duration, 4000))
        assert dense[:, 0].max() > 100.0  # it really does overshoot
        assert max_x >= dense[:, 0].max() - 1e-6


class TestVerdictsAreUnchanged:
    def test_broad_phase_agrees_with_testing_every_obstacle(self, generator):
        """
        The reject is an optimisation, not a policy: on a crowded field its verdicts
        have to match testing every obstacle, hit for hit and miss for miss.
        """
        rng = random.Random(23)
        step = SolverConfig().collision_time_step
        compared = 0

        for _ in range(400):
            obstacles = [
                FieldBorderObstacle(_geometry()),
                PenaltyAreaObstacle(_geometry(), FieldSide.RIGHT),
                PenaltyAreaObstacle(_geometry(), FieldSide.LEFT),
                GenericCircleObstacle(
                    Vector2D(rng.uniform(-2000, 2000), rng.uniform(-1500, 1500)), 60
                ),
            ]
            for _ in range(rng.randint(0, 21)):
                obstacles.append(
                    EnemyRobotObstacle(
                        MotionState(
                            Vector2D(rng.uniform(-4500, 4500), rng.uniform(-3000, 3000)),
                            Vector2D(rng.uniform(-2000, 2000), rng.uniform(-2000, 2000)),
                        ),
                        radius=200,
                    )
                )

            start = MotionState(
                Vector2D(rng.uniform(-4300, 4300), rng.uniform(-2800, 2800)),
                Vector2D(rng.uniform(-1500, 1500), rng.uniform(-1500, 1500)),
            )
            goal = MotionState(
                Vector2D(rng.uniform(-4300, 4300), rng.uniform(-2800, 2800)),
                Vector2D(0.0, 0.0),
            )
            segment = generator.generate(start, goal)

            assert CollisionEngine.is_collision(
                segment, obstacles, step
            ) == _reference_is_collision(segment, obstacles, step)
            compared += 1

        assert compared == 400

    def test_a_distant_obstacle_is_never_asked(self, generator):
        """The point of the exercise: far obstacles cost a box comparison, not a sweep."""

        class Counting(GenericCircleObstacle):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.sweeps = 0

            def batch_collides_segments(self, *args, **kwargs):
                self.sweeps += 1
                return super().batch_collides_segments(*args, **kwargs)

        far = Counting(Vector2D(5000.0, 4000.0), 100.0)
        near = Counting(Vector2D(500.0, 0.0), 100.0)

        start = MotionState(Vector2D(0.0, 0.0), Vector2D(0.0, 0.0))
        goal = MotionState(Vector2D(1000.0, 0.0), Vector2D(0.0, 0.0))
        segment = generator.generate(start, goal)

        assert CollisionEngine.is_collision(segment, [far, near])
        assert far.sweeps == 0
        assert near.sweeps == 1
