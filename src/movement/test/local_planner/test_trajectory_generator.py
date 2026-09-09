import pytest

import random

import numpy as np

from movement.local_planner import TrajectoryGenerator
from movement.local_planner.solver import SolverConfig
from movement.entities.motion import MotionState, MotionConstraints
from movement.entities.trajectory.trajectory_segment import TrajectorySegment

from utils.math_util import Vector2D


class TestTrajectoryGenerator:
    def test_generate_returns_trajectory_segment(self, generator):
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(1000, 0), Vector2D(0, 0))

        segment = generator.generate(start, goal)

        assert isinstance(segment, TrajectorySegment)
        assert segment.init_pos == start.position
        assert segment.init_vel == start.velocity

    def test_generate_reaches_goal_position(self, generator):
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(1500, -500), Vector2D(0, 0))

        segment = generator.generate(start, goal)
        destination = segment.get_destination()

        assert destination.position.x == pytest.approx(1500, abs=1e-3)
        assert destination.position.y == pytest.approx(-500, abs=1e-3)

    def test_generate_zero_distance_produces_zero_duration(self, generator):
        state = MotionState(Vector2D(100, 100), Vector2D(0, 0))

        segment = generator.generate(state, state)

        assert segment.get_total_duration() == pytest.approx(0.0, abs=1e-3)

    def test_default_constraints_used_when_none_given(self):
        generator = TrajectoryGenerator()
        assert generator.constrainsts is not None
        assert generator.constrainsts.max_velocity == Vector2D(900, 900)
        assert generator.constrainsts.max_acceleration == Vector2D(450, 450)

    def test_custom_constraints_are_stored(self):
        constraints = MotionConstraints(Vector2D(500, 500), Vector2D(200, 200))
        generator = TrajectoryGenerator(constraints)
        assert generator.constrainsts is constraints

    def test_update_constrainsts_replaces_constraints(self, generator):
        new_constraints = MotionConstraints(Vector2D(100, 100), Vector2D(50, 50))
        generator.update_constrainsts(new_constraints)
        assert generator.constrainsts is new_constraints


class TestGeneratedTrajectoriesHoldTheirConstraints:
    """
    Guards the constraint signs at the level that matters: with a bad bound the steer
    silently returns trajectories that stop short of their goal rather than raising.
    """

    FIELD_HALF_LENGTH = 6000.0
    FIELD_HALF_WIDTH = 4500.0

    @pytest.fixture
    def planner_generator(self):
        config = SolverConfig()
        return TrajectoryGenerator(
            MotionConstraints(config.max_velocity, config.max_acceleration)
        ), config

    def _random_state(self, rng, max_velocity):
        return MotionState(
            Vector2D(
                rng.uniform(-self.FIELD_HALF_LENGTH, self.FIELD_HALF_LENGTH),
                rng.uniform(-self.FIELD_HALF_WIDTH, self.FIELD_HALF_WIDTH),
            ),
            Vector2D(
                rng.uniform(-max_velocity.x, max_velocity.x),
                rng.uniform(-max_velocity.y, max_velocity.y),
            ),
        )

    def test_they_reach_the_goal(self, planner_generator):
        generator, config = planner_generator
        rng = random.Random(0)

        for _ in range(500):
            start = self._random_state(rng, config.max_velocity)
            goal = MotionState(
                self._random_state(rng, config.max_velocity).position, Vector2D(0.0, 0.0)
            )

            destination = generator.generate(start, goal).get_local_destination()

            assert destination.position.distance(goal.position) < 1.0, (
                f"start={start} goal={goal} reached={destination}"
            )
            assert destination.velocity.distance(goal.velocity) < 1.0, (
                f"start={start} goal={goal} reached={destination}"
            )

    def test_the_velocity_limit_is_enforced(self, planner_generator):
        generator, config = planner_generator
        start = MotionState(Vector2D(0.0, 0.0), Vector2D(0.0, 0.0))
        goal = MotionState(Vector2D(4000.0, 4000.0), Vector2D(0.0, 0.0))

        segment = generator.generate(start, goal)

        for t in np.arange(0.0, segment.get_total_duration(), 0.01):
            velocity = segment.get_state(float(t)).velocity
            assert abs(velocity.x) <= config.max_velocity.x + 1.0
            assert abs(velocity.y) <= config.max_velocity.y + 1.0
