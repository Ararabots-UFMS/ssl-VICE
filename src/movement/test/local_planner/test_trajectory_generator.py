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


class TestLimitsApplyToTheWholeMotion:
    """
    Each axis was handed the full limits, so a diagonal ran at 3536mm/s and 4243mm/s²
    under limits of 2500 and 3000 - 41% more than a robot was ever asked to give along
    an axis.
    """

    @pytest.fixture
    def planner_generator(self):
        config = SolverConfig()
        return TrajectoryGenerator(
            MotionConstraints(config.max_velocity, config.max_acceleration)
        ), config

    @staticmethod
    def _peaks(segment):
        speeds = [
            segment.get_state(float(t)).velocity.size()
            for t in np.linspace(0.0, segment.get_total_duration(), 400)
        ]
        accelerations = [
            p.acceleration.size() for p in segment.motion_path.motion_path if p.duration > 1e-9
        ]
        return max(speeds), max(accelerations)

    @pytest.mark.parametrize("goal", [(4000.0, 4000.0), (4000.0, 1000.0), (-500.0, 3000.0)])
    def test_a_move_from_rest_stays_within_both_limits_in_any_direction(self, planner_generator, goal):
        generator, config = planner_generator
        start = MotionState(Vector2D(0.0, 0.0), Vector2D(0.0, 0.0))

        segment = generator.generate(start, MotionState(Vector2D(*goal), Vector2D(0.0, 0.0)))

        speed, acceleration = self._peaks(segment)
        assert speed <= config.max_velocity.x + 1.0
        assert acceleration <= config.max_acceleration.x + 1.0

    def test_a_move_along_an_axis_keeps_practically_all_of_its_speed(self, planner_generator):
        generator, config = planner_generator
        start = MotionState(Vector2D(0.0, 0.0), Vector2D(0.0, 0.0))

        segment = generator.generate(start, MotionState(Vector2D(4000.0, 0.0), Vector2D(0.0, 0.0)))

        speed, _ = self._peaks(segment)
        assert speed == pytest.approx(config.max_velocity.x, rel=1e-3)

    def test_a_diagonal_runs_as_fast_as_an_axis_move_of_the_same_length(self, planner_generator):
        generator, _ = planner_generator
        start = MotionState(Vector2D(0.0, 0.0), Vector2D(0.0, 0.0))
        side = 4000.0 / 2 ** 0.5

        straight = generator.generate(start, MotionState(Vector2D(4000.0, 0.0), Vector2D(0.0, 0.0)))
        diagonal = generator.generate(start, MotionState(Vector2D(side, side), Vector2D(0.0, 0.0)))

        assert diagonal.get_total_duration() == pytest.approx(
            straight.get_total_duration(), rel=5e-3
        )

    def test_the_acceleration_limit_holds_from_any_start(self, planner_generator):
        generator, config = planner_generator
        rng = random.Random(3)

        for _ in range(300):
            angle = rng.uniform(0.0, 6.283)
            speed = rng.uniform(0.0, config.max_velocity.x)
            start = MotionState(
                Vector2D(rng.uniform(-3000, 3000), rng.uniform(-2000, 2000)),
                Vector2D(speed * np.cos(angle), speed * np.sin(angle)),
            )
            goal = MotionState(
                Vector2D(rng.uniform(-3000, 3000), rng.uniform(-2000, 2000)), Vector2D(0.0, 0.0)
            )

            segment = generator.generate(start, goal)

            assert segment.get_local_destination().position.distance(goal.position) < 1.0
            _, acceleration = self._peaks(segment)
            assert acceleration <= config.max_acceleration.x + 1.0


class TestSharedLimitsNeverCostAPath:
    """
    About one state in ten thousand could not be synchronised under a share and came back
    empty. The bypass search minimises duration, so it singled those out as the best route.
    """

    @pytest.fixture
    def planner_generator(self):
        config = SolverConfig()
        return TrajectoryGenerator(MotionConstraints(config.max_velocity, config.max_acceleration))

    def test_the_state_that_first_failed(self, planner_generator):
        start = MotionState(
            Vector2D(-2241.1605495197346, -1930.117904329045),
            Vector2D(-1763.573922314115, -1589.8579411626222),
        )
        goal = MotionState(Vector2D(-3745.396999114707, -1471.2009374622776), Vector2D(0.0, 0.0))

        end = planner_generator.generate(start, goal).get_local_destination()

        assert end.position.distance(goal.position) < 1e-3
        assert end.velocity.size() < 1e-3

    def test_every_move_to_rest_ends_within_the_planners_tolerance(self, planner_generator):
        rng = random.Random(4)

        for _ in range(4000):
            angle = rng.uniform(0.0, 6.283)
            speed = rng.uniform(0.0, 2500.0)
            start = MotionState(
                Vector2D(rng.uniform(-4000, 4000), rng.uniform(-2500, 2500)),
                Vector2D(speed * np.cos(angle), speed * np.sin(angle)),
            )
            goal = MotionState(
                Vector2D(rng.uniform(-4000, 4000), rng.uniform(-2500, 2500)), Vector2D(0.0, 0.0)
            )

            end = planner_generator.generate(start, goal).get_local_destination()

            assert end.position.distance(goal.position) < 1e-3, f"start={start} goal={goal}"
            assert end.velocity.size() < 1e-3, f"start={start} goal={goal}"
