import pytest

from new_movement.local_planner import TrajectoryGenerator
from new_movement.entities.motion import MotionState, MotionConstraints
from new_movement.entities.trajectory.trajectory_segment import TrajectorySegment

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
