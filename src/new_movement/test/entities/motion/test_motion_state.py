import pytest

from new_movement.entities.motion import MotionState
from utils.math_util import Vector2D


class TestMotionState:
    def test_construction_without_acceleration(self):
        state = MotionState(position=Vector2D(1, 2), velocity=Vector2D(3, 4))

        assert state.position == Vector2D(1, 2)
        assert state.velocity == Vector2D(3, 4)
        assert state.acceleration is None

    def test_construction_with_acceleration(self):
        state = MotionState(
            position=Vector2D(1, 2),
            velocity=Vector2D(3, 4),
            acceleration=Vector2D(5, 6),
        )

        assert state.acceleration == Vector2D(5, 6)

    def test_positional_args(self):
        state = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        assert state.position == Vector2D(0, 0)
        assert state.velocity == Vector2D(0, 0)
        assert state.acceleration is None

    def test_zero_vectors(self):
        state = MotionState(Vector2D(0, 0), Vector2D(0, 0), Vector2D(0, 0))
        assert state.position.x == 0
        assert state.position.y == 0
        assert state.velocity.x == 0
        assert state.velocity.y == 0
        assert state.acceleration.x == 0
        assert state.acceleration.y == 0

    def test_negative_values(self):
        state = MotionState(Vector2D(-1, -2), Vector2D(-3, -4), Vector2D(-5, -6))
        assert state.position == Vector2D(-1, -2)
        assert state.velocity == Vector2D(-3, -4)
        assert state.acceleration == Vector2D(-5, -6)

    def test_equality_between_equivalent_states(self):
        state1 = MotionState(Vector2D(1, 2), Vector2D(3, 4))
        state2 = MotionState(Vector2D(1, 2), Vector2D(3, 4))

        assert state1 == state2

    def test_inequality_with_different_fields(self):
        state1 = MotionState(Vector2D(1, 2), Vector2D(3, 4))
        state2 = MotionState(Vector2D(1, 2), Vector2D(3, 5))

        assert state1 != state2

    def test_inequality_when_acceleration_differs(self):
        state1 = MotionState(Vector2D(1, 2), Vector2D(3, 4), Vector2D(0, 0))
        state2 = MotionState(Vector2D(1, 2), Vector2D(3, 4), None)

        assert state1 != state2
