import pytest

from new_movement.entities.motion import MotionConstraints
from utils.math_util import Vector2D


class TestMotionConstraints:
    def test_default_min_values_are_negated_max_values(self):
        constraints = MotionConstraints(
            max_velocity=Vector2D(10, 20),
            max_acceleration=Vector2D(30, 40),
        )

        assert constraints.min_velocity == Vector2D(-10, -20)
        assert constraints.min_acceleration == Vector2D(-30, -40)

    def test_explicit_min_values_are_kept(self):
        constraints = MotionConstraints(
            max_velocity=Vector2D(10, 20),
            max_acceleration=Vector2D(30, 40),
            min_velocity=Vector2D(-1, -1),
            min_acceleration=Vector2D(-2, -2),
        )

        assert constraints.min_velocity == Vector2D(-1, -1)
        assert constraints.min_acceleration == Vector2D(-2, -2)

    def test_only_min_velocity_explicit(self):
        constraints = MotionConstraints(
            max_velocity=Vector2D(10, 20),
            max_acceleration=Vector2D(30, 40),
            min_velocity=Vector2D(-5, -5),
        )

        assert constraints.min_velocity == Vector2D(-5, -5)
        # min_acceleration should still be derived from max_acceleration
        assert constraints.min_acceleration == Vector2D(-30, -40)

    def test_only_min_acceleration_explicit(self):
        constraints = MotionConstraints(
            max_velocity=Vector2D(10, 20),
            max_acceleration=Vector2D(30, 40),
            min_acceleration=Vector2D(-7, -7),
        )

        assert constraints.min_velocity == Vector2D(-10, -20)
        assert constraints.min_acceleration == Vector2D(-7, -7)

    def test_zero_max_velocity_produces_zero_min_velocity(self):
        constraints = MotionConstraints(
            max_velocity=Vector2D(0, 0),
            max_acceleration=Vector2D(0, 0),
        )

        assert constraints.min_velocity == Vector2D(0, 0)
        assert constraints.min_acceleration == Vector2D(0, 0)

    def test_negative_max_values_flip_sign_for_min(self):
        constraints = MotionConstraints(
            max_velocity=Vector2D(-5, 3),
            max_acceleration=Vector2D(2, -8),
        )

        assert constraints.min_velocity == Vector2D(5, -3)
        assert constraints.min_acceleration == Vector2D(-2, 8)

    def test_asymmetric_components(self):
        constraints = MotionConstraints(
            max_velocity=Vector2D(100, 50),
            max_acceleration=Vector2D(20, 10),
        )

        assert constraints.min_velocity.x == -100
        assert constraints.min_velocity.y == -50
        assert constraints.min_acceleration.x == -20
        assert constraints.min_acceleration.y == -10
