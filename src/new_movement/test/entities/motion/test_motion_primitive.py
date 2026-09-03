import pytest

from new_movement.entities.motion import MotionPrimitive
from utils.math_util import Vector2D


class TestMotionPrimitive:
    def test_construction(self):
        primitive = MotionPrimitive(acceleration=Vector2D(1, 2), duration=3.0)

        assert primitive.acceleration == Vector2D(1, 2)
        assert primitive.duration == 3.0

    def test_to_msg_populates_fields(self):
        primitive = MotionPrimitive(acceleration=Vector2D(1.5, -2.5), duration=4.0)

        msg = primitive.to_msg()

        assert msg.acceleration.x == 1.5
        assert msg.acceleration.y == -2.5
        assert msg.duration == 4.0
        assert isinstance(msg.duration, float)

    def test_to_msg_casts_int_duration_to_float(self):
        primitive = MotionPrimitive(acceleration=Vector2D(0, 0), duration=2)

        msg = primitive.to_msg()

        assert msg.duration == 2.0
        assert isinstance(msg.duration, float)

    def test_from_msg_reconstructs_primitive(self):
        original = MotionPrimitive(acceleration=Vector2D(3, -4), duration=1.25)
        msg = original.to_msg()

        rebuilt = MotionPrimitive.from_msg(msg)

        assert rebuilt.acceleration == Vector2D(3, -4)
        assert rebuilt.duration == 1.25
        assert isinstance(rebuilt.duration, float)

    def test_round_trip_preserves_values(self):
        original = MotionPrimitive(acceleration=Vector2D(-1.1, 2.2), duration=0.5)

        rebuilt = MotionPrimitive.from_msg(original.to_msg())

        assert rebuilt == original

    def test_getitem_index_0_returns_acceleration_components(self):
        primitive = MotionPrimitive(acceleration=Vector2D(7, 8), duration=1.0)

        assert primitive[0] == (7, 8)

    def test_getitem_index_1_returns_duration(self):
        primitive = MotionPrimitive(acceleration=Vector2D(7, 8), duration=1.0)

        assert primitive[1] == 1.0

    def test_getitem_invalid_index_raises(self):
        primitive = MotionPrimitive(acceleration=Vector2D(7, 8), duration=1.0)

        with pytest.raises(IndexError):
            primitive[2]

    def test_zero_duration(self):
        primitive = MotionPrimitive(acceleration=Vector2D(1, 1), duration=0.0)

        assert primitive.duration == 0.0
        assert primitive[1] == 0.0

    def test_negative_acceleration_components(self):
        primitive = MotionPrimitive(acceleration=Vector2D(-3, -4), duration=1.0)

        assert primitive[0] == (-3, -4)
