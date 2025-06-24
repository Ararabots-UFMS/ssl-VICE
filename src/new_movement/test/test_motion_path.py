import pytest
from new_movement.new_movement.entities.States import Vector2D
from new_movement.new_movement.entities.Motion import MotionPrimitive, MotionPath

def test_motion_path_creation():
    p1 = MotionPrimitive(Vector2D(0, 0), 2)
    p2 = MotionPrimitive(Vector2D(1, 1), Vector2D(2, 2), 3)
    path = MotionPath([p1, p2])
    assert len(path.primitives) == 2
    assert path.duration() == 5


def test_truncate_zero():
    p1 = MotionPrimitive(Vector2D(0, 0), Vector2D(1, 1), 2)
    path = MotionPath([p1])
    path.truncate(0)
    assert len(path.primitives) == 0


def test_truncate_full():
    p1 = MotionPrimitive(Vector2D(0, 0), Vector2D(1, 1), 2)
    p2 = MotionPrimitive(Vector2D(1, 1), Vector2D(2, 2), 3)
    path = MotionPath([p1, p2])
    path.truncate(5)
    assert len(path.primitives) == 2
    assert path.duration() == 5


def test_truncate_partial():
    p1 = MotionPrimitive(Vector2D(0, 0), Vector2D(1, 1), 2)
    p2 = MotionPrimitive(Vector2D(1, 1), Vector2D(2, 2), 3)
    path = MotionPath([p1, p2])
    path.truncate(3)
    assert len(path.primitives) == 2
    assert path.primitives[1].duration == 1
    assert path.primitives[1].end.x == 1 + (1 / 3) * (2 - 1)
    assert path.primitives[1].end.y == 1 + (1 / 3) * (2 - 1)


def test_truncate_on_primitive_boundary():
    p1 = MotionPrimitive(Vector2D(0, 0), Vector2D(1, 1), 2)
    p2 = MotionPrimitive(Vector2D(1, 1), Vector2D(2, 2), 3)
    path = MotionPath([p1, p2])
    path.truncate(2)
    assert len(path.primitives) == 1
    assert path.primitives[0].duration == 2
    assert path.primitives[0].end.x == 1
    assert path.primitives[0].end.y == 1
