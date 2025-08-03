import pytest
from new_movement.new_movement.entities.States import Vector2D
from new_movement.new_movement.entities.Motion import MotionPrimitive, MotionPath

# python



@pytest.fixture
def mock_vector2d():
    return Vector2D(1.0, 1.0)
def make_motion_path(durations, vector2d):
    return MotionPath([MotionPrimitive(vector2d, d) for d in durations])

def test_truncate_zero_or_negative(mock_vector2d):
    path = make_motion_path([1.0, 2.0], mock_vector2d)
    path.truncate(0)
    assert path.motion_path == []
    path = make_motion_path([1.0, 2.0], mock_vector2d)
    path.truncate(-5)
    assert path.motion_path == []

def test_truncate_greater_than_total(mock_vector2d):
    path = make_motion_path([1.0, 2.0], mock_vector2d)
    original = list(path.motion_path)
    path.truncate(5)
    assert path.motion_path == original

def test_truncate_exact_total(mock_vector2d):
    path = make_motion_path([1.0, 2.0], mock_vector2d)
    original = list(path.motion_path)
    path.truncate(3.0)
    assert path.motion_path == original

def test_truncate_partial(mock_vector2d):
    path = make_motion_path([1.0, 2.0, 3.0], mock_vector2d)
    path.truncate(2.5)
    assert len(path.motion_path) == 2
    assert path.motion_path[0].duration == 1.0
    assert pytest.approx(path.motion_path[1].duration) == 1.5

def test_truncate_partial_exact_on_boundary(mock_vector2d):
    path = make_motion_path([1.0, 2.0, 3.0], mock_vector2d)
    path.truncate(1.0)
    assert len(path.motion_path) == 1
    assert path.motion_path[0].duration == 1.0

def test_truncate_empty_path(mock_vector2d):
    path = MotionPath([])
    path.truncate(1.0)
    assert path.motion_path == []