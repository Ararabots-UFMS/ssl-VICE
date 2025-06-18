import pytest
from new_movement.new_movement.entities.Trajectory import TrajectorySegment
from new_movement.new_movement.entities.States import Vector2D, State
from new_movement.new_movement.entities.Motion import MotionPrimitive, MotionPath

# Funções mock para substituir os integradores
from new_movement.new_movement.helpers.BB_steer import (
    integrate_control_2d_at_time as integrate_t,
    integrate_control_2d as integrate,
)

@pytest.fixture
def empty_motion_path():
    return MotionPath([])

@pytest.fixture
def basic_motion_path():
    return MotionPath([
        MotionPrimitive(acceleration=Vector2D(1, 0), duration=1.0),
        MotionPrimitive(acceleration=Vector2D(0, 1), duration=2.0)
    ])

def test_constructor_trajectory_segment(basic_motion_path):
    initPos = Vector2D(0, 0)
    initVel = Vector2D(1, 1)
    segment = TrajectorySegment(initPos, initVel, basic_motion_path)

    assert segment.initPos == initPos
    assert segment.initVel == initVel
    assert segment.motionPath == basic_motion_path
    assert segment.child is None

def test_get_local_time(basic_motion_path):
    segment = TrajectorySegment(Vector2D(0, 0), Vector2D(0, 0), basic_motion_path)
    assert segment.get_local_time() == 3.0

def test_add_child_valid(basic_motion_path):
    seg1 = TrajectorySegment(Vector2D(0, 0), Vector2D(0, 0), basic_motion_path)
    dest_state = seg1.get_local_destination()

    seg2 = TrajectorySegment(dest_state.position, dest_state.velocity, basic_motion_path)
    seg1.add_child(seg2)

    assert seg1.child == seg2

def test_add_child_invalid(basic_motion_path):
    seg1 = TrajectorySegment(Vector2D(0, 0), Vector2D(0, 0), basic_motion_path)
    seg2 = TrajectorySegment(Vector2D(100, 100), Vector2D(0, 0), basic_motion_path)

    with pytest.raises(Exception, match="non continuous trajectory"):
        seg1.add_child(seg2)

def test_get_total_time_with_child(basic_motion_path):
    seg1 = TrajectorySegment(Vector2D(0, 0), Vector2D(0, 0), basic_motion_path)
    dest_state = seg1.get_local_destination()
    seg2 = TrajectorySegment(dest_state.position, dest_state.velocity, basic_motion_path)
    seg1.add_child(seg2)

    assert seg1.get_total_time() == 6.0

def test_get_local_destination_returns_state(basic_motion_path):
    seg = TrajectorySegment(Vector2D(0, 0), Vector2D(0, 0), basic_motion_path)
    state = seg.get_local_destination()

    assert isinstance(state, State)
    assert isinstance(state.position, Vector2D)
    assert isinstance(state.velocity, Vector2D)

def test_get_destination_terminal_segment(basic_motion_path):
    seg = TrajectorySegment(Vector2D(0, 0), Vector2D(0, 0), basic_motion_path)
    state = seg.get_destination()

    assert isinstance(state, State)

def test_get_destination_with_child(basic_motion_path):
    seg1 = TrajectorySegment(Vector2D(0, 0), Vector2D(0, 0), basic_motion_path)
    dest = seg1.get_local_destination()
    seg2 = TrajectorySegment(dest.position, dest.velocity, basic_motion_path)
    seg1.add_child(seg2)

    dest_state = seg1.get_destination()
    assert isinstance(dest_state, State)
