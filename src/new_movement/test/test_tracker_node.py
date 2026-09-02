from new_movement.entities.motion import MotionState
from new_movement.entities.trajectory import Trajectory
from new_movement.local_planner import TrajectoryGenerator
from new_movement.movement_tracker import (
    build_control_reference_point,
    build_overhead_point,
)

from utils.math_util import Vector2D


def test_build_overhead_point():
    generator = TrajectoryGenerator()
    start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
    goal = MotionState(Vector2D(1000, 0), Vector2D(0, 0))
    segment = generator.generate(start, goal)
    trajectory = Trajectory(segment)

    msg = trajectory.to_msg(robot_id=1)
    point = build_overhead_point(1, msg, trajectory, time_offset=0.0, lookahead_time=0.1)

    assert point is not None
    assert point.robot_id == 1
    assert point.timestamp >= 0.0
    assert len(point.trajectory.segments) > 0


def test_build_control_reference_point():
    generator = TrajectoryGenerator()
    start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
    goal = MotionState(Vector2D(1000, 0), Vector2D(0, 0))
    segment = generator.generate(start, goal)
    trajectory = Trajectory(segment)
    msg = trajectory.to_msg(robot_id=2)

    state = MotionState(Vector2D(1500, -500), Vector2D(300, -200))
    point = build_control_reference_point(2, msg, state, time_offset=0.5)

    assert point is not None
    assert point.robot_id == 2
    assert point.pos.x == 1500
    assert point.pos.y == -500
    assert point.vel.x == 300
    assert point.vel.y == -200
    assert point.timestamp == 0.5
    assert len(point.trajectory.segments) > 0
