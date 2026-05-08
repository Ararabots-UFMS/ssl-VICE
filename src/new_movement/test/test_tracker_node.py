from new_movement.entities.States import State, Vector2D
from new_movement.entities.Trajectory import Trajectory
from new_movement.utilities.trajectory_generator.TrajGenerator import TrajectoryGenerator
from new_movement.tracker_node import (
    build_control_reference_point,
    build_overhead_point,
)


def test_build_overhead_point():
    generator = TrajectoryGenerator()
    start = State(Vector2D(0, 0), Vector2D(0, 0))
    goal = State(Vector2D(1000, 0), Vector2D(0, 0))
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
    start = State(Vector2D(0, 0), Vector2D(0, 0))
    goal = State(Vector2D(1000, 0), Vector2D(0, 0))
    segment = generator.generate(start, goal)
    trajectory = Trajectory(segment)
    msg = trajectory.to_msg(robot_id=2)

    state = State(Vector2D(1500, -500), Vector2D(300, -200))
    point = build_control_reference_point(2, msg, state, time_offset=0.5)

    assert point is not None
    assert point.robot_id == 2
    assert point.pos.x == 1500
    assert point.pos.y == -500
    assert point.vel.x == 300
    assert point.vel.y == -200
    assert point.timestamp == 0.5
    assert len(point.trajectory.segments) > 0
