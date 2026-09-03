import copy

import pytest
from unittest.mock import MagicMock, patch

from new_movement.entities.motion import MotionState
from new_movement.entities.trajectory import Trajectory
from new_movement.local_planner import TrajectoryGenerator
from new_movement.movement_tracker import (
    MovementTracker,
    build_control_reference_point,
    build_overhead_point,
)

from movement_interfaces.msg import Trajectory as TrajectoryMsg

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


# ---------------------------------------------------------------------------
# Node-level tests for MovementTracker
# ---------------------------------------------------------------------------

class FakeTime:
    """Minimal stand-in for an rclpy.time.Time supporting subtraction and
    .nanoseconds, enough for the arithmetic done in MovementTracker."""

    def __init__(self, seconds: float):
        self.nanoseconds = int(seconds * 1e9)

    def __sub__(self, other):
        result = FakeTime(0)
        result.nanoseconds = self.nanoseconds - other.nanoseconds
        return result


DEFAULT_PARAMS = {
    "lookahead_time": 0.2,
    "improvement_threshold": 0.1,
    "control_reference_topic": "movement_tracker/control_reference",
    "change_radius": 10,
}


@pytest.fixture
def tracker_node():
    params = dict(DEFAULT_PARAMS)

    def fake_get_parameter(name):
        return MagicMock(value=params[name])

    with patch('rclpy.init'), \
         patch('rclpy.node.Node.__init__', return_value=None), \
         patch.object(MovementTracker, 'create_subscription', return_value=MagicMock()), \
         patch.object(MovementTracker, 'create_publisher', return_value=MagicMock()), \
         patch.object(MovementTracker, 'create_timer', return_value=MagicMock()), \
         patch.object(MovementTracker, 'declare_parameter', return_value=MagicMock()), \
         patch.object(MovementTracker, 'get_parameter', side_effect=fake_get_parameter), \
         patch.object(MovementTracker, 'get_logger', return_value=MagicMock()), \
         patch.object(MovementTracker, 'get_clock', return_value=MagicMock(now=MagicMock(return_value=FakeTime(0.0)))):
        node = MovementTracker()

    # Re-bind as plain instance attributes so they keep working after the
    # `with` block above exits (class-level patches are undone on exit, but
    # MovementTracker calls get_parameter/get_logger/get_clock again at
    # runtime from trajectory_callback/timer_callback, well past __init__).
    node.get_parameter = MagicMock(side_effect=fake_get_parameter)
    node.get_logger = MagicMock()
    node.get_clock = MagicMock(now=MagicMock(return_value=FakeTime(0.0)))
    node._test_params = params
    return node


def _trajectory_msg(start_xy, goal_xy, robot_id=1, handoff_stamp=0.0):
    generator = TrajectoryGenerator()
    start = MotionState(Vector2D(*start_xy), Vector2D(0, 0))
    goal = MotionState(Vector2D(*goal_xy), Vector2D(0, 0))
    segment = generator.generate(start, goal)
    trajectory = Trajectory(segment)
    msg = trajectory.to_msg(robot_id=robot_id)
    msg.handoff_stamp = handoff_stamp
    return msg, trajectory


class TestTrajectoryCallback:
    def test_empty_segments_ignored(self, tracker_node):
        msg = MagicMock()
        msg.segments = []

        tracker_node.trajectory_callback(msg)

        assert tracker_node.robot_data == {}

    def test_decode_failure_is_logged_and_ignored(self, tracker_node):
        msg = MagicMock()
        msg.segments = [MagicMock()]
        msg.robot_id = 1

        with patch('new_movement.movement_tracker.Trajectory.from_msg', side_effect=ValueError("bad msg")):
            tracker_node.trajectory_callback(msg)

        assert tracker_node.robot_data == {}
        tracker_node.get_logger().warn.assert_called()

    def test_first_trajectory_sets_pending(self, tracker_node):
        msg, trajectory = _trajectory_msg((0, 0), (1000, 0), robot_id=1, handoff_stamp=5.0)

        tracker_node.trajectory_callback(msg)

        assert 1 in tracker_node.robot_data
        pending = tracker_node.robot_data[1]["pending"]
        assert pending is not None
        assert pending["handoff_stamp"] == 5.0
        assert pending["trajectory_msg"] is not msg  # deep-copied
        assert pending["trajectory_msg"].robot_id == msg.robot_id

    def test_older_handoff_is_discarded(self, tracker_node):
        first_msg, _ = _trajectory_msg((0, 0), (1000, 0), robot_id=1, handoff_stamp=5.0)
        tracker_node.trajectory_callback(first_msg)
        original_pending = tracker_node.robot_data[1]["pending"]

        older_msg, _ = _trajectory_msg((0, 0), (2000, 0), robot_id=1, handoff_stamp=3.0)
        tracker_node.trajectory_callback(older_msg)

        assert tracker_node.robot_data[1]["pending"] is original_pending

    def test_goal_changed_replaces_pending(self, tracker_node):
        first_msg, _ = _trajectory_msg((0, 0), (10, 0), robot_id=1, handoff_stamp=1.0)
        tracker_node.trajectory_callback(first_msg)

        # New goal is far away (default change_radius is 10mm)
        far_msg, _ = _trajectory_msg((0, 0), (5000, 5000), robot_id=1, handoff_stamp=2.0)
        tracker_node.trajectory_callback(far_msg)

        pending = tracker_node.robot_data[1]["pending"]
        assert pending["handoff_stamp"] == 2.0

    def test_same_goal_no_improvement_keeps_pending(self, tracker_node):
        # Identical start/goal/handoff_stamp -> improvement is exactly zero,
        # which never clears the >= threshold check, so pending must stay.
        first_msg, _ = _trajectory_msg((0, 0), (3000, 0), robot_id=1, handoff_stamp=1.0)
        tracker_node.trajectory_callback(first_msg)
        original_pending = tracker_node.robot_data[1]["pending"]

        second_msg, _ = _trajectory_msg((0, 0), (3000, 0), robot_id=1, handoff_stamp=1.0)
        tracker_node.trajectory_callback(second_msg)

        assert tracker_node.robot_data[1]["pending"] is original_pending

    def test_same_goal_large_improvement_replaces_pending(self, tracker_node):
        # Pending trajectory starts far from the goal (long duration); the
        # new one starts almost at the goal (short duration) -> a large
        # relative improvement that must clear the 0.1 threshold.
        first_msg, _ = _trajectory_msg((0, 0), (10000, 0), robot_id=1, handoff_stamp=1.0)
        tracker_node.trajectory_callback(first_msg)
        original_pending = tracker_node.robot_data[1]["pending"]

        second_msg, _ = _trajectory_msg((9900, 0), (10000, 0), robot_id=1, handoff_stamp=1.0)
        tracker_node.trajectory_callback(second_msg)

        pending = tracker_node.robot_data[1]["pending"]
        assert pending is not original_pending
        assert pending["handoff_stamp"] == 1.0


class TestTimerCallback:
    def test_skips_when_dt_not_positive(self, tracker_node):
        tracker_node.last_time = FakeTime(5.0)
        tracker_node.get_clock = MagicMock(now=MagicMock(return_value=FakeTime(5.0)))
        tracker_node._update_active_trajectory = MagicMock()
        tracker_node._handle_pending_handoff = MagicMock()
        tracker_node.robot_data = {1: {}}

        tracker_node.timer_callback()

        tracker_node._update_active_trajectory.assert_not_called()
        tracker_node._handle_pending_handoff.assert_not_called()
        # last_time is only updated once dt > 0, so it stays at 5.0
        assert tracker_node.last_time.nanoseconds == FakeTime(5.0).nanoseconds

    def test_dispatches_per_robot(self, tracker_node):
        tracker_node.last_time = FakeTime(0.0)
        tracker_node.get_clock = MagicMock(now=MagicMock(return_value=FakeTime(1.0)))
        tracker_node._update_active_trajectory = MagicMock()
        tracker_node._handle_pending_handoff = MagicMock()
        tracker_node.robot_data = {1: {"foo": "bar"}, 2: {"foo": "baz"}}

        tracker_node.timer_callback()

        assert tracker_node._update_active_trajectory.call_count == 2
        assert tracker_node._handle_pending_handoff.call_count == 2
        assert tracker_node.last_time.nanoseconds == FakeTime(1.0).nanoseconds


class TestUpdateActiveTrajectory:
    def _make_trajectory(self, duration):
        traj = MagicMock()
        traj.get_total_duration.return_value = duration
        traj.get_state.return_value = MotionState(Vector2D(100, 200), Vector2D(10, 20))
        return traj

    def test_noop_when_no_trajectory(self, tracker_node):
        data = {"trajectory": None}
        tracker_node._update_active_trajectory(1, data, dt=0.5, now_sec=1.0, lookahead=0.2)
        tracker_node.control_reference_pub.publish.assert_not_called()

    def test_noop_when_duration_not_positive(self, tracker_node):
        traj = self._make_trajectory(0.0)
        data = {"trajectory": traj}
        tracker_node._update_active_trajectory(1, data, dt=0.5, now_sec=1.0, lookahead=0.2)
        tracker_node.control_reference_pub.publish.assert_not_called()

    def test_publishes_control_reference_and_overhead(self, tracker_node):
        traj = self._make_trajectory(5.0)
        msg_stub = MagicMock()
        data = {"trajectory": traj, "trajectory_msg": msg_stub, "time_offset": 0.0}

        tracker_node._update_active_trajectory(1, data, dt=0.5, now_sec=10.0, lookahead=0.2)

        tracker_node.control_reference_pub.publish.assert_called_once()
        tracker_node.overhead_pub.publish.assert_called_once()
        assert data["time_offset"] == pytest.approx(0.5)

    def test_time_offset_clamped_to_duration(self, tracker_node):
        traj = self._make_trajectory(1.0)
        msg_stub = MagicMock()
        data = {"trajectory": traj, "trajectory_msg": msg_stub, "time_offset": 0.9}

        tracker_node._update_active_trajectory(1, data, dt=0.5, now_sec=10.0, lookahead=0.2)

        assert data["time_offset"] == pytest.approx(1.0)

    def test_overhead_not_published_once_past_duration(self, tracker_node):
        traj = self._make_trajectory(1.0)
        msg_stub = MagicMock()
        data = {"trajectory": traj, "trajectory_msg": msg_stub, "time_offset": 1.0}

        tracker_node._update_active_trajectory(1, data, dt=0.5, now_sec=10.0, lookahead=0.2)

        tracker_node.overhead_pub.publish.assert_not_called()
        # Control reference is still published from the last known state.
        tracker_node.control_reference_pub.publish.assert_called_once()
        assert data["time_offset"] == pytest.approx(1.0)


class TestHandlePendingHandoff:
    def _pending_trajectory(self, duration):
        traj = MagicMock()
        traj.get_total_duration.return_value = duration
        return traj

    def test_noop_when_no_pending(self, tracker_node):
        data = {"pending": None}
        tracker_node._handle_pending_handoff(1, data, now_sec=10.0, lookahead=0.2)
        assert data["pending"] is None

    def test_handoff_stamp_zero_activates_immediately(self, tracker_node):
        traj = self._pending_trajectory(5.0)
        traj_msg = MagicMock()
        data = {"pending": {"trajectory": traj, "trajectory_msg": traj_msg, "handoff_stamp": 0.0}}

        tracker_node._handle_pending_handoff(1, data, now_sec=10.0, lookahead=0.2)

        assert data["trajectory"] is traj
        assert data["trajectory_msg"] is traj_msg
        assert data["time_offset"] == 0.0
        assert data["pending"] is None

    def test_future_handoff_not_yet_due(self, tracker_node):
        traj = self._pending_trajectory(5.0)
        pending = {"trajectory": traj, "trajectory_msg": MagicMock(), "handoff_stamp": 20.0}
        data = {"pending": pending}

        tracker_node._handle_pending_handoff(1, data, now_sec=10.0, lookahead=0.2)

        # Not due yet: pending untouched, no trajectory activated.
        assert data["pending"] is pending
        assert "trajectory" not in data

    def test_late_handoff_activates_and_warns(self, tracker_node):
        traj = self._pending_trajectory(5.0)
        pending = {"trajectory": traj, "trajectory_msg": MagicMock(), "handoff_stamp": 5.0}
        data = {"pending": pending}

        # now_sec - handoff_stamp = 5.0 which is >> lookahead * 0.5
        tracker_node._handle_pending_handoff(1, data, now_sec=10.0, lookahead=0.2)

        assert data["trajectory"] is traj
        assert data["time_offset"] == pytest.approx(5.0)
        assert data["pending"] is None
        tracker_node.get_logger().warn.assert_called()

    def test_fully_elapsed_pending_is_discarded_with_warning(self, tracker_node):
        traj = self._pending_trajectory(1.0)  # duration shorter than dt_late
        pending = {"trajectory": traj, "trajectory_msg": MagicMock(), "handoff_stamp": 5.0}
        data = {"pending": pending, "trajectory": "unchanged"}

        tracker_node._handle_pending_handoff(1, data, now_sec=20.0, lookahead=0.2)

        # dt_late = 15.0 >= duration 1.0 -> discarded, original trajectory untouched
        assert data["trajectory"] == "unchanged"
        assert data["pending"] is None
        tracker_node.get_logger().warn.assert_called()


class TestUpdateGuiTrajectories:
    def test_builds_message_and_skips_uninitialized_robots(self, tracker_node):
        active_msg = MagicMock()
        active_msg.segments = [MagicMock()]
        pending_msg = MagicMock()

        tracker_node.robot_data = {
            1: {"trajectory_msg": active_msg, "pending": {"trajectory_msg": pending_msg}, "time_offset": 1.5},
            2: {"trajectory_msg": None, "pending": None, "time_offset": 0.0},  # not yet activated
        }

        tracker_node._update_gui_trajectories()

        tracker_node.gui_trajectories_pub.publish.assert_called_once()
        published = tracker_node.gui_trajectories_pub.publish.call_args[0][0]
        assert published.current_trajectories == [active_msg]
        assert published.pending_trajectories == [pending_msg]
        assert published.time_offsets == [1.5]

    def test_no_pending_uses_empty_trajectory_msg(self, tracker_node):
        active_msg = MagicMock()
        active_msg.segments = [MagicMock()]

        tracker_node.robot_data = {
            1: {"trajectory_msg": active_msg, "pending": None, "time_offset": 0.0},
        }

        tracker_node._update_gui_trajectories()

        published = tracker_node.gui_trajectories_pub.publish.call_args[0][0]
        assert len(published.pending_trajectories) == 1
        assert isinstance(published.pending_trajectories[0], TrajectoryMsg)
