import math
from unittest.mock import MagicMock

import pytest

from new_movement.entities.States import State, Vector2D
from new_movement.entities.Trajectory import Trajectory
from new_movement.utilities.trajectory_generator.TrajGenerator import TrajectoryGenerator
from new_movement.tracker_node import (
    TrackerNode,
    build_control_reference_point,
    build_overhead_point,
    tracking_error,
)


def _trajectory(distance: float = 1000.0) -> Trajectory:
    generator = TrajectoryGenerator()
    start = State(Vector2D(0, 0), Vector2D(0, 0))
    goal = State(Vector2D(distance, 0), Vector2D(0, 0))
    return Trajectory(generator.generate(start, goal))


def _tracker(**params) -> TrackerNode:
    """A TrackerNode without the ROS machinery, as in test_movement_manager."""
    tracker = TrackerNode.__new__(TrackerNode)
    tracker.get_logger = MagicMock()
    tracker.overhead_pub = MagicMock()
    tracker.control_reference_pub = MagicMock()
    tracker._handoff_latencies = []
    tracker._tracking_errors = {}
    tracker._divergence_streak = {}
    tracker._diverged = {}

    values = {"divergence_radius": 400.0, "divergence_frames": 3}
    values.update(params)
    tracker.get_parameter = lambda name: MagicMock(value=values[name])

    return tracker


def _pending(trajectory: Trajectory, handoff_stamp: float) -> dict:
    return {
        "trajectory": trajectory,
        "trajectory_msg": trajectory.to_msg(1),
        "handoff_stamp": handoff_stamp,
    }


def test_build_overhead_point():
    trajectory = _trajectory()
    msg = trajectory.to_msg(robot_id=1)
    point = build_overhead_point(
        1, msg, trajectory, time_offset=0.0, lookahead_time=0.1, now_sec=500.0
    )

    assert point is not None
    assert point.robot_id == 1
    assert point.timestamp >= 0.0
    assert len(point.trajectory.segments) > 0


def test_build_control_reference_point():
    trajectory = _trajectory()
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


class TestHandoffClamping:
    """
    A plan that arrives later than its own duration used to be discarded, leaving the
    robot driving the previous trajectory to the previous goal. Short corrective moves
    have the shortest durations, so they were the ones most often thrown away.
    """

    def test_late_plan_is_activated_at_its_goal(self):
        tracker = _tracker()
        trajectory = _trajectory(200.0)
        duration = trajectory.get_total_duration()
        data = {"pending": _pending(trajectory, 100.0)}

        tracker._handle_pending_handoff(
            1, data, now_sec=100.0 + duration + 1.0, lookahead=0.2
        )

        assert data["trajectory"] is trajectory
        assert data["time_offset"] == pytest.approx(duration)
        assert data["pending"] is None

    def test_late_plan_replaces_the_previous_goal(self):
        tracker = _tracker()
        old = _trajectory(5000.0)
        new = _trajectory(200.0)
        duration = new.get_total_duration()
        data = {
            "trajectory": old,
            "trajectory_msg": old.to_msg(1),
            "time_offset": 0.0,
            "pending": _pending(new, 100.0),
        }

        tracker._handle_pending_handoff(
            1, data, now_sec=100.0 + duration + 1.0, lookahead=0.2
        )

        assert data["trajectory"] is new
        reference = new.get_state(data["time_offset"]).position
        assert reference.distance(new.get_destination().position) < 1.0

    def test_on_time_plan_starts_at_its_own_age(self):
        tracker = _tracker()
        trajectory = _trajectory(3000.0)
        data = {"pending": _pending(trajectory, 100.0)}

        tracker._handle_pending_handoff(1, data, now_sec=100.05, lookahead=0.2)

        assert data["time_offset"] == pytest.approx(0.05)

    def test_handoff_in_the_future_waits(self):
        tracker = _tracker()
        data = {"pending": _pending(_trajectory(3000.0), 100.0)}

        tracker._handle_pending_handoff(1, data, now_sec=99.9, lookahead=0.2)

        assert data["pending"] is not None
        assert "trajectory" not in data

    def test_unstamped_plan_starts_at_zero(self):
        tracker = _tracker()
        data = {"pending": _pending(_trajectory(3000.0), 0.0)}

        tracker._handle_pending_handoff(1, data, now_sec=100.0, lookahead=0.2)

        assert data["time_offset"] == pytest.approx(0.0)
        assert tracker._handoff_latencies == []


class TestHandoffLatency:
    def test_latency_is_recorded_per_activated_plan(self):
        tracker = _tracker()
        for stamp in (100.0, 100.5):
            data = {"pending": _pending(_trajectory(3000.0), stamp)}
            tracker._handle_pending_handoff(1, data, now_sec=stamp + 0.04, lookahead=0.2)

        assert tracker._handoff_latencies == [
            pytest.approx(0.04),
            pytest.approx(0.04),
        ]

    def test_summary_clears_the_window(self):
        tracker = _tracker()
        tracker._handoff_latencies = [0.01, 0.02, 0.03]

        tracker._log_handoff_latency()

        assert tracker._handoff_latencies == []
        assert tracker.get_logger().info.called


class TestOverheadFreshness:
    """
    A fresh overhead point is always stamped in the future, which is what makes the
    planner's tight overhead_max_age correct: any positive age means the tracker has
    stopped refreshing it and the planner should fall back to the vision state.
    """

    def test_overhead_point_is_stamped_ahead_of_now(self):
        trajectory = _trajectory(3000.0)
        point = build_overhead_point(
            1, trajectory.to_msg(1), trajectory,
            time_offset=0.0, lookahead_time=0.2, now_sec=500.0,
        )

        assert point.wall_stamp == pytest.approx(500.2)

    def test_overhead_stops_when_the_trajectory_ends(self):
        tracker = _tracker()
        trajectory = _trajectory(1000.0)
        data = {
            "trajectory": trajectory,
            "trajectory_msg": trajectory.to_msg(1),
            "time_offset": trajectory.get_total_duration(),
        }

        tracker._update_active_trajectory(1, data, dt=0.01, now_sec=500.0, lookahead=0.2)

        assert not tracker.overhead_pub.publish.called


class TestTrackingError:
    """
    Vision is delayed, so the reference has to be evaluated at the instant the
    measurement was taken. Comparing against the reference at time_offset would report
    the delay itself as tracking error.
    """

    OFFSET = 1.0
    AGE = 0.1

    def _setup(self):
        trajectory = _trajectory(4000.0)
        reference = trajectory.get_state(self.OFFSET - self.AGE)
        return trajectory, reference

    def _error(self, trajectory, dx, dy):
        reference = trajectory.get_state(self.OFFSET - self.AGE)
        measured = Vector2D(reference.position.x + dx, reference.position.y + dy)
        return tracking_error(trajectory, self.OFFSET, measured, self.AGE)

    def test_robot_on_its_reference_has_no_error(self):
        trajectory, _ = self._setup()
        along, cross = self._error(trajectory, 0.0, 0.0)

        assert along == pytest.approx(0.0, abs=1e-6)
        assert cross == pytest.approx(0.0, abs=1e-6)

    def test_the_measurement_delay_is_not_counted_as_error(self):
        trajectory, reference = self._setup()

        # Comparing against the reference at time_offset instead reports one delay's
        # worth of travel as error.
        naive = trajectory.get_state(self.OFFSET).position.distance(reference.position)
        along, cross = self._error(trajectory, 0.0, 0.0)

        assert naive > 20.0
        assert math.hypot(along, cross) < naive / 100.0

    def test_lagging_robot_reports_negative_along_track(self):
        trajectory, _ = self._setup()
        along, cross = self._error(trajectory, -30.0, 0.0)

        assert along == pytest.approx(-30.0, abs=1e-6)
        assert cross == pytest.approx(0.0, abs=1e-6)

    def test_sideways_displacement_reports_cross_track(self):
        trajectory, _ = self._setup()
        along, cross = self._error(trajectory, 0.0, 50.0)

        assert along == pytest.approx(0.0, abs=1e-6)
        assert abs(cross) == pytest.approx(50.0, abs=1e-6)

    def test_measurement_older_than_the_trajectory_is_dropped(self):
        trajectory = _trajectory(4000.0)

        assert tracking_error(trajectory, 0.0, Vector2D(0, 0), measurement_age=0.1) is None


class TestTrackingErrorSampling:
    def test_one_sample_per_vision_frame_per_robot(self):
        tracker = _tracker()
        tracker.get_clock = MagicMock()
        tracker.get_clock().now().nanoseconds = int(100.1 * 1e9)

        trajectory = _trajectory(4000.0)
        tracker.robot_data = {
            1: {"trajectory": trajectory, "time_offset": 1.0},
        }

        robot = MagicMock()
        robot.id = 1
        robot.position_x, robot.position_y = 0.0, 0.0
        msg = MagicMock()
        msg.ally_robots = [robot]
        msg.vision_wall_stamp = 100.0

        tracker.game_state_callback(msg)
        tracker.game_state_callback(msg)

        assert len(tracker._tracking_errors[1]) == 2

    def test_robots_without_a_trajectory_are_skipped(self):
        tracker = _tracker()
        tracker.get_clock = MagicMock()
        tracker.get_clock().now().nanoseconds = int(100.1 * 1e9)
        tracker.robot_data = {}

        robot = MagicMock()
        robot.id = 7
        robot.position_x, robot.position_y = 0.0, 0.0
        msg = MagicMock()
        msg.ally_robots = [robot]
        msg.vision_wall_stamp = 100.0

        tracker.game_state_callback(msg)

        assert tracker._tracking_errors == {}


def _stationary_trajectory() -> Trajectory:
    """A plan whose start is already its goal: no primitives, zero duration."""
    generator = TrajectoryGenerator()
    at_rest = State(Vector2D(1000, 500), Vector2D(0, 0))
    return Trajectory(generator.generate(at_rest, at_rest))


class TestZeroDurationPlans:
    """
    A start == goal plan has an empty motion path, and sum() of nothing is int 0. That
    int reached GUITrajectories.time_offsets, whose field is float, and killed the node.
    """

    def test_duration_is_a_float(self):
        assert isinstance(_stationary_trajectory().get_total_duration(), float)

    def test_activating_one_keeps_time_offset_a_float(self):
        tracker = _tracker()
        trajectory = _stationary_trajectory()
        data = {"pending": _pending(trajectory, 100.0)}

        tracker._handle_pending_handoff(1, data, now_sec=101.0, lookahead=0.2)

        assert isinstance(data["time_offset"], float)

    def test_the_gui_message_only_carries_floats(self):
        tracker = _tracker()
        tracker.gui_trajectories_pub = MagicMock()
        trajectory = _stationary_trajectory()
        tracker.robot_data = {
            1: {
                "trajectory": trajectory,
                "trajectory_msg": trajectory.to_msg(1),
                "time_offset": 0,
            }
        }

        tracker._update_gui_trajectories()

        msg = tracker.gui_trajectories_pub.publish.call_args[0][0]
        assert all(isinstance(offset, float) for offset in msg.time_offsets)

    def test_the_control_reference_is_still_published(self):
        tracker = _tracker()
        trajectory = _stationary_trajectory()
        data = {
            "trajectory": trajectory,
            "trajectory_msg": trajectory.to_msg(1),
            "time_offset": 0.0,
        }

        tracker._update_active_trajectory(1, data, dt=0.01, now_sec=500.0, lookahead=0.2)

        # Bailing before this left the controller chasing the previous, moving reference.
        assert tracker.control_reference_pub.publish.called
        assert not tracker.overhead_pub.publish.called


class TestDivergence:
    """
    Measured: a robot sat 355mm off its reference for nearly five seconds, peaking at
    966mm, and nothing replanned. Every plan issued in that window was built on a
    prediction the robot had already stopped matching.
    """

    ON_PATH = (10.0, 20.0)
    OFF_PATH = (100.0, 500.0)

    def test_one_bad_frame_is_not_a_divergence(self):
        tracker = _tracker()

        tracker._update_divergence(0, self.OFF_PATH)

        assert tracker._diverged.get(0) is not True

    def test_a_sustained_departure_is(self):
        tracker = _tracker()

        for _ in range(3):
            tracker._update_divergence(0, self.OFF_PATH)

        assert tracker._diverged[0] is True

    def test_a_good_frame_resets_the_streak(self):
        tracker = _tracker()

        tracker._update_divergence(0, self.OFF_PATH)
        tracker._update_divergence(0, self.OFF_PATH)
        tracker._update_divergence(0, self.ON_PATH)
        tracker._update_divergence(0, self.OFF_PATH)

        assert tracker._diverged.get(0) is not True

    def test_recovery_clears_the_flag(self):
        tracker = _tracker()
        for _ in range(3):
            tracker._update_divergence(0, self.OFF_PATH)

        for _ in range(3):
            tracker._update_divergence(0, self.ON_PATH)

        assert tracker._diverged[0] is False

    def test_along_track_error_alone_can_trigger_it(self):
        tracker = _tracker()

        for _ in range(3):
            tracker._update_divergence(0, (-800.0, 0.0))

        assert tracker._diverged[0] is True


class TestDivergenceSuppressesTheOverhead:
    def _active(self, tracker, diverged):
        trajectory = _trajectory(4000.0)
        tracker._diverged[1] = diverged
        data = {
            "trajectory": trajectory,
            "trajectory_msg": trajectory.to_msg(1),
            "time_offset": 0.5,
        }
        tracker._update_active_trajectory(1, data, dt=0.01, now_sec=500.0, lookahead=0.15)
        return data

    def test_the_overhead_point_is_withheld(self):
        tracker = _tracker()

        self._active(tracker, diverged=True)

        # The planner's cache ages out and it falls back to the measured state.
        assert not tracker.overhead_pub.publish.called

    def test_the_control_reference_still_goes_out(self):
        tracker = _tracker()

        self._active(tracker, diverged=True)

        assert tracker.control_reference_pub.publish.called

    def test_the_reference_stops_advancing(self):
        tracker = _tracker()

        data = self._active(tracker, diverged=True)

        assert data["time_offset"] == pytest.approx(0.5)

    def test_a_robot_on_its_path_is_unaffected(self):
        tracker = _tracker()

        data = self._active(tracker, diverged=False)

        assert tracker.overhead_pub.publish.called
        assert data["time_offset"] == pytest.approx(0.51)

    def test_activating_a_new_plan_clears_the_flag(self):
        tracker = _tracker()
        tracker._diverged[1] = True
        tracker._divergence_streak[1] = 9
        data = {"pending": _pending(_trajectory(3000.0), 100.0)}

        tracker._handle_pending_handoff(1, data, now_sec=100.05, lookahead=0.15)

        assert tracker._diverged[1] is False
        assert tracker._divergence_streak[1] == 0
