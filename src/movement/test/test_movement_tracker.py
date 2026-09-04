import math
from unittest.mock import MagicMock, patch

import pytest

from movement.entities.motion import MotionState
from movement.entities.trajectory import Trajectory
from movement.local_planner import TrajectoryGenerator
from movement.movement_tracker import (
    MovementTracker,
    build_control_reference_point,
    build_overhead_point,
    tracking_error,
)

from movement_interfaces.msg import Trajectory as TrajectoryMsg

from utils.math_util import Vector2D


DEFAULT_PARAMS = {
    "lookahead_time": 0.2,
    "improvement_threshold": 0.1,
    "change_radius": 10,
    "divergence_radius": 400.0,
    "divergence_frames": 3,
    "recovery_frames": 10,
    "divergence_timeout_frames": 120,
}


class FakeTime:
    """Enough of an rclpy.time.Time for the arithmetic MovementTracker does."""

    def __init__(self, seconds: float):
        self.nanoseconds = int(seconds * 1e9)

    def __sub__(self, other):
        return FakeTime((self.nanoseconds - other.nanoseconds) / 1e9)


def _tracker(now_sec: float = 0.0, **params) -> MovementTracker:
    """A MovementTracker without the ROS machinery, as in test_movement_manager."""
    tracker = MovementTracker.__new__(MovementTracker)
    tracker.get_logger = MagicMock()
    tracker.overhead_pub = MagicMock()
    tracker.control_reference_pub = MagicMock()
    tracker.gui_trajectories_pub = MagicMock()
    tracker.robot_data = {}
    tracker._last_warned = {}
    tracker._measured = {}
    tracker._divergence_streak = {}
    tracker._recovery_streak = {}
    tracker._divergence_age = {}
    tracker._diverged = {}

    values = dict(DEFAULT_PARAMS, **params)
    tracker.get_parameter = lambda name: MagicMock(value=values[name])
    _set_clock(tracker, now_sec)
    tracker.last_time = tracker.get_clock().now()
    return tracker


def _set_clock(tracker, now_sec: float) -> None:
    tracker.get_clock = MagicMock(return_value=MagicMock(now=MagicMock(return_value=FakeTime(now_sec))))


@pytest.fixture
def tracker():
    return _tracker()


def _trajectory(distance: float = 1000.0) -> Trajectory:
    generator = TrajectoryGenerator()
    start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
    goal = MotionState(Vector2D(distance, 0), Vector2D(0, 0))
    return Trajectory(generator.generate(start, goal))


def _stationary_trajectory() -> Trajectory:
    """A plan whose start is already its goal: no primitives, zero duration."""
    generator = TrajectoryGenerator()
    at_rest = MotionState(Vector2D(1000, 500), Vector2D(0, 0))
    return Trajectory(generator.generate(at_rest, at_rest))


def _pending(trajectory: Trajectory, handoff_stamp: float) -> dict:
    return {
        "trajectory": trajectory,
        "trajectory_msg": trajectory.to_msg(1),
        "handoff_stamp": handoff_stamp,
    }


def _vision(robot_id: int, position, stamp: float = 100.0):
    robot = MagicMock()
    robot.id = robot_id
    robot.position_x, robot.position_y = position
    robot.velocity_x = robot.velocity_y = 0.0
    msg = MagicMock()
    msg.ally_robots = [robot]
    msg.vision_wall_stamp = stamp
    return msg


def test_build_overhead_point():
    trajectory = _trajectory()
    point = build_overhead_point(
        1, trajectory.to_msg(1), trajectory,
        time_offset=0.0, lookahead_time=0.1, now_sec=500.0,
    )

    assert point.robot_id == 1
    assert len(point.trajectory.segments) > 0
    # Always stamped ahead of now, which is what makes the planner's tight
    # overhead_max_age correct: any positive age means the tracker stopped refreshing.
    assert point.wall_stamp == pytest.approx(500.1)


def test_build_control_reference_point():
    trajectory = _trajectory()
    state = MotionState(Vector2D(1500, -500), Vector2D(300, -200))

    point = build_control_reference_point(2, trajectory.to_msg(2), state, time_offset=0.5)

    assert point.robot_id == 2
    assert (point.pos.x, point.pos.y) == (1500, -500)
    assert (point.vel.x, point.vel.y) == (300, -200)
    assert point.timestamp == 0.5
    assert len(point.trajectory.segments) > 0


def _trajectory_msg(start_xy, goal_xy, robot_id=1, handoff_stamp=0.0):
    generator = TrajectoryGenerator()
    start = MotionState(Vector2D(*start_xy), Vector2D(0, 0))
    goal = MotionState(Vector2D(*goal_xy), Vector2D(0, 0))
    msg = Trajectory(generator.generate(start, goal)).to_msg(robot_id=robot_id)
    msg.handoff_stamp = handoff_stamp
    return msg


class TestTrajectoryCallback:
    def test_empty_segments_ignored(self, tracker):
        msg = MagicMock()
        msg.segments = []

        tracker.trajectory_callback(msg)

        assert tracker.robot_data == {}

    def test_decode_failure_is_logged_and_ignored(self, tracker):
        msg = _trajectory_msg((0, 0), (1000, 0))

        with patch.object(Trajectory, "from_msg", side_effect=ValueError("bad msg")):
            tracker.trajectory_callback(msg)

        assert tracker.robot_data.get(1, {}).get("pending") is None
        tracker.get_logger().warn.assert_called()

    def test_first_trajectory_sets_pending(self, tracker):
        msg = _trajectory_msg((0, 0), (1000, 0), handoff_stamp=5.0)

        tracker.trajectory_callback(msg)

        assert tracker.robot_data[1]["pending"]["handoff_stamp"] == 5.0

    def test_older_handoff_is_discarded(self, tracker):
        tracker.trajectory_callback(_trajectory_msg((0, 0), (1000, 0), handoff_stamp=5.0))
        tracker.trajectory_callback(_trajectory_msg((0, 0), (9000, 0), handoff_stamp=1.0))

        assert tracker.robot_data[1]["pending"]["handoff_stamp"] == 5.0

    def test_goal_changed_replaces_pending(self, tracker):
        tracker.trajectory_callback(_trajectory_msg((0, 0), (1000, 0), handoff_stamp=5.0))
        tracker.trajectory_callback(_trajectory_msg((0, 0), (5000, 0), handoff_stamp=6.0))

        pending = tracker.robot_data[1]["pending"]
        assert pending["handoff_stamp"] == 6.0
        assert pending["trajectory"].get_destination().position.x == pytest.approx(5000)

    def test_same_goal_keeps_pending_without_a_real_improvement(self, tracker):
        tracker.trajectory_callback(_trajectory_msg((0, 0), (1000, 0), handoff_stamp=5.0))
        tracker.trajectory_callback(_trajectory_msg((0, 0), (1000, 0), handoff_stamp=5.01))

        assert tracker.robot_data[1]["pending"]["handoff_stamp"] == 5.0

    def test_same_goal_takes_a_substantial_improvement(self, tracker):
        tracker.trajectory_callback(_trajectory_msg((0, 0), (1000, 0), handoff_stamp=5.0))
        # Starting much closer to the same goal is a far shorter plan.
        tracker.trajectory_callback(_trajectory_msg((950, 0), (1000, 0), handoff_stamp=5.01))

        assert tracker.robot_data[1]["pending"]["handoff_stamp"] == 5.01


class TestTimerCallback:
    def test_skips_when_dt_not_positive(self, tracker):
        tracker._update_active_trajectory = MagicMock()
        tracker._handle_pending_handoff = MagicMock()
        tracker.robot_data = {1: {}}

        tracker.timer_callback()

        tracker._update_active_trajectory.assert_not_called()
        tracker._handle_pending_handoff.assert_not_called()

    def test_dispatches_per_robot(self, tracker):
        tracker._update_active_trajectory = MagicMock()
        tracker._handle_pending_handoff = MagicMock()
        tracker.robot_data = {1: {}, 2: {}}
        _set_clock(tracker, 1.0)

        tracker.timer_callback()

        assert tracker._update_active_trajectory.call_count == 2
        assert tracker._handle_pending_handoff.call_count == 2


class TestUpdateActiveTrajectory:
    def _data(self, trajectory, time_offset=0.0):
        return {
            "trajectory": trajectory,
            "trajectory_msg": trajectory.to_msg(1),
            "time_offset": time_offset,
        }

    def test_noop_when_no_trajectory(self, tracker):
        tracker._update_active_trajectory(1, {"trajectory": None}, 0.5, 1.0, 0.2)

        tracker.control_reference_pub.publish.assert_not_called()

    def test_publishes_the_reference_and_the_overhead_point(self, tracker):
        data = self._data(_trajectory(4000.0))

        tracker._update_active_trajectory(1, data, dt=0.5, now_sec=10.0, lookahead=0.2)

        tracker.control_reference_pub.publish.assert_called_once()
        tracker.overhead_pub.publish.assert_called_once()
        assert data["time_offset"] == pytest.approx(0.5)

    def test_the_offset_is_clamped_and_the_overhead_stops_at_the_end(self, tracker):
        trajectory = _trajectory(1000.0)
        duration = trajectory.get_total_duration()
        data = self._data(trajectory, time_offset=duration)

        tracker._update_active_trajectory(1, data, dt=0.5, now_sec=10.0, lookahead=0.2)

        assert data["time_offset"] == pytest.approx(duration)
        assert not tracker.overhead_pub.publish.called
        # The reference is still published from the last known state.
        tracker.control_reference_pub.publish.assert_called_once()

    def test_a_zero_duration_plan_still_publishes_the_reference(self, tracker):
        """Bailing before it left the controller chasing the previous, moving one."""
        data = self._data(_stationary_trajectory())

        tracker._update_active_trajectory(1, data, dt=0.01, now_sec=500.0, lookahead=0.2)

        assert tracker.control_reference_pub.publish.called
        assert not tracker.overhead_pub.publish.called

    def test_divergence_withholds_the_overhead_but_not_the_reference(self, tracker):
        """
        The planner's cache then ages out and it replans from the measured state. Only
        the point is withheld: freezing the offset too pinned the reference to the
        opening frames of every new trajectory, and the robot crawled.
        """
        tracker._diverged[1] = True
        data = self._data(_trajectory(4000.0), time_offset=0.5)

        tracker._update_active_trajectory(1, data, dt=0.01, now_sec=500.0, lookahead=0.15)

        assert not tracker.overhead_pub.publish.called
        assert tracker.control_reference_pub.publish.called
        assert data["time_offset"] == pytest.approx(0.51)

    def test_a_robot_on_its_path_is_unaffected(self, tracker):
        tracker._diverged[1] = False
        data = self._data(_trajectory(4000.0), time_offset=0.5)

        tracker._update_active_trajectory(1, data, dt=0.01, now_sec=500.0, lookahead=0.15)

        assert tracker.overhead_pub.publish.called
        assert data["time_offset"] == pytest.approx(0.51)


class TestHandlePendingHandoff:
    """
    A plan arriving later than its own duration used to be discarded, leaving the robot
    driving the previous trajectory to the previous goal. Short corrective moves have
    the shortest durations, so they were the ones most often thrown away.
    """

    def test_noop_when_no_pending(self, tracker):
        data = {"pending": None}

        tracker._handle_pending_handoff(1, data, now_sec=10.0, lookahead=0.2)

        assert data["pending"] is None

    def test_an_unstamped_plan_starts_at_zero(self, tracker):
        data = {"pending": _pending(_trajectory(3000.0), 0.0)}

        tracker._handle_pending_handoff(1, data, now_sec=100.0, lookahead=0.2)

        assert data["time_offset"] == pytest.approx(0.0)
        assert data["pending"] is None

    def test_a_handoff_in_the_future_waits(self, tracker):
        pending = _pending(_trajectory(3000.0), 100.0)

        data = {"pending": pending}
        tracker._handle_pending_handoff(1, data, now_sec=99.9, lookahead=0.2)

        assert data["pending"] is pending
        assert "trajectory" not in data

    def test_an_on_time_plan_starts_at_its_own_age(self, tracker):
        data = {"pending": _pending(_trajectory(3000.0), 100.0)}

        tracker._handle_pending_handoff(1, data, now_sec=100.05, lookahead=0.2)

        assert data["time_offset"] == pytest.approx(0.05)

    def test_a_late_plan_is_clamped_to_its_goal_not_discarded(self, tracker):
        old, new = _trajectory(5000.0), _trajectory(200.0)
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
        assert data["time_offset"] == pytest.approx(duration)
        reference = new.get_state(data["time_offset"]).position
        assert reference.distance(new.get_destination().position) < 1.0

    def test_a_zero_duration_plan_keeps_the_offset_a_float(self, tracker):
        """sum() of no primitives is int 0, which GUITrajectories' float field rejects."""
        trajectory = _stationary_trajectory()
        assert isinstance(trajectory.get_total_duration(), float)
        data = {"pending": _pending(trajectory, 100.0)}

        tracker._handle_pending_handoff(1, data, now_sec=101.0, lookahead=0.2)

        assert isinstance(data["time_offset"], float)

    def test_it_leaves_the_divergence_state_alone(self, tracker):
        """
        Resetting the streaks here latched the flag on: while diverged, plans activate
        on arrival faster than a recovery streak of 10 vision frames can accumulate.
        """
        tracker._diverged[1] = True
        tracker._recovery_streak[1] = 7

        tracker._handle_pending_handoff(
            1, {"pending": _pending(_trajectory(3000.0), 100.0)},
            now_sec=100.05, lookahead=0.15,
        )

        assert tracker._diverged[1] is True
        assert tracker._recovery_streak[1] == 7


class TestHandoffReprojection:
    """
    dt_late as the starting offset assumes the robot spent that long following this
    plan. For one built from a measured state, or a recovery stop, the opening stretch
    describes motion the robot never made — half a metre of reference, at 0.28s late,
    placed where the robot has never been.
    """

    def _activate(self, tracker, trajectory, stamp, now_sec):
        data = {"pending": _pending(trajectory, stamp)}
        tracker._handle_pending_handoff(1, data, now_sec=now_sec, lookahead=0.15)
        return data["time_offset"]

    def _measured_at(self, tracker, position, stamp):
        tracker._measured[1] = (Vector2D(*position), Vector2D(0.0, 0.0), stamp)

    def test_it_starts_where_the_robot_actually_is(self, tracker):
        trajectory = _trajectory(4000.0)
        # Stamped 0.28s ago, but the robot never drove the opening stretch.
        start = trajectory.get_state(0.0).position
        self._measured_at(tracker, (start.x, start.y), stamp=100.28)

        assert self._activate(
            tracker, trajectory, stamp=100.0, now_sec=100.28
        ) == pytest.approx(0.0, abs=0.02)

    def test_a_robot_that_did_follow_it_starts_where_it_got_to(self, tracker):
        trajectory = _trajectory(4000.0)
        travelled = trajectory.get_state(0.20).position
        self._measured_at(tracker, (travelled.x, travelled.y), stamp=100.28)

        assert self._activate(
            tracker, trajectory, stamp=100.0, now_sec=100.28
        ) == pytest.approx(0.20, abs=0.03)

    def test_the_reference_never_leads_by_more_than_the_elapsed_time(self, tracker):
        trajectory = _trajectory(4000.0)
        far = trajectory.get_state(1.0).position
        self._measured_at(tracker, (far.x, far.y), stamp=100.05)

        assert self._activate(tracker, trajectory, stamp=100.0, now_sec=100.05) <= 0.05 + 1e-9

    def test_it_falls_back_to_dt_late_without_a_measurement(self, tracker):
        assert self._activate(
            tracker, _trajectory(4000.0), stamp=100.0, now_sec=100.1
        ) == pytest.approx(0.1)


class TestTrackingError:
    """
    Vision is delayed, so the reference has to be evaluated at the instant the
    measurement was taken. Comparing against the reference at time_offset would report
    the delay itself as tracking error.
    """

    OFFSET = 1.0
    AGE = 0.1

    def _error(self, trajectory, dx, dy):
        reference = trajectory.get_state(self.OFFSET - self.AGE)
        measured = Vector2D(reference.position.x + dx, reference.position.y + dy)
        return tracking_error(trajectory, self.OFFSET, measured, self.AGE)

    def test_the_measurement_delay_is_not_counted_as_error(self):
        trajectory = _trajectory(4000.0)
        reference = trajectory.get_state(self.OFFSET - self.AGE)
        naive = trajectory.get_state(self.OFFSET).position.distance(reference.position)

        along, cross = self._error(trajectory, 0.0, 0.0)

        assert naive > 20.0
        assert math.hypot(along, cross) < naive / 100.0

    def test_a_lagging_robot_reports_negative_along_track(self):
        along, cross = self._error(_trajectory(4000.0), -30.0, 0.0)

        assert along == pytest.approx(-30.0, abs=1e-6)
        assert cross == pytest.approx(0.0, abs=1e-6)

    def test_sideways_displacement_reports_cross_track(self):
        along, cross = self._error(_trajectory(4000.0), 0.0, 50.0)

        assert along == pytest.approx(0.0, abs=1e-6)
        assert abs(cross) == pytest.approx(50.0, abs=1e-6)

    @pytest.mark.parametrize("time_offset, age", [(0.0, 0.1), (99.0, 0.0)])
    def test_an_instant_outside_the_plan_is_clamped_not_dropped(self, time_offset, age):
        """
        A plan stamped from vision is activated at an offset equal to that measurement's
        own age, so returning None made the error unmeasurable in exactly the situation
        the divergence flag creates — and the flag could not clear.
        """
        trajectory = _trajectory(4000.0)
        reference = trajectory.get_state(max(time_offset - age, 0.0)).position

        error = tracking_error(trajectory, time_offset, reference, age)

        assert error is not None
        assert math.hypot(*error) == pytest.approx(0.0, abs=1e-6)


class TestDivergence:
    """
    Measured: a robot sat 355mm off its reference for nearly five seconds, peaking at
    966mm, and nothing replanned. Every plan issued in that window was built on a
    prediction the robot had already stopped matching.
    """

    ON_PATH = (10.0, 20.0)
    OFF_PATH = (100.0, 500.0)

    def test_one_bad_frame_is_not_a_divergence(self, tracker):
        tracker._update_divergence(0, self.OFF_PATH)

        assert tracker._diverged.get(0) is not True

    @pytest.mark.parametrize("error", [OFF_PATH, (-800.0, 0.0)])
    def test_a_sustained_departure_is(self, tracker, error):
        for _ in range(3):
            tracker._update_divergence(0, error)

        assert tracker._diverged[0] is True

    def test_a_good_frame_resets_the_streak(self, tracker):
        for error in (self.OFF_PATH, self.OFF_PATH, self.ON_PATH, self.OFF_PATH):
            tracker._update_divergence(0, error)

        assert tracker._diverged.get(0) is not True

    def test_clearing_the_flag_needs_a_sustained_recovery(self, tracker):
        """Asymmetric hysteresis: the flag survives a frame of luck."""
        for _ in range(3):
            tracker._update_divergence(0, self.OFF_PATH)

        tracker._update_divergence(0, self.ON_PATH)
        assert tracker._diverged[0] is True

        for _ in range(8):
            tracker._update_divergence(0, self.ON_PATH)
        tracker._update_divergence(0, self.OFF_PATH)
        for _ in range(9):
            tracker._update_divergence(0, self.ON_PATH)
        assert tracker._diverged[0] is True

        tracker._update_divergence(0, self.ON_PATH)
        assert tracker._diverged[0] is False

    def test_recovery_survives_a_handoff_on_every_other_frame(self):
        """
        While diverged, plans are stamped in the past and activate on arrival — measured
        at ~42/s against vision's 60/s. A recovery streak any handoff could reset never
        reached the 10 consecutive frames it needed, so the flag latched on.
        """
        vision_hz, handoff_hz, seconds = 60.0, 42.0, 5.0
        vision_age = 0.09
        tracker = _tracker(now_sec=100.0 + vision_age)
        trajectory = _trajectory(4000.0)

        tracker._diverged[0] = True
        tracker.robot_data = {0: {"trajectory": trajectory, "time_offset": vision_age}}
        on_path = trajectory.get_state(0.0).position
        msg = _vision(0, (on_path.x, on_path.y))

        events = (
            [(k / vision_hz, "vision") for k in range(int(seconds * vision_hz))]
            + [(k / handoff_hz, "handoff") for k in range(int(seconds * handoff_hz))]
        )
        for _, kind in sorted(events):
            if kind == "vision":
                tracker.game_state_callback(msg)
            else:
                tracker._handle_pending_handoff(
                    0, {"pending": _pending(trajectory, 100.0)},
                    now_sec=100.0 + vision_age, lookahead=0.15,
                )

        assert tracker._diverged[0] is False

    def test_the_flag_cannot_be_held_indefinitely(self):
        """Safety valve: this flag has latched twice, so it is bounded by construction."""
        tracker = _tracker(divergence_timeout_frames=30)
        tracker._diverged[0] = True

        for _ in range(30):
            tracker._update_divergence(0, (600.0, 0.0))  # never recovering

        assert tracker._diverged[0] is False
        assert tracker.get_logger().warn.called


class TestWarningVolume:
    """
    Every condition warned about here repeats at the planner's rate for as long as it
    lasts. Unthrottled, the log reported the planning frequency rather than the event.
    """

    def _late_handoff(self, tracker, trajectory, now_sec):
        tracker._handle_pending_handoff(
            1, {"pending": _pending(trajectory, 100.0)}, now_sec=now_sec, lookahead=0.15
        )

    def test_a_repeating_condition_is_reported_once_per_period(self, tracker):
        trajectory = _trajectory(3000.0)
        base = 100.0 + trajectory.get_total_duration() + 1.0

        # 50 plans over half a second, every one fully elapsed on arrival.
        for k in range(50):
            self._late_handoff(tracker, trajectory, now_sec=base + k * 0.01)
        assert tracker.get_logger().warn.call_count == 1

        self._late_handoff(tracker, trajectory, now_sec=base + 5.0)
        assert tracker.get_logger().warn.call_count == 2

    def test_a_plan_shorter_than_the_lookahead_is_not_a_fault(self, tracker):
        """
        Its whole duration is under the planning latency, so it could never have been
        caught in time. Near the goal that is most plans, and it is what the clamp is for.
        """
        trajectory = _trajectory(1.0)
        duration = trajectory.get_total_duration()
        assert duration < 0.15

        self._late_handoff(tracker, trajectory, now_sec=100.0 + duration + 0.1)

        assert not tracker.get_logger().warn.called

    def test_robots_are_throttled_independently(self, tracker):
        trajectory = _trajectory(3000.0)
        now = 100.0 + trajectory.get_total_duration() + 1.0

        for robot_id in (1, 2, 3):
            tracker._handle_pending_handoff(
                robot_id, {"pending": _pending(trajectory, 100.0)},
                now_sec=now, lookahead=0.15,
            )

        assert tracker.get_logger().warn.call_count == 3


class TestUpdateGuiTrajectories:
    def test_builds_message_and_skips_uninitialized_robots(self, tracker):
        trajectory = _trajectory(1000.0)
        pending_msg = trajectory.to_msg(1)
        tracker.robot_data = {
            1: {
                "trajectory_msg": trajectory.to_msg(1),
                "pending": {"trajectory_msg": pending_msg},
                "time_offset": 0,  # int, as a zero-duration plan leaves it
            },
            2: {"trajectory_msg": None, "pending": None, "time_offset": 0.0},
        }

        tracker._update_gui_trajectories()

        published = tracker.gui_trajectories_pub.publish.call_args[0][0]
        assert len(published.current_trajectories) == 1
        assert published.pending_trajectories == [pending_msg]
        # The float() matters: GUITrajectories.time_offsets rejects an int.
        assert all(isinstance(offset, float) for offset in published.time_offsets)

    def test_no_pending_uses_an_empty_trajectory_msg(self, tracker):
        trajectory = _trajectory(1000.0)
        tracker.robot_data = {
            1: {"trajectory_msg": trajectory.to_msg(1), "pending": None, "time_offset": 0.0},
        }

        tracker._update_gui_trajectories()

        published = tracker.gui_trajectories_pub.publish.call_args[0][0]
        assert isinstance(published.pending_trajectories[0], TrajectoryMsg)
