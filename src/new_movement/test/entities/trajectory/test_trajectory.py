import pytest

from new_movement.entities.motion import MotionPath, MotionPrimitive
from new_movement.entities.trajectory import Trajectory, TrajectorySegment
from utils.math_util import Vector2D


def _segment(init_pos, init_vel, accel, duration):
    path = MotionPath([MotionPrimitive(accel, duration)])
    return TrajectorySegment(init_pos, init_vel, path)


class TestTrajectoryConstruction:
    def test_empty_trajectory(self):
        traj = Trajectory()
        assert traj.root is None
        assert traj.tail is None

    def test_single_segment_trajectory(self):
        segment = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 2.0)
        traj = Trajectory(segment)

        assert traj.root is segment
        assert traj.tail is segment

    def test_constructor_finds_tail_of_chained_segments(self):
        seg1 = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 2.0)
        dest1 = seg1.get_local_destination()
        seg2 = _segment(dest1.position, dest1.velocity, Vector2D(0, 1), 1.0)
        seg1.add_child(seg2)

        traj = Trajectory(seg1)

        assert traj.root is seg1
        assert traj.tail is seg2


class TestTrajectoryAppend:
    def test_append_to_empty_trajectory_sets_root_and_tail(self):
        traj = Trajectory()
        segment = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 2.0)

        traj.append(segment)

        assert traj.root is segment
        assert traj.tail is segment

    def test_append_chains_and_updates_tail(self):
        seg1 = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 2.0)
        traj = Trajectory(seg1)
        dest1 = seg1.get_local_destination()
        seg2 = _segment(dest1.position, dest1.velocity, Vector2D(0, 1), 1.0)

        traj.append(seg2)

        assert traj.root is seg1
        assert traj.tail is seg2
        assert seg1.child is seg2

    def test_append_raises_on_discontinuous_segment(self):
        seg1 = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 2.0)
        traj = Trajectory(seg1)
        bad_seg = _segment(Vector2D(9999, 9999), Vector2D(9999, 9999), Vector2D(0, 0), 1.0)

        with pytest.raises(Exception):
            traj.append(bad_seg)


class TestTrajectoryDurationsAndState:
    def test_get_total_duration_empty(self):
        assert Trajectory().get_total_duration() == 0.0

    def test_get_total_duration_single_segment(self):
        segment = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 2.0)
        assert Trajectory(segment).get_total_duration() == 2.0

    def test_get_state_empty_trajectory_returns_none(self):
        traj = Trajectory()
        assert traj.get_state(1.0) is None
        assert traj.get_position(1.0) is None
        assert traj.get_velocity(1.0) is None
        assert traj.get_acceleration(1.0) is None
        assert traj.get_destination() is None

    def test_get_position_and_velocity(self):
        segment = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 2.0)
        traj = Trajectory(segment)

        assert traj.get_position(1.0) == Vector2D(0.5, 0.0)
        assert traj.get_velocity(1.0) == Vector2D(1.0, 0.0)

    def test_get_destination_multi_segment(self):
        seg1 = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 2.0)
        dest1 = seg1.get_local_destination()
        seg2 = _segment(dest1.position, dest1.velocity, Vector2D(0, 1), 1.0)
        seg1.add_child(seg2)
        traj = Trajectory(seg1)

        destination = traj.get_destination()

        # Child starts with the parent's ending velocity (2, 0), so during
        # its 1s of (0, 1) acceleration the x position still advances by
        # v_x * t = 2 while y picks up 0.5*1*1^2 = 0.5.
        assert destination.position == Vector2D(4.0, 0.5)
        assert destination.velocity == Vector2D(2.0, 1.0)


class TestTrajectoryConnect:
    def test_connect_truncates_and_appends(self):
        segment = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 4.0)
        traj = Trajectory(segment)

        state_at_2 = traj.get_state(2.0)
        new_segment = _segment(
            state_at_2.position, state_at_2.velocity, Vector2D(0, 1), 1.0
        )

        traj.connect(new_segment, 2.0)

        assert traj.get_total_duration() == 3.0
        assert traj.root.get_local_duration() == 2.0
        assert traj.tail is new_segment

    def test_connect_raises_when_time_negative(self):
        segment = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 4.0)
        traj = Trajectory(segment)
        new_segment = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(0, 1), 1.0)

        with pytest.raises(ValueError):
            traj.connect(new_segment, -1.0)

    def test_connect_raises_when_time_exceeds_total_duration(self):
        segment = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 4.0)
        traj = Trajectory(segment)
        new_segment = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(0, 1), 1.0)

        with pytest.raises(ValueError):
            traj.connect(new_segment, 100.0)

    def test_connect_raises_on_empty_trajectory(self):
        traj = Trajectory()
        new_segment = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(0, 1), 1.0)

        with pytest.raises(ValueError):
            traj.connect(new_segment, 0.0)


class TestTrajectoryRelocate:
    def test_relocate_replaces_root_keeping_children(self):
        seg1 = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 1.0)
        dest1 = seg1.get_local_destination()
        child = _segment(dest1.position, dest1.velocity, Vector2D(0, 1), 1.0)
        traj = Trajectory(seg1)
        traj.append(child)

        new_root = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 1.0)
        traj.relocate(new_root)

        assert traj.root is new_root
        assert traj.get_total_duration() == 2.0

    def test_relocate_without_child_updates_tail_too(self):
        seg1 = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 1.0)
        traj = Trajectory(seg1)

        new_root = _segment(Vector2D(5, 5), Vector2D(0, 0), Vector2D(1, 0), 1.0)
        traj.relocate(new_root)

        assert traj.root is new_root
        assert traj.tail is new_root

    def test_relocate_raises_when_discontinuous_with_existing_child(self):
        seg1 = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 1.0)
        dest1 = seg1.get_local_destination()
        child = _segment(dest1.position, dest1.velocity, Vector2D(0, 1), 1.0)
        traj = Trajectory(seg1)
        traj.append(child)

        discontinuous_root = _segment(
            Vector2D(500, 500), Vector2D(500, 500), Vector2D(2, 0), 1.0
        )

        with pytest.raises(Exception):
            traj.relocate(discontinuous_root)


class TestTrajectoryMsgRoundTrip:
    def test_to_msg_contains_all_segments(self):
        seg1 = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 1.0)
        dest1 = seg1.get_local_destination()
        seg2 = _segment(dest1.position, dest1.velocity, Vector2D(0, 1), 1.0)
        seg1.add_child(seg2)
        traj = Trajectory(seg1)

        msg = traj.to_msg(robot_id=7)

        assert msg.robot_id == 7
        assert len(msg.segments) == 2

    def test_to_msg_empty_trajectory(self):
        msg = Trajectory().to_msg(robot_id=3)
        assert msg.robot_id == 3
        assert msg.segments == []

    def test_from_msg_round_trip(self):
        seg1 = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 1.0)
        dest1 = seg1.get_local_destination()
        seg2 = _segment(dest1.position, dest1.velocity, Vector2D(0, 1), 1.0)
        seg1.add_child(seg2)
        traj = Trajectory(seg1)

        rebuilt = Trajectory.from_msg(traj.to_msg(robot_id=1))

        assert rebuilt.get_total_duration() == traj.get_total_duration()
        assert rebuilt.get_destination().position == traj.get_destination().position


class TestTrajectoryToList:
    def test_requires_exactly_one_of_time_step_or_samples_size(self):
        segment = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 2.0)
        traj = Trajectory(segment)

        with pytest.raises(ValueError):
            traj.to_list()

        with pytest.raises(ValueError):
            traj.to_list(time_step=1.0, samples_size=2)

    def test_empty_trajectory_returns_empty_list(self):
        traj = Trajectory()
        assert traj.to_list(time_step=1.0) == []
        assert traj.to_list(samples_size=5) == []

    def test_to_list_by_time_step_includes_start_and_end(self):
        segment = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 3.0)
        traj = Trajectory(segment)

        positions = traj.to_list(time_step=1.0)

        assert positions[0] == traj.get_position(0.0)
        assert positions[-1] == traj.get_position(3.0)
        assert len(positions) == 4

    def test_to_list_by_samples_size(self):
        segment = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 3.0)
        traj = Trajectory(segment)

        positions = traj.to_list(samples_size=4)

        assert len(positions) == 4
        assert positions[0] == traj.get_position(0.0)
        assert positions[-1] == traj.get_position(3.0)

    def test_to_list_output_states(self):
        segment = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 3.0)
        traj = Trajectory(segment)

        states = traj.to_list(time_step=1.5, output_states=True)

        assert all(hasattr(s, "position") and hasattr(s, "velocity") for s in states)
        assert states[-1].position == traj.get_position(3.0)

    def test_to_list_zero_length_interval_returns_single_item(self):
        segment = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 3.0)
        traj = Trajectory(segment)

        positions = traj.to_list(time_step=1.0, start_time=1.0, end_time=1.0)

        assert positions == [traj.get_position(1.0)]

    def test_to_list_clamps_out_of_range_start_and_end(self):
        segment = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 3.0)
        traj = Trajectory(segment)

        positions = traj.to_list(time_step=1.0, start_time=-10.0, end_time=1000.0)

        assert positions[0] == traj.get_position(0.0)
        assert positions[-1] == traj.get_position(3.0)

    def test_to_list_samples_size_one_uses_full_interval_as_step(self):
        # With samples_size=1 the implementation falls back to using the
        # whole interval as a single time_step (rather than a single
        # sample), so start and end are both included.
        segment = _segment(Vector2D(0, 0), Vector2D(0, 0), Vector2D(1, 0), 3.0)
        traj = Trajectory(segment)

        positions = traj.to_list(samples_size=1)

        assert positions == [traj.get_position(0.0), traj.get_position(3.0)]
