import pytest

from movement.entities.motion import MotionPath, MotionPrimitive, MotionState
from movement.entities.trajectory import TrajectorySegment
from movement.trapezoidal_steering import ControlIntegrator
from utils.math_util import Vector2D


def _straight_segment():
    """Segment starting at origin, at rest, accelerating along +x for 2s."""
    path = MotionPath([MotionPrimitive(Vector2D(1, 0), 2.0)])
    return TrajectorySegment(Vector2D(0, 0), Vector2D(0, 0), path)


class TestTrajectorySegmentConstruction:
    def test_stores_initial_state_and_path(self):
        path = MotionPath([MotionPrimitive(Vector2D(1, 0), 2.0)])
        segment = TrajectorySegment(Vector2D(1, 2), Vector2D(3, 4), path)

        assert segment.init_pos == Vector2D(1, 2)
        assert segment.init_vel == Vector2D(3, 4)
        assert segment.motion_path is path
        assert segment.child is None
        assert isinstance(segment.integrator, ControlIntegrator)

    def test_initial_state_property(self):
        segment = TrajectorySegment(
            Vector2D(1, 2), Vector2D(3, 4), MotionPath([])
        )

        state = segment.initial_state

        assert state == MotionState(Vector2D(1, 2), Vector2D(3, 4))


class TestTrajectorySegmentMsgRoundTrip:
    def test_to_msg_populates_fields(self):
        segment = _straight_segment()

        msg = segment.to_msg()

        assert msg.init_pos.x == 0
        assert msg.init_pos.y == 0
        assert msg.init_vel.x == 0
        assert msg.init_vel.y == 0
        assert len(msg.motion_path.primitives) == 1

    def test_from_msg_round_trip(self):
        segment = _straight_segment()

        rebuilt = TrajectorySegment.from_msg(segment.to_msg())

        assert rebuilt.init_pos == segment.init_pos
        assert rebuilt.init_vel == segment.init_vel
        assert len(rebuilt.motion_path.motion_path) == 1
        assert rebuilt.motion_path.motion_path[0].duration == 2.0


class TestTrajectorySegmentDurations:
    def test_local_duration_sums_primitive_durations(self):
        path = MotionPath(
            [MotionPrimitive(Vector2D(1, 0), 2.0), MotionPrimitive(Vector2D(0, 1), 3.0)]
        )
        segment = TrajectorySegment(Vector2D(0, 0), Vector2D(0, 0), path)

        assert segment.get_local_duration() == 5.0

    def test_local_duration_with_empty_path_is_zero(self):
        segment = TrajectorySegment(Vector2D(0, 0), Vector2D(0, 0), MotionPath([]))
        assert segment.get_local_duration() == 0.0

    def test_total_duration_without_child_equals_local_duration(self):
        segment = _straight_segment()
        assert segment.get_total_duration() == segment.get_local_duration() == 2.0

    def test_total_duration_with_child_sums_both(self):
        parent = _straight_segment()
        dest = parent.get_local_destination()
        child = TrajectorySegment(
            dest.position, dest.velocity, MotionPath([MotionPrimitive(Vector2D(0, 1), 1.0)])
        )
        parent.add_child(child)

        assert parent.get_total_duration() == 3.0
        assert parent.get_local_duration() == 2.0


class TestTrajectorySegmentDestinationAndState:
    def test_local_destination_matches_kinematics(self):
        # a=1 for t=2s starting at rest: x = 0.5*1*2^2 = 2, v = 1*2 = 2
        segment = _straight_segment()

        dest = segment.get_local_destination()

        assert dest.position == Vector2D(2.0, 0.0)
        assert dest.velocity == Vector2D(2.0, 0.0)

    def test_get_state_partway_through(self):
        segment = _straight_segment()

        state = segment.get_state(1.0)

        assert state.position == Vector2D(0.5, 0.0)
        assert state.velocity == Vector2D(1.0, 0.0)

    def test_get_state_at_end_matches_destination(self):
        segment = _straight_segment()
        assert segment.get_state(2.0).position == segment.get_local_destination().position

    def test_get_state_beyond_duration_clamps_to_destination(self):
        segment = _straight_segment()

        state = segment.get_state(100.0)

        assert state.position == Vector2D(2.0, 0.0)
        assert state.velocity == Vector2D(2.0, 0.0)

    def test_get_destination_without_child_is_local_destination(self):
        segment = _straight_segment()
        assert segment.get_destination() == segment.get_local_destination()

    def test_get_destination_with_child_recurses_to_last_segment(self):
        parent = _straight_segment()
        dest = parent.get_local_destination()
        child = TrajectorySegment(
            dest.position, dest.velocity, MotionPath([MotionPrimitive(Vector2D(0, 1), 1.0)])
        )
        parent.add_child(child)

        final_dest = parent.get_destination()

        # Child starts with the parent's ending velocity (2, 0), so during
        # its 1s of (0, 1) acceleration the x position still advances by
        # v_x * t = 2 while y picks up 0.5*1*1^2 = 0.5.
        assert final_dest.position == Vector2D(4.0, 0.5)
        assert final_dest.velocity == Vector2D(2.0, 1.0)

    def test_get_state_within_child_segment(self):
        parent = _straight_segment()
        dest = parent.get_local_destination()
        child = TrajectorySegment(
            dest.position, dest.velocity, MotionPath([MotionPrimitive(Vector2D(0, 1), 1.0)])
        )
        parent.add_child(child)

        state = parent.get_state(3.0)

        assert state.position == Vector2D(4.0, 0.5)
        assert state.velocity == Vector2D(2.0, 1.0)


class TestTrajectorySegmentAcceleration:
    def test_get_acceleration_within_local_segment_raises(self):
        """
        Documented bug: get_acceleration delegates to
        `MotionPath.get_acceleration_at_time`, a method that does not exist
        on MotionPath, so any in-range call currently raises AttributeError.
        """
        segment = _straight_segment()

        with pytest.raises(AttributeError):
            segment.get_acceleration(1.0)

    def test_get_acceleration_delegates_to_child_also_raises(self):
        parent = _straight_segment()
        dest = parent.get_local_destination()
        child = TrajectorySegment(
            dest.position, dest.velocity, MotionPath([MotionPrimitive(Vector2D(0, 5), 1.0)])
        )
        parent.add_child(child)

        with pytest.raises(AttributeError):
            parent.get_acceleration(2.5)

    def test_get_acceleration_returns_none_past_end_without_child(self):
        segment = _straight_segment()
        assert segment.get_acceleration(100.0) is None


class TestTrajectorySegmentAddChild:
    def test_add_child_succeeds_when_continuous(self):
        parent = _straight_segment()
        dest = parent.get_local_destination()
        child = TrajectorySegment(dest.position, dest.velocity, MotionPath([]))

        parent.add_child(child)

        assert parent.child is child

    def test_add_child_raises_when_discontinuous(self):
        parent = _straight_segment()
        child = TrajectorySegment(Vector2D(9999, 9999), Vector2D(9999, 9999), MotionPath([]))

        with pytest.raises(Exception):
            parent.add_child(child)

    def test_add_child_raises_when_only_velocity_matches(self):
        parent = _straight_segment()
        dest = parent.get_local_destination()
        child = TrajectorySegment(Vector2D(9999, 9999), dest.velocity, MotionPath([]))

        with pytest.raises(Exception):
            parent.add_child(child)

    def test_add_child_raises_when_only_position_matches(self):
        parent = _straight_segment()
        dest = parent.get_local_destination()
        child = TrajectorySegment(dest.position, Vector2D(9999, 9999), MotionPath([]))

        with pytest.raises(Exception):
            parent.add_child(child)
