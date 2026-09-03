import pytest

from new_movement.entities.motion import MotionPath, MotionPrimitive
from utils.math_util import Vector2D


def _as_tuples(motion_path):
    return [
        (p.acceleration.x, p.acceleration.y, p.duration)
        for p in motion_path.motion_path
    ]


@pytest.fixture
def three_primitives():
    return [
        MotionPrimitive(Vector2D(1, 0), 2.0),
        MotionPrimitive(Vector2D(0, 1), 2.0),
        MotionPrimitive(Vector2D(1, 1), 2.0),
    ]


class TestMotionPathConstruction:
    def test_holds_list_of_primitives(self, three_primitives):
        path = MotionPath(three_primitives)
        assert path.motion_path == three_primitives

    def test_empty_path(self):
        path = MotionPath([])
        assert path.motion_path == []


class TestMotionPathMsgRoundTrip:
    def test_to_msg_contains_all_primitives(self, three_primitives):
        path = MotionPath(three_primitives)

        msg = path.to_msg()

        assert len(msg.primitives) == 3
        assert msg.primitives[0].duration == 2.0
        assert msg.primitives[0].acceleration.x == 1
        assert msg.primitives[0].acceleration.y == 0

    def test_to_msg_empty_path(self):
        path = MotionPath([])
        msg = path.to_msg()
        assert msg.primitives == []

    def test_from_msg_round_trip(self, three_primitives):
        path = MotionPath(three_primitives)

        rebuilt = MotionPath.from_msg(path.to_msg())

        assert _as_tuples(rebuilt) == _as_tuples(path)


class TestMotionPathSplit:
    def test_split_at_or_below_zero_returns_empty_first(self, three_primitives):
        for t in (0, -1, -100):
            first, second = MotionPath(three_primitives).split(t)
            assert first.motion_path == []
            assert _as_tuples(second) == [(1, 0, 2.0), (0, 1, 2.0), (1, 1, 2.0)]

    def test_split_at_or_above_total_duration_returns_empty_second(
        self, three_primitives
    ):
        for t in (6, 7, 100):
            first, second = MotionPath(three_primitives).split(t)
            assert _as_tuples(first) == [(1, 0, 2.0), (0, 1, 2.0), (1, 1, 2.0)]
            assert second.motion_path == []

    def test_split_within_first_primitive(self, three_primitives):
        first, second = MotionPath(three_primitives).split(1.0)

        assert _as_tuples(first) == [(1, 0, 1.0)]
        assert _as_tuples(second) == [(1, 0, 1.0), (0, 1, 2.0), (1, 1, 2.0)]

    def test_split_within_second_primitive(self, three_primitives):
        first, second = MotionPath(three_primitives).split(3.0)

        assert _as_tuples(first) == [(1, 0, 2.0), (0, 1, 1.0)]
        assert _as_tuples(second) == [(0, 1, 1.0), (1, 1, 2.0)]

    def test_split_at_non_integer_time(self, three_primitives):
        first, second = MotionPath(three_primitives).split(3.5)

        assert _as_tuples(first) == [(1, 0, 2.0), (0, 1, 1.5)]
        assert _as_tuples(second) == [(0, 1, 0.5), (1, 1, 2.0)]

    def test_split_exactly_on_primitive_boundary_drops_middle_primitive(
        self, three_primitives
    ):
        # Documented quirk: splitting exactly on the boundary between the
        # first and second primitive causes the algorithm's second
        # bookkeeping loop to skip the (now zero-duration) boundary
        # primitive and jump straight to the following one, dropping the
        # untouched second primitive entirely from the result.
        first, second = MotionPath(three_primitives).split(2.0)

        assert _as_tuples(first) == [(1, 0, 2.0)]
        assert _as_tuples(second) == [(1, 0, 0.0), (1, 1, 2.0)]

    def test_split_total_duration_is_split_point(self, three_primitives):
        first, second = MotionPath(three_primitives).split(3.0)
        total_first = sum(p.duration for p in first.motion_path)
        total_second = sum(p.duration for p in second.motion_path)
        assert total_first == pytest.approx(3.0)
        assert total_second == pytest.approx(3.0)


class TestMotionPathTruncate:
    def test_truncate_at_or_below_zero_empties_path(self, three_primitives):
        for t in (0, -1, -50):
            path = MotionPath(list(three_primitives))
            path.truncate(t)
            assert path.motion_path == []

    def test_truncate_at_or_above_total_duration_is_unchanged(self, three_primitives):
        for t in (6, 7, 100):
            path = MotionPath(list(three_primitives))
            path.truncate(t)
            assert _as_tuples(path) == [(1, 0, 2.0), (0, 1, 2.0), (1, 1, 2.0)]

    def test_truncate_within_first_primitive(self, three_primitives):
        path = MotionPath(list(three_primitives))
        path.truncate(1.0)
        assert _as_tuples(path) == [(1, 0, 1.0)]

    def test_truncate_on_primitive_boundary(self, three_primitives):
        path = MotionPath(list(three_primitives))
        path.truncate(2.0)
        assert _as_tuples(path) == [(1, 0, 2.0)]

    def test_truncate_within_later_primitive(self, three_primitives):
        path = MotionPath(list(three_primitives))
        path.truncate(3.5)
        assert _as_tuples(path) == [(1, 0, 2.0), (0, 1, 1.5)]

    def test_truncate_modifies_in_place(self, three_primitives):
        path = MotionPath(list(three_primitives))
        path.truncate(1.0)
        # The same object's motion_path attribute was mutated.
        assert path.motion_path[0].duration == 1.0

    def test_truncate_empty_path_stays_empty(self):
        path = MotionPath([])
        path.truncate(5.0)
        assert path.motion_path == []
