import pytest

from movement.trapezoidal_steering import ControlMerger


@pytest.fixture
def merger():
    return ControlMerger()


class TestDuration:
    def test_duration_sums_segments(self, merger):
        assert merger.duration([[1.0, 2.0], [-1.0, 3.0]]) == pytest.approx(5.0)

    def test_duration_empty(self, merger):
        assert merger.duration([]) == 0.0


class TestVectorWithScalar:
    def test_empty_inputs_return_empty(self, merger):
        assert merger.vector_with_scalar([], [[1.0, 1.0]]) == []
        assert merger.vector_with_scalar([[[1.0], 1.0]], []) == []
        assert merger.vector_with_scalar([], []) == []

    def test_aligned_single_segment(self, merger):
        vector_control = [[[1.0], 2.0]]
        scalar_control = [[2.0, 2.0]]
        result = merger.vector_with_scalar(vector_control, scalar_control)
        assert result == [[[1.0, 2.0], 2.0]]

    def test_aligned_multi_segment_same_boundaries(self, merger):
        vector_control = [[[1.0], 1.0], [[-1.0], 1.0]]
        scalar_control = [[2.0, 1.0], [-2.0, 1.0]]
        result = merger.vector_with_scalar(vector_control, scalar_control)
        assert result == [
            [[1.0, 2.0], 1.0],
            [[-1.0, -2.0], 1.0],
        ]

    def test_misaligned_boundaries_split_segments(self, merger):
        # first control has one segment of duration 2, second has two
        # segments (1 and 1). Boundaries at t=1 must split the first.
        vector_control = [[[1.0], 2.0]]
        scalar_control = [[2.0, 1.0], [3.0, 1.0]]
        result = merger.vector_with_scalar(vector_control, scalar_control)
        assert result == [
            [[1.0, 2.0], 1.0],
            [[1.0, 3.0], 1.0],
        ]
        # total durations line up
        assert merger.duration(result) == pytest.approx(2.0)

    def test_total_duration_is_min_of_inputs_overlap(self, merger):
        vector_control = [[[1.0], 3.0]]
        scalar_control = [[2.0, 5.0]]
        result = merger.vector_with_scalar(vector_control, scalar_control)
        # merging stops once the shorter control is exhausted
        assert merger.duration(result) == pytest.approx(3.0)


class TestScalarAxes:
    def test_empty_controls_returns_empty(self, merger):
        assert merger.scalar_axes([]) == []

    def test_single_axis_wraps_into_vector(self, merger):
        controls = [[[1.0, 2.0], [-1.0, 2.0]]]
        result = merger.scalar_axes(controls)
        assert result == [[[1.0], 2.0], [[-1.0], 2.0]]

    def test_two_axes_aligned(self, merger):
        controls = [
            [[1.0, 1.0], [-1.0, 1.0]],
            [[2.0, 1.0], [-2.0, 1.0]],
        ]
        result = merger.scalar_axes(controls)
        assert result == [
            [[1.0, 2.0], 1.0],
            [[-1.0, -2.0], 1.0],
        ]

    def test_three_axes_aligned(self, merger):
        controls = [
            [[1.0, 1.0]],
            [[2.0, 1.0]],
            [[3.0, 1.0]],
        ]
        result = merger.scalar_axes(controls)
        assert result == [[[1.0, 2.0, 3.0], 1.0]]

    def test_result_duration_matches_each_axis(self, merger):
        controls = [
            [[1.0, 1.0], [0.0, 2.0], [-1.0, 1.0]],
            [[0.5, 4.0]],
        ]
        result = merger.scalar_axes(controls)
        assert merger.duration(result) == pytest.approx(4.0)
