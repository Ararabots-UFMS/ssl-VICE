import numpy as np
import pytest

from movement.local_planner import InformedSampler

from utils.math_util import Vector2D


class TestInformedSampler:
    def test_constructor_stores_parameters(self):
        s = InformedSampler(field_length=1000, field_width=500, max_velocity=42)
        assert s.field_length == 1000
        assert s.field_width == 500
        assert s.max_velocity == 42

    def test_sample_uniform_stays_on_the_field(self, sampler):
        for _ in range(100):
            p = sampler.sample_uniform()
            assert abs(p.x) <= 6000
            assert abs(p.y) <= 4500

    def test_sample_velocity_respects_the_bound(self, sampler):
        for _ in range(100):
            v = sampler.sample_velocity()
            assert abs(v.x) <= 3000
            assert abs(v.y) <= 3000


class TestSamplerSpread:
    """
    The old sampler drew a 2D gaussian of sigma = distance/4 around the midpoint, so a
    corner-to-corner move scattered via points metres off the line to clear obstacles a
    couple of hundred millimetres wide, and re-rolled the route on every cycle.
    """

    START = Vector2D(-4500, -3000)
    GOAL = Vector2D(4500, 3000)

    def _decompose(self, sampler, spread, count=2000):
        axis = self.GOAL.subtract(self.START)
        unit = axis.multiplyByScalar(1.0 / axis.size())
        perpendicular = unit.perpendicular()

        offsets = []
        for _ in range(count):
            point = sampler.sample_near_axis(self.START, self.GOAL, spread)
            relative = point.subtract(self.START)
            offsets.append((relative.dot(unit), relative.dot(perpendicular)))
        return np.array(offsets)

    def test_offsets_are_perpendicular_to_the_line(self, sampler):
        along, cross = self._decompose(sampler, spread=0.05).T
        length = self.GOAL.subtract(self.START).size()

        # Along the line the via stays in the middle stretch; the scatter is all across.
        assert along.min() >= 0.25 * length - 1
        assert along.max() <= 0.75 * length + 1
        assert cross.std() == pytest.approx(0.05 * length, rel=0.15)

    def test_spread_widens_the_search(self, sampler):
        tight = self._decompose(sampler, spread=0.03)[:, 1].std()
        wide = self._decompose(sampler, spread=0.35)[:, 1].std()

        assert wide > tight * 5

    def test_a_degenerate_line_falls_back_to_the_field(self, sampler):
        """start == goal leaves no axis to offset from, and sigma 0 must not raise."""
        for _ in range(20):
            point = sampler.sample_near_axis(Vector2D(100, 100), Vector2D(100, 100))
            assert abs(point.x) <= 6000
            assert abs(point.y) <= 4500


class TestTangentialVelocity:
    def test_it_points_down_the_path(self, sampler):
        start, via, goal = Vector2D(0, 0), Vector2D(1000, 500), Vector2D(2000, 0)

        for _ in range(100):
            v = sampler.sample_tangential_velocity(start, via, goal)
            assert v.size() <= 3000 + 1e-6
            if v.size() < 1e-6:
                continue
            # Uniform sampling of the velocity square would point half of these back.
            assert v.dot(goal.subtract(start)) > 0

    def test_it_is_zero_when_the_via_doubles_back(self, sampler):
        # via beyond the goal on the same line: the two legs cancel.
        v = sampler.sample_tangential_velocity(
            Vector2D(0, 0), Vector2D(3000, 0), Vector2D(2000, 0)
        )

        assert v.size() == pytest.approx(0.0)
