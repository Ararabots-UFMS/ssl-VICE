from new_movement.local_planner import InformedSampler

from utils.math_util import Vector2D


class TestInformedSampler:
    def test_sample_uniform(self, sampler):
        for _ in range(100):
            p = sampler.sample_uniform()
            assert abs(p.x) <= 6000
            assert abs(p.y) <= 4500

    def test_sample_velocity(self, sampler):
        for _ in range(100):
            v = sampler.sample_velocity()
            assert abs(v.x) <= 3000
            assert abs(v.y) <= 3000

    def test_sample_near_axis(self, sampler):
        start = Vector2D(0, 0)
        goal = Vector2D(2000, 0)
        for _ in range(100):
            p = sampler.sample_near_axis(start, goal)
            assert isinstance(p, Vector2D)

    def test_constructor_stores_parameters(self):
        s = InformedSampler(field_length=1000, field_width=500, max_velocity=42)
        assert s.field_length == 1000
        assert s.field_width == 500
        assert s.max_velocity == 42

    def test_sample_near_axis_same_point_still_returns_vector(self, sampler):
        # start == goal: distance is 0, so np.random.normal is sampled with
        # scale 0 -- should not raise and should still return a Vector2D.
        point = Vector2D(0, 0)
        for _ in range(20):
            p = sampler.sample_near_axis(point, point)
            assert isinstance(p, Vector2D)
