import numpy as np
import pytest

from new_movement.entities.obstacle.generic_circle_obstacle import GenericCircleObstacle
from new_movement.entities.obstacle.static_obstacle import StaticObstacle
from utils.math_util import Vector2D


class TestConstructor:
    def test_fields_with_default_padding(self):
        obs = GenericCircleObstacle(Vector2D(1000, 0), 200)
        assert obs.center == Vector2D(1000, 0)
        # default padding is 90.0
        assert obs.radius == pytest.approx(290.0)

    def test_fields_with_explicit_padding(self):
        obs = GenericCircleObstacle(Vector2D(1000, 0), 200, padding=0)
        assert obs.radius == pytest.approx(200.0)

    def test_is_a_static_obstacle(self):
        obs = GenericCircleObstacle(Vector2D(0, 0), 100, padding=0)
        assert isinstance(obs, StaticObstacle)

    def test_negative_center_coordinates(self):
        obs = GenericCircleObstacle(Vector2D(-500, -300), 100, padding=10)
        assert obs.center == Vector2D(-500, -300)
        assert obs.radius == pytest.approx(110.0)


class TestDistanceTo:
    def test_distance_outside_circle(self):
        obs = GenericCircleObstacle(Vector2D(0, 0), 100, padding=0)
        assert obs.distanceTo(Vector2D(200, 0)) == pytest.approx(100.0)

    def test_distance_at_center(self):
        obs = GenericCircleObstacle(Vector2D(0, 0), 100, padding=0)
        assert obs.distanceTo(Vector2D(0, 0)) == pytest.approx(-100.0)

    def test_distance_exactly_on_boundary(self):
        obs = GenericCircleObstacle(Vector2D(0, 0), 100, padding=0)
        assert obs.distanceTo(Vector2D(100, 0)) == pytest.approx(0.0)

    def test_distance_accounts_for_padding(self):
        obs = GenericCircleObstacle(Vector2D(0, 0), 100, padding=50)
        # radius becomes 150
        assert obs.distanceTo(Vector2D(150, 0)) == pytest.approx(0.0)


class TestIsCollidingAt:
    def test_inside_collides(self):
        obs = GenericCircleObstacle(Vector2D(0, 0), 100, padding=0)
        assert obs.isCollidingAt(Vector2D(50, 0)) is True

    def test_outside_does_not_collide(self):
        obs = GenericCircleObstacle(Vector2D(0, 0), 100, padding=0)
        assert obs.isCollidingAt(Vector2D(500, 0)) is False

    def test_exactly_on_boundary_does_not_collide(self):
        obs = GenericCircleObstacle(Vector2D(0, 0), 100, padding=0)
        # distanceTo == 0 is not < 0, so it should not be flagged as colliding
        assert obs.isCollidingAt(Vector2D(100, 0)) is False

    def test_zero_radius_and_padding_only_collides_at_center(self):
        obs = GenericCircleObstacle(Vector2D(0, 0), 0, padding=0)
        assert obs.isCollidingAt(Vector2D(0, 0)) is False  # distance 0, not < 0
        assert obs.isCollidingAt(Vector2D(1, 0)) is False


class TestAdaptDestination:
    def test_returns_same_point_when_outside(self):
        obs = GenericCircleObstacle(Vector2D(0, 0), 100, padding=0)
        target = Vector2D(500, 0)
        assert obs.adaptDestination(target) == target

    def test_projects_point_to_edge_plus_margin(self):
        obs = GenericCircleObstacle(Vector2D(0, 0), 100, padding=0)
        result = obs.adaptDestination(Vector2D(50, 0), margin=30)
        assert result.x == pytest.approx(130.0)
        assert result.y == pytest.approx(0.0)

    def test_default_margin_is_30(self):
        obs = GenericCircleObstacle(Vector2D(0, 0), 100, padding=0)
        result = obs.adaptDestination(Vector2D(0, 50))
        assert result.x == pytest.approx(0.0)
        assert result.y == pytest.approx(130.0)

    def test_target_at_center_uses_arbitrary_direction(self):
        obs = GenericCircleObstacle(Vector2D(0, 0), 100, padding=0)
        result = obs.adaptDestination(Vector2D(0, 0), margin=0)
        # arbitrary direction is (1, 0)
        assert result.x == pytest.approx(100.0)
        assert result.y == pytest.approx(0.0)

    def test_projection_from_non_origin_center(self):
        obs = GenericCircleObstacle(Vector2D(1000, 1000), 100, padding=0)
        result = obs.adaptDestination(Vector2D(1050, 1000), margin=0)
        assert result.x == pytest.approx(1100.0)
        assert result.y == pytest.approx(1000.0)


class TestCheckPositions:
    def test_no_positions_inside(self):
        obs = GenericCircleObstacle(Vector2D(0, 0), 100, padding=0)
        positions = np.array([[500.0, 500.0], [1000.0, 1000.0]])
        assert obs._check_positions(positions) is False

    def test_some_position_inside(self):
        obs = GenericCircleObstacle(Vector2D(0, 0), 100, padding=0)
        positions = np.array([[5000.0, 5000.0], [10.0, 0.0]])
        assert obs._check_positions(positions) is True

    def test_matches_batch_collides(self):
        obs = GenericCircleObstacle(Vector2D(0, 0), 100, padding=0)
        positions = np.array([[10.0, 0.0]])
        times = np.array([0.0])
        assert obs.batch_collides(positions, times) == obs._check_positions(positions)
