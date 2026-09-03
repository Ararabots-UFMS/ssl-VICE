from types import SimpleNamespace

import numpy as np
import pytest

from new_movement.entities.obstacle.field_border_obstacle import FieldBorderObstacle
from new_movement.entities.obstacle.static_obstacle import StaticObstacle
from utils.math_util import Vector2D


def _line(name, x1, y1, x2, y2):
    return SimpleNamespace(name=name, x1=x1, y1=y1, x2=x2, y2=y2)


def _geometry(field_lines, field_length=0, field_width=0):
    return SimpleNamespace(
        field_lines=field_lines, field_length=field_length, field_width=field_width
    )


@pytest.fixture
def rect_geometry():
    """A field described directly by TopTouchLine/BottomTouchLine."""
    return _geometry(
        [
            _line("TopTouchLine", -2000, 1000, 2000, 1000),
            _line("BottomTouchLine", -2000, -1000, 2000, -1000),
        ]
    )


@pytest.fixture
def obstacle(rect_geometry):
    return FieldBorderObstacle(rect_geometry, padding=100)


class TestConstructorFromLines:
    def test_corners_computed_with_padding(self, obstacle):
        assert obstacle.top_left_point == Vector2D(-1900, 900)
        assert obstacle.top_right_point == Vector2D(1900, 900)
        assert obstacle.bot_left_point == Vector2D(-1900, -900)
        assert obstacle.bot_right_point == Vector2D(1900, -900)

    def test_not_using_fallback(self, obstacle):
        assert obstacle._fallback is False

    def test_is_static_obstacle(self, obstacle):
        assert isinstance(obstacle, StaticObstacle)


class TestConstructorFallback:
    def test_fallback_uses_field_dimensions(self):
        geometry = _geometry([], field_length=12000, field_width=9000)
        obs = FieldBorderObstacle(geometry, padding=90)

        assert obs._fallback is True
        assert obs.top_left_point == Vector2D(-5910, 4410)
        assert obs.top_right_point == Vector2D(5910, 4410)
        assert obs.bot_left_point == Vector2D(-5910, -4410)
        assert obs.bot_right_point == Vector2D(5910, -4410)

    def test_fallback_default_extent_when_no_dimensions(self):
        geometry = _geometry([], field_length=0, field_width=0)
        obs = FieldBorderObstacle(geometry, padding=90)

        assert obs._fallback is True
        assert obs.top_left_point == Vector2D(-1000.0, 1000.0)
        assert obs.top_right_point == Vector2D(1000.0, 1000.0)
        assert obs.bot_left_point == Vector2D(-1000.0, -1000.0)
        assert obs.bot_right_point == Vector2D(1000.0, -1000.0)

    def test_partial_lines_still_triggers_fallback(self):
        # Only the top line is present, bottom line is missing.
        geometry = _geometry(
            [_line("TopTouchLine", -2000, 1000, 2000, 1000)],
            field_length=12000,
            field_width=9000,
        )
        obs = FieldBorderObstacle(geometry, padding=90)
        assert obs._fallback is True
        # The bottom corners come from the fallback rectangle.
        assert obs.bot_left_point == Vector2D(-5910, -4410)


class TestIsCollidingAt:
    def test_inside_safe_area_does_not_collide(self, obstacle):
        assert obstacle.isCollidingAt(Vector2D(0, 0)) is False

    def test_outside_field_collides(self, obstacle):
        assert obstacle.isCollidingAt(Vector2D(3000, 0)) is True

    def test_on_border_line_collides(self, obstacle):
        # Exactly on the boundary (not strictly inside) counts as colliding.
        assert obstacle.isCollidingAt(Vector2D(1900, 0)) is True

    def test_negative_coordinates_inside(self, obstacle):
        assert obstacle.isCollidingAt(Vector2D(-1000, -500)) is False

    def test_negative_coordinates_outside(self, obstacle):
        assert obstacle.isCollidingAt(Vector2D(-3000, -500)) is True


class TestDistanceTo:
    def test_distance_when_inside(self, obstacle):
        # Closest corner to origin is the first corner in iteration order
        # (top_left) since all four corners are equidistant from (0, 0).
        assert obstacle.distanceTo(Vector2D(0, 0)) == pytest.approx(900.0)

    def test_distance_is_negative_when_colliding(self, obstacle):
        distance = obstacle.distanceTo(Vector2D(3000, 900))
        assert distance == pytest.approx(-1100.0)

    def test_distance_negative_zero_at_exact_edge_reference(self, obstacle):
        # Point directly at the top_right y value but beyond x: only dx contributes.
        distance = obstacle.distanceTo(Vector2D(1900, 900))
        assert distance <= 0


class TestAdaptDestination:
    def test_pulls_far_point_toward_closest_corner_x(self, obstacle):
        result = obstacle.adaptDestination(Vector2D(3000, 0), margin=50)
        assert result.x == pytest.approx(1850.0)
        assert result.y == pytest.approx(0.0)

    def test_pulls_far_point_toward_closest_corner_y(self, obstacle):
        result = obstacle.adaptDestination(Vector2D(0, 2000), margin=50)
        assert result.x == pytest.approx(0.0)
        assert result.y == pytest.approx(850.0)

    def test_default_margin(self, obstacle):
        result = obstacle.adaptDestination(Vector2D(3000, 0))
        assert result.x == pytest.approx(1850.0)

    def test_point_already_inside_moves_only_offending_axis(self, obstacle):
        result = obstacle.adaptDestination(Vector2D(500, 500), margin=50)
        # Neither axis exceeds its corner in absolute value, so unchanged.
        assert result.x == pytest.approx(500.0)
        assert result.y == pytest.approx(500.0)


class TestFindClosestCorner:
    def test_closest_corner_matches_nearest_point(self, obstacle):
        closest = obstacle._findClosestCorner(Vector2D(1800, 800))
        assert closest == Vector2D(1900, 900)


class TestCheckPositions:
    def test_all_inside_returns_false(self, obstacle):
        positions = np.array([[0.0, 0.0], [100.0, 100.0]])
        assert obstacle._check_positions(positions) is False

    def test_any_outside_returns_true(self, obstacle):
        positions = np.array([[0.0, 0.0], [5000.0, 5000.0]])
        assert obstacle._check_positions(positions) is True

    def test_batch_collides_matches_check_positions(self, obstacle):
        positions = np.array([[5000.0, 5000.0]])
        times = np.array([0.0])
        assert obstacle.batch_collides(positions, times) == obstacle._check_positions(
            positions
        )
