from types import SimpleNamespace

import numpy as np
import pytest

from new_movement.entities.obstacle.penalty_area_obstacle import PenaltyAreaObstacle
from new_movement.entities.obstacle.static_obstacle import StaticObstacle
from utils.field_util import FieldSide
from utils.math_util import Vector2D


def _line(name, x1, y1, x2, y2):
    return SimpleNamespace(name=name, x1=x1, y1=y1, x2=x2, y2=y2)


def _geometry(field_lines, field_length=0, field_width=0):
    return SimpleNamespace(
        field_lines=field_lines, field_length=field_length, field_width=field_width
    )


@pytest.fixture
def left_geometry():
    return _geometry(
        [
            _line("LeftFieldRightPenaltyStretch", -3000, 1000, -2000, 1000),
            _line("LeftFieldLeftPenaltyStretch", -3000, -1000, -2000, -1000),
        ]
    )


@pytest.fixture
def left_obstacle(left_geometry):
    return PenaltyAreaObstacle(left_geometry, FieldSide.LEFT, padding=100)


@pytest.fixture
def right_geometry():
    return _geometry(
        [
            _line("RightFieldRightPenaltyStretch", 2000, 1000, 3000, 1000),
            _line("RightFieldLeftPenaltyStretch", 2000, -1000, 3000, -1000),
        ]
    )


class TestConstructorLeftSide:
    def test_corners_computed_with_padding(self, left_obstacle):
        assert left_obstacle.top_left_point == Vector2D(-3100, 1100)
        assert left_obstacle.top_right_point == Vector2D(-1900, 1100)
        assert left_obstacle.bot_left_point == Vector2D(-3100, -1100)
        assert left_obstacle.bot_right_point == Vector2D(-1900, -1100)

    def test_padding_stays_positive(self, left_obstacle):
        assert left_obstacle.padding == 100

    def test_not_fallback(self, left_obstacle):
        assert left_obstacle._fallback is False

    def test_is_static_obstacle(self, left_obstacle):
        assert isinstance(left_obstacle, StaticObstacle)


class TestConstructorRightSide:
    def test_padding_is_negated(self, right_geometry):
        obs = PenaltyAreaObstacle(right_geometry, FieldSide.RIGHT, padding=100)
        assert obs.padding == -100

    def test_corners_computed(self, right_geometry):
        obs = PenaltyAreaObstacle(right_geometry, FieldSide.RIGHT, padding=100)
        assert obs.top_left_point == Vector2D(2100, 900)
        assert obs.top_right_point == Vector2D(2900, 900)
        assert obs.bot_left_point == Vector2D(2100, -900)
        assert obs.bot_right_point == Vector2D(2900, -900)


class TestConstructorFallback:
    def test_fallback_uses_field_dimensions_left(self):
        geometry = _geometry([], field_length=12000, field_width=9000)
        obs = PenaltyAreaObstacle(geometry, FieldSide.LEFT, padding=90)

        assert obs._fallback is True
        assert obs.top_left_point == Vector2D(-6090, 1890)
        assert obs.top_right_point == Vector2D(-4314, 1890)
        assert obs.bot_left_point == Vector2D(-6090, -1890)
        assert obs.bot_right_point == Vector2D(-4314, -1890)

    def test_fallback_degenerate_when_no_dimensions(self):
        geometry = _geometry([], field_length=0, field_width=0)
        obs = PenaltyAreaObstacle(geometry, FieldSide.LEFT, padding=90)

        assert obs._fallback is True
        assert obs.top_left_point == Vector2D(-99999.0, 99999.0)
        assert obs.top_right_point == Vector2D(-99989.0, 99999.0)
        assert obs.bot_left_point == Vector2D(-99999.0, 99989.0)
        assert obs.bot_right_point == Vector2D(-99989.0, 99989.0)


class TestIsCollidingAt:
    def test_inside_penalty_area_collides(self, left_obstacle):
        assert left_obstacle.isCollidingAt(Vector2D(-2500, 0)) is True

    def test_outside_penalty_area_does_not_collide(self, left_obstacle):
        assert left_obstacle.isCollidingAt(Vector2D(0, 0)) is False

    def test_on_boundary_does_not_collide(self, left_obstacle):
        # Strict inequality means exactly-on-boundary is not a collision.
        assert left_obstacle.isCollidingAt(Vector2D(-1900, 0)) is False

    def test_negative_y_inside(self, left_obstacle):
        assert left_obstacle.isCollidingAt(Vector2D(-2500, -1000)) is True


class TestDistanceTo:
    def test_distance_negative_when_inside(self, left_obstacle):
        distance = left_obstacle.distanceTo(Vector2D(-2500, 0))
        assert distance == pytest.approx(-600.0)

    def test_distance_positive_when_outside(self, left_obstacle):
        distance = left_obstacle.distanceTo(Vector2D(0, 0))
        assert distance == pytest.approx(1900.0)


class TestAdaptDestination:
    def test_returns_same_point_when_outside(self, left_obstacle):
        target = Vector2D(0, 0)
        assert left_obstacle.adaptDestination(target) == target

    def test_moves_along_y_when_dx_greater_than_dy(self, left_obstacle):
        result = left_obstacle.adaptDestination(Vector2D(-2500, 1050))
        assert result.x == pytest.approx(-2500.0)
        assert result.y == pytest.approx(1100.0)

    def test_moves_along_x_when_dx_lte_dy(self, left_obstacle):
        result = left_obstacle.adaptDestination(Vector2D(-1950, 0))
        assert result.x == pytest.approx(-1900.0)
        assert result.y == pytest.approx(0.0)


class TestFindClosestCorner:
    def test_closest_corner(self, left_obstacle):
        closest = left_obstacle._findClosestCorner(Vector2D(-1950, 0))
        assert closest == Vector2D(-1900, 1100)


class TestCheckPositions:
    def test_no_position_inside(self, left_obstacle):
        positions = np.array([[0.0, 0.0], [500.0, 500.0]])
        assert left_obstacle._check_positions(positions) is False

    def test_some_position_inside(self, left_obstacle):
        positions = np.array([[0.0, 0.0], [-2500.0, 0.0]])
        assert left_obstacle._check_positions(positions) is True

    def test_batch_collides_matches_check_positions(self, left_obstacle):
        positions = np.array([[-2500.0, 0.0]])
        times = np.array([0.0])
        assert left_obstacle.batch_collides(
            positions, times
        ) == left_obstacle._check_positions(positions)
