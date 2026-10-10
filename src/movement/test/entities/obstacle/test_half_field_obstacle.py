from types import SimpleNamespace

import numpy as np
import pytest

from movement.entities.obstacle.half_field_obstacle import HalfFieldObstacle
from movement.local_planner.obstacle_factory import ObstacleFactory
from utils.math_util import Vector2D


@pytest.mark.parametrize("side", [-1, 1])
def test_halfway_line_and_robot_radius(side):
    obstacle = HalfFieldObstacle(side)
    assert obstacle.isCollidingAt(Vector2D(0, 2000))
    assert obstacle.isCollidingAt(Vector2D(side * 90, 0))
    assert obstacle.isCollidingAt(Vector2D(-side * 3000, 0))
    assert not obstacle.isCollidingAt(Vector2D(side * 150, 0))
    returned = obstacle.adaptDestination(Vector2D(-side * 3000, 800))
    assert not obstacle.isCollidingAt(returned)
    assert returned.y == 800
    starts = np.array([[side * 200, 0]])
    ends = np.array([[-side * 200, 0]])
    assert obstacle._check_segments(starts, ends)
    assert not obstacle._check_segments(starts, starts)


@pytest.mark.parametrize("side", [-1, 0, 1])
def test_obstacle_is_enabled_only_for_selected_role(side):
    config = SimpleNamespace(defensive_half=side, avoid_ball=False,
                             avoid_penalty_area=False, avoid_center_area=False)
    obstacles = ObstacleFactory().create_obstacles(2, config, None, [], [], [])
    assert len(obstacles) == (0 if side == 0 else 1)
    if side:
        assert isinstance(obstacles[0], HalfFieldObstacle)
        assert obstacles[0].side == side
