import pytest

from unittest.mock import MagicMock

from new_movement.local_planner import ObstacleFactory
from new_movement.entities.obstacle import GenericCircleObstacle, EnemyRobotObstacle


def _make_geometry():
    geometry = MagicMock()
    geometry.field_lines = []
    geometry.field_length = 12000
    geometry.field_width = 9000
    return geometry


def _make_config(avoid_penalty_area=True, avoid_ball=True, avoid_center_area=False):
    config = MagicMock()
    config.planning_options.avoid_penalty_area = avoid_penalty_area
    config.planning_options.avoid_ball = avoid_ball
    config.planning_options.avoid_center_area = avoid_center_area
    return config


def _make_ball(x=0, y=0):
    ball = MagicMock()
    ball.position_x = x
    ball.position_y = y
    return ball


def _make_enemy(x=1000, y=1000, vx=0, vy=0):
    enemy = MagicMock()
    enemy.position_x = x
    enemy.position_y = y
    enemy.velocity_x = vx
    enemy.velocity_y = vy
    return enemy


class TestObstacleFactory:
    def test_create_obstacles(self):
        factory = ObstacleFactory()

        obstacles = factory.create_obstacles(
            robot_id=1,
            config=_make_config(),
            geometry=_make_geometry(),
            balls=[_make_ball()],
            enemy_robots=[_make_enemy()],
            ally_robots=[],
        )

        assert len(obstacles) > 0
        # Check if FieldBorder, PenaltyArea, Ball, and Enemy are present (or at least some of them)
        types = [type(o) for o in obstacles]
        assert EnemyRobotObstacle in types
        # GenericCircleObstacle is used for ball
        assert GenericCircleObstacle in types

    def test_no_balls_and_no_enemies_still_returns_field_obstacles(self):
        factory = ObstacleFactory()

        obstacles = factory.create_obstacles(
            robot_id=1,
            config=_make_config(),
            geometry=_make_geometry(),
            balls=[],
            enemy_robots=[],
            ally_robots=[],
        )

        # No ball / enemy obstacles should be generated, but field
        # border + penalty areas (from the mocked geometry) still may be
        # attempted; regardless, the ball/enemy specific entries must be absent.
        types = [type(o) for o in obstacles]
        assert EnemyRobotObstacle not in types

    def test_avoid_ball_false_skips_ball_obstacle(self):
        factory = ObstacleFactory()

        obstacles = factory.create_obstacles(
            robot_id=1,
            config=_make_config(avoid_ball=False),
            geometry=_make_geometry(),
            balls=[_make_ball()],
            enemy_robots=[],
            ally_robots=[],
        )

        types = [type(o) for o in obstacles]
        assert GenericCircleObstacle not in types

    def test_avoid_center_area_true_adds_circle_obstacle(self):
        factory = ObstacleFactory()

        obstacles = factory.create_obstacles(
            robot_id=1,
            config=_make_config(avoid_ball=False, avoid_center_area=True),
            geometry=_make_geometry(),
            balls=[],
            enemy_robots=[],
            ally_robots=[],
        )

        types = [type(o) for o in obstacles]
        assert GenericCircleObstacle in types

    def test_no_geometry_skips_field_and_penalty_obstacles(self):
        factory = ObstacleFactory()

        obstacles = factory.create_obstacles(
            robot_id=1,
            config=_make_config(avoid_ball=False),
            geometry=None,
            balls=[],
            enemy_robots=[],
            ally_robots=[],
        )

        assert obstacles == []


class _Robot:
    """
    Plain stand-in for a Robots message — deliberately not a MagicMock.

    MagicMock implements __index__, so indexing a list with one quietly succeeds and
    hides exactly the bug these tests exist to catch.
    """

    def __init__(self, robot_id, x=1000.0, y=1000.0):
        self.id = robot_id
        self.position_x = x
        self.position_y = y
        self.velocity_x = 0.0
        self.velocity_y = 0.0


class TestRobotCollectionShapes:
    """
    movement_planner and movement_optimizer pass GameState's Robots[] lists;
    movement_path_driver passes {id: Robots} dicts. Both shapes are live in the tree,
    and every other test here passes ally_robots=[], so the mismatch reached the field:
    the ally loop indexed the list with the message it had just iterated, and the enemy
    loop iterated a dict's integer keys and logged every enemy away.
    """

    def _obstacles(self, ally_robots, enemy_robots, robot_id=1):
        return ObstacleFactory().create_obstacles(
            robot_id=robot_id,
            config=_make_config(avoid_penalty_area=False, avoid_ball=False),
            geometry=None,
            balls=[],
            enemy_robots=enemy_robots,
            ally_robots=ally_robots,
        )

    @staticmethod
    def _as_list(*ids):
        return [_Robot(i) for i in ids]

    @staticmethod
    def _as_dict(*ids):
        return {i: _Robot(i) for i in ids}

    @pytest.mark.parametrize("shape", ["_as_list", "_as_dict"])
    def test_allies_become_obstacles_and_exclude_the_planning_robot(self, shape):
        allies = getattr(self, shape)(1, 2, 3)

        obstacles = self._obstacles(allies, [], robot_id=1)

        assert len(obstacles) == 2
        assert all(isinstance(o, EnemyRobotObstacle) for o in obstacles)

    @pytest.mark.parametrize("shape", ["_as_list", "_as_dict"])
    def test_enemies_become_obstacles(self, shape):
        assert len(self._obstacles([], getattr(self, shape)(4, 5))) == 2

    def test_both_collections_together(self):
        obstacles = self._obstacles(self._as_list(1, 2), self._as_list(6, 7))

        assert len(obstacles) == 3  # ally 2, plus both enemies

    def test_empty_collections_are_still_fine(self):
        assert self._obstacles([], []) == []
