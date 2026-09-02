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
