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
    hides exactly the bug these tests exist to catch. A real ROS message has no
    __index__, which is why the field saw "list indices must be integers or slices,
    not Robots" while the suite stayed green.
    """

    def __init__(self, robot_id, x=1000.0, y=1000.0, vx=0.0, vy=0.0):
        self.id = robot_id
        self.position_x = x
        self.position_y = y
        self.velocity_x = vx
        self.velocity_y = vy


def _make_robot(robot_id, x=1000.0, y=1000.0, vx=0.0, vy=0.0):
    return _Robot(robot_id, x, y, vx, vy)


class TestRobotCollectionShapes:
    """
    movement_planner and movement_optimizer pass GameState's Robots[] lists;
    movement_path_driver passes {id: Robots} dicts. Both shapes are live in the tree.

    Every other test in this file passes ally_robots=[], so the ally loop never ran and
    the mismatch reached the field instead: publishing a target made the planner log
    "list indices must be integers or slices, not Robots" on every cycle, because the
    loop indexed the list with the message it had just iterated.
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

    def test_allies_as_a_list_are_obstacles(self):
        obstacles = self._obstacles([_make_robot(2), _make_robot(3)], [])

        assert len(obstacles) == 2
        assert all(isinstance(o, EnemyRobotObstacle) for o in obstacles)

    def test_allies_as_a_dict_are_obstacles(self):
        allies = {2: _make_robot(2), 3: _make_robot(3)}

        assert len(self._obstacles(allies, [])) == 2

    def test_the_planning_robot_is_not_its_own_obstacle(self):
        obstacles = self._obstacles([_make_robot(1), _make_robot(2)], [], robot_id=1)

        assert len(obstacles) == 1

    def test_self_exclusion_works_for_the_dict_shape_too(self):
        allies = {1: _make_robot(1), 2: _make_robot(2)}

        assert len(self._obstacles(allies, [], robot_id=1)) == 1

    def test_enemies_as_a_list_are_obstacles(self):
        assert len(self._obstacles([], [_make_robot(4), _make_robot(5)])) == 2

    def test_enemies_as_a_dict_are_obstacles(self):
        """A dict used to iterate its integer keys, so every enemy was logged away."""
        enemies = {4: _make_robot(4), 5: _make_robot(5)}

        assert len(self._obstacles([], enemies)) == 2

    def test_both_collections_together(self):
        obstacles = self._obstacles(
            [_make_robot(1), _make_robot(2)], [_make_robot(6), _make_robot(7)]
        )

        assert len(obstacles) == 3  # ally 2, plus both enemies

    def test_empty_collections_are_still_fine(self):
        assert self._obstacles([], []) == []
