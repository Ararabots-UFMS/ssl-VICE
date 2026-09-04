import numpy as np
import pytest

from new_movement.entities.obstacle.enemy_robot_obstacle import EnemyRobotObstacle
from new_movement.entities.obstacle.obstacle import Obstacle
from new_movement.entities.motion.motion_state import MotionState
from utils.math_util import Vector2D


@pytest.fixture
def robot_state():
    return MotionState(Vector2D(0, 0), Vector2D(100, 0))


@pytest.fixture
def obstacle(robot_state):
    return EnemyRobotObstacle(robot_state, radius=50, max_lookahead=1.0)


class TestConstructor:
    def test_default_fields(self, robot_state):
        obs = EnemyRobotObstacle(robot_state)
        assert obs.robotState is robot_state
        assert obs.radius == 90
        assert obs.max_lookahead == 0.5

    def test_explicit_fields(self, obstacle, robot_state):
        assert obstacle.robotState is robot_state
        assert obstacle.radius == 50
        assert obstacle.max_lookahead == 1.0

    def test_is_obstacle(self, obstacle):
        assert isinstance(obstacle, Obstacle)


class TestGetPosHelpers:
    def test_get_pos_returns_robot_position(self, obstacle, robot_state):
        assert obstacle._getPos() == robot_state.position

    def test_get_predicted_pos(self, obstacle):
        # position (0,0) + velocity (100,0) * t
        predicted = obstacle._getPredictedPos(0.5)
        assert predicted == Vector2D(50, 0)

    def test_get_predicted_pos_zero_time(self, obstacle):
        assert obstacle._getPredictedPos(0) == Vector2D(0, 0)


class TestClosestPointInLine:
    def test_point_projects_within_segment(self, obstacle):
        start = Vector2D(0, 0)
        end = Vector2D(50, 0)
        closest = obstacle._closestPointInLine(Vector2D(25, 100), start, end)
        assert closest == Vector2D(25, 0)

    def test_point_clamped_to_start(self, obstacle):
        start = Vector2D(0, 0)
        end = Vector2D(50, 0)
        closest = obstacle._closestPointInLine(Vector2D(-100, 5), start, end)
        assert closest == start

    def test_point_clamped_to_end(self, obstacle):
        start = Vector2D(0, 0)
        end = Vector2D(50, 0)
        closest = obstacle._closestPointInLine(Vector2D(1000, 5), start, end)
        assert closest == end

    def test_degenerate_segment_returns_start(self, obstacle):
        start = Vector2D(10, 10)
        end = Vector2D(10, 10)
        closest = obstacle._closestPointInLine(Vector2D(50, 50), start, end)
        assert closest == start


class TestDistanceTo:
    def test_distance_perpendicular_to_tube(self, obstacle):
        # tube from (0,0) to (50,0) at t=0.5; point (25,100) projects to (25,0)
        distance = obstacle.distanceTo(Vector2D(25, 100), t=0.5)
        assert distance == pytest.approx(50.0)  # 100 - radius(50)

    def test_distance_negative_inside_tube(self, obstacle):
        distance = obstacle.distanceTo(Vector2D(25, 10), t=0.5)
        assert distance == pytest.approx(-40.0)  # 10 - 50

    def test_distance_clamps_to_max_lookahead(self, obstacle):
        # t way beyond max_lookahead should behave the same as t == max_lookahead
        distance_far = obstacle.distanceTo(Vector2D(150, 0), t=100.0)
        distance_clamped = obstacle.distanceTo(Vector2D(150, 0), t=1.0)
        assert distance_far == pytest.approx(distance_clamped)


class TestIsCollidingAt:
    def test_inside_tube_collides(self, obstacle):
        assert obstacle.isCollidingAt(Vector2D(25, 10), t=0.5) is True

    def test_outside_tube_does_not_collide(self, obstacle):
        assert obstacle.isCollidingAt(Vector2D(25, 200), t=0.5) is False

    def test_exactly_on_boundary_collides(self, obstacle):
        # Unlike other obstacles, EnemyRobotObstacle treats distance == 0 as a collision.
        assert obstacle.isCollidingAt(Vector2D(25, 50), t=0.5) is True


class TestBatchCollides:
    def test_matches_isCollidingAt_when_colliding(self, obstacle):
        positions = np.array([[25.0, 10.0]])
        times = np.array([0.5])
        assert obstacle.batch_collides(positions, times) is True

    def test_no_collision(self, obstacle):
        positions = np.array([[25.0, 200.0], [-500.0, -500.0]])
        times = np.array([0.5, 0.5])
        assert obstacle.batch_collides(positions, times) is False

    def test_respects_max_lookahead_clamp(self, obstacle):
        # times far beyond max_lookahead should clamp identically to max_lookahead
        positions = np.array([[100.0, 0.0]])
        times = np.array([1000.0])
        # At clamped t=1.0, predicted end is (100, 0), point lies on tube end -> collides
        assert obstacle.batch_collides(positions, times) is True

    def test_empty_positions(self, obstacle):
        positions = np.zeros((0, 2))
        times = np.zeros((0,))
        assert obstacle.batch_collides(positions, times) is False


class TestAdaptDestination:
    def test_target_on_line_uses_perpendicular_push(self, obstacle):
        # Target exactly on the tube centerline -> dir_vector size is 0.
        result = obstacle.adaptDestination(Vector2D(25, 0), t=0.5)
        assert result.x == pytest.approx(25.0)
        assert result.y == pytest.approx(-50.0)

    def test_target_inside_tube_pushed_to_boundary(self, obstacle):
        result = obstacle.adaptDestination(Vector2D(25, 10), t=0.5)
        assert result.x == pytest.approx(25.0)
        assert result.y == pytest.approx(50.0)

    def test_target_already_outside_radius_unchanged(self, obstacle):
        target = Vector2D(25, 100)
        result = obstacle.adaptDestination(target, t=0.5)
        assert result == target


class TestUpdateState:
    def test_update_state(self, obstacle):
        new_state = MotionState(Vector2D(5, 5), Vector2D(1, 1))
        obstacle.updateState(new_state)
        assert obstacle.robotState is new_state


class TestVelocity:
    def test_velocity_returns_robot_state_velocity(self):
        state = MotionState(Vector2D(0, 0), Vector2D(42, -7))
        obs = EnemyRobotObstacle(state)
        assert obs.velocity() == Vector2D(42, -7)


class TestSweptSegments:
    def test_a_tube_clipped_between_samples_is_caught(self):
        enemy = EnemyRobotObstacle(
            MotionState(Vector2D(500.0, -300.0), Vector2D(0.0, 600.0)), radius=90.0
        )

        assert enemy.batch_collides_segments(
            np.array([[0.0, 0.0]]), np.array([[1000.0, 0.0]]),
            np.array([0.0]), np.array([1.0]),
        )

    def test_an_enemy_well_clear_is_not_flagged(self):
        enemy = EnemyRobotObstacle(
            MotionState(Vector2D(500.0, 3000.0), Vector2D(0.0, 10.0)), radius=90.0
        )

        assert not enemy.batch_collides_segments(
            np.array([[0.0, 0.0]]), np.array([[1000.0, 0.0]]),
            np.array([0.0]), np.array([1.0]),
        )


class TestBounds:
    def test_the_tube_box_holds_every_colliding_point(self):
        import random

        rng = random.Random(3)
        for _ in range(200):
            state = MotionState(
                Vector2D(rng.uniform(-3000, 3000), rng.uniform(-2000, 2000)),
                Vector2D(rng.uniform(-3000, 3000), rng.uniform(-3000, 3000)),
            )
            obstacle = EnemyRobotObstacle(state, radius=200)
            min_x, min_y, max_x, max_y = obstacle.bounds()

            for _ in range(40):
                point = Vector2D(rng.uniform(-6000, 6000), rng.uniform(-4500, 4500))
                if obstacle.isCollidingAt(point, rng.uniform(0.0, 2.0)):
                    assert min_x <= point.x <= max_x
                    assert min_y <= point.y <= max_y
