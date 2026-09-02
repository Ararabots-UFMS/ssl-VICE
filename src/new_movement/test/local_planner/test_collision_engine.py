import pytest

from new_movement.local_planner import CollisionEngine
from new_movement.entities.motion import MotionState
from new_movement.entities.obstacle import GenericCircleObstacle, EnemyRobotObstacle

from utils.math_util import Vector2D


class TestCollisionEngine:
    def test_no_collision(self, generator):
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(500, 0), Vector2D(0, 0))
        traj_seg = generator.generate(start, goal)

        obstacles = [GenericCircleObstacle(Vector2D(1000, 1000), 100, padding=0)]
        assert not CollisionEngine.is_collision(traj_seg, obstacles)

    def test_static_collision(self, generator):
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(2000, 0), Vector2D(0, 0))
        traj_seg = generator.generate(start, goal)

        # Obstacle right in the middle
        obstacles = [GenericCircleObstacle(Vector2D(1000, 0), 100, padding=0)]
        assert CollisionEngine.is_collision(traj_seg, obstacles)

    def test_dynamic_collision(self, generator):
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(2000, 0), Vector2D(0, 0))
        traj_seg = generator.generate(start, goal)

        # Enemy robot moving towards the path
        enemy_state = MotionState(Vector2D(1000, 500), Vector2D(0, -1000))
        obstacles = [EnemyRobotObstacle(enemy_state, radius=100)]

        # At some point t, it should collide
        assert CollisionEngine.is_collision(traj_seg, obstacles)

    def test_no_obstacles(self, generator):
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(2000, 0), Vector2D(0, 0))
        traj_seg = generator.generate(start, goal)

        assert not CollisionEngine.is_collision(traj_seg, [])

    def test_zero_duration_trajectory_never_collides(self, generator):
        # Start == goal produces a (near) zero-duration segment; the engine
        # should short-circuit and report no collision even with an
        # obstacle sitting right on top of the point.
        state = MotionState(Vector2D(100, 100), Vector2D(0, 0))
        traj_seg = generator.generate(state, state)

        obstacles = [GenericCircleObstacle(Vector2D(100, 100), 500, padding=0)]
        assert not CollisionEngine.is_collision(traj_seg, obstacles)
