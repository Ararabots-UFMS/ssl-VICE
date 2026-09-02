import pytest
import numpy as np
from unittest.mock import MagicMock, patch

from new_movement.local_planner import (
    CollisionEngine,
    InformedSampler,
    TrajectoryOptimizer,
    Orchestrator,
    ObstacleFactory
)
from new_movement.local_planner.solver import BypassSolver, PlanningStatus, SolverConfig

from new_movement.entities.motion import MotionState
from new_movement.entities.trajectory import Trajectory
from new_movement.entities.obstacle import GenericCircleObstacle,EnemyRobotObstacle
from new_movement.local_planner import TrajectoryGenerator

from utils.math_util import Vector2D


@pytest.fixture
def generator():
    return TrajectoryGenerator()

@pytest.fixture
def sampler():
    return InformedSampler(field_length=12000, field_width=9000, max_velocity=3000)

@pytest.fixture
def obstacles():
    # A circle obstacle at (1000, 0) with radius 200
    return [GenericCircleObstacle(Vector2D(1000, 0), 200, padding=0)]

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

class TestBypassSolver:
    def test_solve_blocked_path(self, sampler, generator):
        solver = BypassSolver(max_iterations=100, sampler=sampler, collision_time_step=0.05)
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(2000, 0), Vector2D(0, 0))
        
        # Big obstacle blocking the direct line
        obstacles = [GenericCircleObstacle(Vector2D(1000, 0), 500, padding=0)]
        
        traj = solver.solve(start, goal, obstacles, generator)
        assert traj is not None
        assert traj.root is not None
        # Verify result is collision free
        assert not CollisionEngine.is_collision(traj.root, obstacles)
        assert not CollisionEngine.is_collision(traj.root.child, obstacles)

class TestTrajectoryOptimizer:
    def test_optimize_shortens_path(self, generator):
        optimizer = TrajectoryOptimizer(trys=10, early_stop=5)
        
        # Create a suboptimal path (two segments instead of one)
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        mid = MotionState(Vector2D(500, 500), Vector2D(0, 0))
        goal = MotionState(Vector2D(1000, 0), Vector2D(0, 0))
        
        seg1 = generator.generate(start, mid)
        seg2 = generator.generate(mid, goal)
        seg1.add_child(seg2)
        traj = Trajectory(seg1)
        
        initial_duration = traj.get_total_duration()
        optimized_traj = optimizer.optimize(traj, generator, [])
        
        # It should ideally be shorter or equal
        assert optimized_traj.get_total_duration() <= initial_duration + 1e-6

class TestOrchestrator:
    def test_find_direct_path(self):
        planner = Orchestrator()
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(1000, 0), Vector2D(0, 0))
        
        traj = planner.find(start, goal, [])
        assert planner.status == PlanningStatus.DIRECT_PATH
        assert traj.root is not None

    def test_find_bypass(self):
        planner = Orchestrator(SolverConfig(max_iterations=100))
        start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        goal = MotionState(Vector2D(2000, 0), Vector2D(0, 0))
        obstacles = [GenericCircleObstacle(Vector2D(1000, 0), 300, padding=0)]
        
        traj = planner.find(start, goal, obstacles)
        assert planner.status == PlanningStatus.BYPASS_FOUND
        assert not CollisionEngine.is_collision(traj.root, obstacles)

    def test_handle_static_collisions_start_inside(self):
        planner = Orchestrator()
        obs = GenericCircleObstacle(Vector2D(0, 0), 200, padding=0)
        start = MotionState(Vector2D(50, 50), Vector2D(0, 0)) # Inside obstacle
        goal = MotionState(Vector2D(1000, 0), Vector2D(0, 0))
        
        new_start, new_goal, safety_traj = planner._handle_static_collisions(start, goal, [obs])
        
        # Safety traj should take us out of the obstacle
        assert safety_traj.root is not None
        assert not obs.isCollidingAt(new_start.position)

class TestObstacleFactory:
    def test_create_obstacles(self):
        factory = ObstacleFactory()
        
        # Mock geometry
        geometry = MagicMock()
        geometry.field_lines = []
        geometry.field_length = 12000
        geometry.field_width = 9000
        
        # Mock config
        config = MagicMock()
        config.planning_options.avoid_penalty_area = True
        config.planning_options.avoid_ball = True
        
        # Mock ball
        ball = MagicMock()
        ball.position_x = 0
        ball.position_y = 0
        
        # Mock robots
        enemy = MagicMock()
        enemy.position_x = 1000
        enemy.position_y = 1000
        enemy.velocity_x = 0
        enemy.velocity_y = 0
        
        obstacles = factory.create_obstacles(
            robot_id=1,
            config=config,
            geometry=geometry,
            balls=[ball],
            enemy_robots=[enemy],
            ally_robots=[]
        )
        
        assert len(obstacles) > 0
        # Check if FieldBorder, PenaltyArea, Ball, and Enemy are present (or at least some of them)
        types = [type(o) for o in obstacles]
        assert EnemyRobotObstacle in types
        # GenericCircleObstacle is used for ball
        assert GenericCircleObstacle in types
