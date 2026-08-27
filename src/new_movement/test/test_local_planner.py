import random

import pytest
import numpy as np
from unittest.mock import MagicMock, patch

from new_movement.local_planner.collision import CollisionEngine
from new_movement.local_planner.sampler import InformedSampler
from new_movement.local_planner.solvers import BypassSolver, PlanningStatus
from new_movement.local_planner.optimizer import TrajectoryOptimizer
from new_movement.local_planner.planner import Planner, SolverConfig
from new_movement.local_planner.factory import ObstacleFactory

from new_movement.entities.States import State, Vector2D, MoveConstraints
from new_movement.entities.Trajectory import Trajectory, TrajectorySegment
from new_movement.entities.StaticObstacle import GenericCircleObstacle, StaticObstacle
from new_movement.entities.DynamicObstacles import EnemyRobotObstacle
from new_movement.utilities.trajectory_generator.TrajGenerator import TrajectoryGenerator

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

class TestTrajectoryGeneratorRegression:
    """
    Guards the constraint signs. With a negative bound on one axis the trapezoidal
    steer brakes the wrong way and stops enforcing the velocity cap, which used to
    leave ~65% of the generated trajectories short of their goal without ever
    raising. These tests are cheap enough to run on every commit.
    """

    FIELD_HALF_LENGTH = 6000.0
    FIELD_HALF_WIDTH = 4500.0

    @pytest.fixture
    def planner_generator(self):
        config = SolverConfig()
        return TrajectoryGenerator(
            MoveConstraints(config.max_velocity, config.max_acceleration)
        ), config

    def _random_state(self, rng, max_velocity):
        return State(
            Vector2D(
                rng.uniform(-self.FIELD_HALF_LENGTH, self.FIELD_HALF_LENGTH),
                rng.uniform(-self.FIELD_HALF_WIDTH, self.FIELD_HALF_WIDTH),
            ),
            Vector2D(
                rng.uniform(-max_velocity.x, max_velocity.x),
                rng.uniform(-max_velocity.y, max_velocity.y),
            ),
        )

    def test_generated_trajectories_reach_the_goal(self, planner_generator):
        generator, config = planner_generator
        rng = random.Random(0)

        for _ in range(2000):
            start = self._random_state(rng, config.max_velocity)
            goal = State(
                self._random_state(rng, config.max_velocity).position, Vector2D(0.0, 0.0)
            )

            destination = generator.generate(start, goal).get_local_destination()

            assert destination.position.distance(goal.position) < 1.0, (
                f"start={start} goal={goal} reached={destination}"
            )
            assert destination.velocity.distance(goal.velocity) < 1.0, (
                f"start={start} goal={goal} reached={destination}"
            )

    def test_velocity_limit_is_enforced(self, planner_generator):
        generator, config = planner_generator
        start = State(Vector2D(0.0, 0.0), Vector2D(0.0, 0.0))
        goal = State(Vector2D(4000.0, 4000.0), Vector2D(0.0, 0.0))

        segment = generator.generate(start, goal)

        duration = segment.get_total_duration()
        for t in np.arange(0.0, duration, 0.01):
            velocity = segment.get_state(float(t)).velocity
            assert abs(velocity.x) <= config.max_velocity.x + 1.0
            assert abs(velocity.y) <= config.max_velocity.y + 1.0


class TestSolverConfig:
    def test_rejects_negative_velocity_bound(self):
        with pytest.raises(ValueError):
            SolverConfig(max_velocity=Vector2D(2000.0, -2000.0))

    def test_rejects_negative_acceleration_bound(self):
        with pytest.raises(ValueError):
            SolverConfig(max_acceleration=Vector2D(1500.0, -1500.0))

    def test_defaults_are_independent_between_instances(self):
        first, second = SolverConfig(), SolverConfig()
        assert first.max_velocity is not second.max_velocity


class TestTrajectoryContinuity:
    def test_add_child_rejects_a_position_gap(self, generator):
        start = State(Vector2D(0, 0), Vector2D(0, 0))
        goal = State(Vector2D(1000, 0), Vector2D(0, 0))
        segment = generator.generate(start, goal)

        # Same (zero) velocity as the parent's endpoint, but three metres away.
        detached = generator.generate(
            State(Vector2D(4000, 0), Vector2D(0, 0)),
            State(Vector2D(5000, 0), Vector2D(0, 0)),
        )

        with pytest.raises(Exception):
            segment.add_child(detached)


class TestCollisionEngine:
    def test_no_collision(self, generator):
        start = State(Vector2D(0, 0), Vector2D(0, 0))
        goal = State(Vector2D(500, 0), Vector2D(0, 0))
        traj_seg = generator.generate(start, goal)
        
        obstacles = [GenericCircleObstacle(Vector2D(1000, 1000), 100, padding=0)]
        assert not CollisionEngine.is_collision(traj_seg, obstacles)

    def test_static_collision(self, generator):
        start = State(Vector2D(0, 0), Vector2D(0, 0))
        goal = State(Vector2D(2000, 0), Vector2D(0, 0))
        traj_seg = generator.generate(start, goal)
        
        # Obstacle right in the middle
        obstacles = [GenericCircleObstacle(Vector2D(1000, 0), 100, padding=0)]
        assert CollisionEngine.is_collision(traj_seg, obstacles)

    def test_dynamic_collision(self, generator):
        start = State(Vector2D(0, 0), Vector2D(0, 0))
        goal = State(Vector2D(2000, 0), Vector2D(0, 0))
        traj_seg = generator.generate(start, goal)
        
        # Enemy robot moving towards the path
        enemy_state = State(Vector2D(1000, 500), Vector2D(0, -1000))
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

    def test_tangential_velocity_points_down_the_path(self, sampler):
        start, via, goal = Vector2D(0, 0), Vector2D(1000, 500), Vector2D(2000, 0)

        for _ in range(100):
            v = sampler.sample_tangential_velocity(start, via, goal)
            assert v.size() <= 3000 + 1e-6
            if v.size() < 1e-6:
                continue
            # Uniform sampling of the velocity square would point half of these back.
            assert v.dot(goal.subtract(start)) > 0

    def test_tangential_velocity_is_zero_when_the_via_doubles_back(self, sampler):
        # via beyond the goal on the same line: the two legs cancel.
        v = sampler.sample_tangential_velocity(
            Vector2D(0, 0), Vector2D(3000, 0), Vector2D(2000, 0)
        )

        assert v.size() == pytest.approx(0.0)

class TestBypassSolver:
    def test_solve_blocked_path(self, sampler, generator):
        solver = BypassSolver(max_iterations=100, sampler=sampler, collision_time_step=0.05)
        start = State(Vector2D(0, 0), Vector2D(0, 0))
        goal = State(Vector2D(2000, 0), Vector2D(0, 0))
        
        # Big obstacle blocking the direct line
        obstacles = [GenericCircleObstacle(Vector2D(1000, 0), 500, padding=0)]
        
        traj = solver.solve(start, goal, obstacles, generator)
        assert traj is not None
        assert traj.root is not None
        # Verify result is collision free
        assert not CollisionEngine.is_collision(traj.root, obstacles)
        assert not CollisionEngine.is_collision(traj.root.child, obstacles)


class TestBypassSolverStability:
    """
    Sampling alone returns a structurally different path every cycle for unchanged
    inputs, so the controller never gets a reference it can settle on. The previous via
    point is re-solved from the current start and defended by cost_margin.
    """

    START = State(Vector2D(-2000, 0), Vector2D(0, 0))
    GOAL = State(Vector2D(2000, 0), Vector2D(0, 0))

    def _solver(self, sampler, margin=0.15):
        return BypassSolver(
            max_iterations=20,
            sampler=sampler,
            collision_time_step=0.04,
            cost_margin=margin,
        )

    def _blocking(self, radius=500):
        return [GenericCircleObstacle(Vector2D(0, 0), radius, padding=0)]

    def test_the_via_point_is_reported(self, sampler, generator):
        traj = self._solver(sampler).solve(
            self.START, self.GOAL, self._blocking(), generator
        )

        assert traj.via_state is not None

    def test_an_unbeaten_via_point_survives_replanning(self, sampler, generator):
        solver = self._solver(sampler)
        obstacles = self._blocking()

        first = solver.solve(self.START, self.GOAL, obstacles, generator)
        for _ in range(20):
            again = solver.solve(
                self.START, self.GOAL, obstacles, generator, first.via_state
            )
            assert again.via_state.position.distance(first.via_state.position) == 0.0

    def test_a_blocked_via_point_is_abandoned(self, sampler, generator):
        solver = self._solver(sampler)
        obstacles = self._blocking()

        first = solver.solve(self.START, self.GOAL, obstacles, generator)
        blocked = obstacles + [
            GenericCircleObstacle(first.via_state.position, 500, padding=0)
        ]

        second = solver.solve(self.START, self.GOAL, blocked, generator, first.via_state)

        assert second is not None
        assert second.via_state.position.distance(first.via_state.position) > 100.0
        assert not CollisionEngine.is_collision(second.root, blocked)
        assert not CollisionEngine.is_collision(second.root.child, blocked)

    def test_a_zero_margin_takes_any_improvement(self, sampler, generator):
        """The margin is what makes it sticky, not the warm start on its own."""
        solver = self._solver(sampler, margin=0.0)
        obstacles = self._blocking()

        first = solver.solve(self.START, self.GOAL, obstacles, generator)
        durations = {first.get_total_duration()}
        via = first.via_state
        for _ in range(20):
            result = solver.solve(self.START, self.GOAL, obstacles, generator, via)
            via = result.via_state
            durations.add(result.get_total_duration())

        assert len(durations) > 1

class TestTrajectoryOptimizer:
    def test_optimize_shortens_path(self, generator):
        optimizer = TrajectoryOptimizer(trys=10, early_stop=5)
        
        # Create a suboptimal path (two segments instead of one)
        start = State(Vector2D(0, 0), Vector2D(0, 0))
        mid = State(Vector2D(500, 500), Vector2D(0, 0))
        goal = State(Vector2D(1000, 0), Vector2D(0, 0))
        
        seg1 = generator.generate(start, mid)
        seg2 = generator.generate(mid, goal)
        seg1.add_child(seg2)
        traj = Trajectory(seg1)
        
        initial_duration = traj.get_total_duration()
        optimized_traj = optimizer.optimize(traj, generator, [])
        
        # It should ideally be shorter or equal
        assert optimized_traj.get_total_duration() <= initial_duration + 1e-6

class TestPlanner:
    def test_find_direct_path(self):
        planner = Planner()
        start = State(Vector2D(0, 0), Vector2D(0, 0))
        goal = State(Vector2D(1000, 0), Vector2D(0, 0))
        
        traj = planner.find(start, goal, [])
        assert planner.status == PlanningStatus.DIRECT_PATH
        assert traj.root is not None

    def test_find_bypass(self):
        planner = Planner(SolverConfig(max_iterations=100))
        start = State(Vector2D(0, 0), Vector2D(0, 0))
        goal = State(Vector2D(2000, 0), Vector2D(0, 0))
        obstacles = [GenericCircleObstacle(Vector2D(1000, 0), 300, padding=0)]
        
        traj = planner.find(start, goal, obstacles)
        assert planner.status == PlanningStatus.BYPASS_FOUND
        assert not CollisionEngine.is_collision(traj.root, obstacles)

    def test_handle_static_collisions_start_inside(self):
        planner = Planner()
        obs = GenericCircleObstacle(Vector2D(0, 0), 200, padding=0)
        start = State(Vector2D(50, 50), Vector2D(0, 0)) # Inside obstacle
        goal = State(Vector2D(1000, 0), Vector2D(0, 0))
        
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


class TestSamplerSpread:
    """
    The old sampler drew a 2D gaussian of sigma = distance/4 around the midpoint, so a
    corner-to-corner move scattered via points metres off the line to clear obstacles a
    couple of hundred millimetres wide, and re-rolled the route every time the warm
    start was unavailable.
    """

    START = Vector2D(-4500, -3000)
    GOAL = Vector2D(4500, 3000)

    def _decompose(self, sampler, spread, count=2000):
        axis = self.GOAL.subtract(self.START)
        unit = axis.multiplyByScalar(1.0 / axis.size())
        perpendicular = unit.perpendicular()

        offsets = []
        for _ in range(count):
            point = sampler.sample_near_axis(self.START, self.GOAL, spread)
            relative = point.subtract(self.START)
            offsets.append((relative.dot(unit), relative.dot(perpendicular)))
        return np.array(offsets)

    def test_offsets_are_perpendicular_to_the_line(self, sampler):
        along, cross = self._decompose(sampler, spread=0.05).T
        length = self.GOAL.subtract(self.START).size()

        # Along the line the via stays in the middle stretch; the scatter is all across.
        assert along.min() >= 0.25 * length - 1
        assert along.max() <= 0.75 * length + 1
        assert cross.std() == pytest.approx(0.05 * length, rel=0.15)

    def test_spread_widens_the_search(self, sampler):
        tight = self._decompose(sampler, spread=0.03)[:, 1].std()
        wide = self._decompose(sampler, spread=0.35)[:, 1].std()

        assert wide > tight * 5

    def test_a_degenerate_line_falls_back_to_the_field(self, sampler):
        point = sampler.sample_near_axis(Vector2D(100, 100), Vector2D(100, 100))

        assert abs(point.x) <= 6000
        assert abs(point.y) <= 4500
