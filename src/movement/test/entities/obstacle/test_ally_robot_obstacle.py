import numpy as np
import pytest

from movement.entities.obstacle.ally_robot_obstacle import AllyRobotObstacle
from movement.entities.obstacle.obstacle import Obstacle
from movement.entities.motion.motion_state import MotionState
from utils.math_util import Vector2D


class _FakeTrajectory:
    """Duck-typed stand-in for `Trajectory`: only `get_position` is used by
    AllyRobotObstacle, so a real Trajectory (with its full motion-planning
    machinery) is unnecessary for these unit tests."""

    def __init__(self, start: Vector2D, velocity: Vector2D = None):
        self.start = start
        self.velocity = velocity or Vector2D(0, 0)
        self.calls = []

    def get_position(self, t: float) -> Vector2D:
        self.calls.append(t)
        return self.start.add(self.velocity.multiplyByScalar(t))


@pytest.fixture
def robot_state():
    return MotionState(Vector2D(1000, 0), Vector2D(0, 0))


@pytest.fixture
def stationary_trajectory():
    return _FakeTrajectory(Vector2D(1000, 0))


@pytest.fixture
def obstacle(robot_state, stationary_trajectory):
    return AllyRobotObstacle(robot_state, stationary_trajectory, radius=100)


class TestConstructor:
    def test_default_fields(self, robot_state, stationary_trajectory):
        obs = AllyRobotObstacle(robot_state, stationary_trajectory)
        assert obs.robotState is robot_state
        assert obs.trajectory is stationary_trajectory
        assert obs.time_offset == 0.0
        assert obs.radius == 90

    def test_explicit_fields(self, robot_state, stationary_trajectory):
        obs = AllyRobotObstacle(
            robot_state, stationary_trajectory, time_offset=2.5, radius=150
        )
        assert obs.time_offset == 2.5
        assert obs.radius == 150

    def test_is_obstacle(self, obstacle):
        assert isinstance(obstacle, Obstacle)


class TestDistanceTo:
    def test_outside_radius(self, obstacle):
        assert obstacle.distanceTo(Vector2D(1300, 0), t=0) == pytest.approx(200.0)

    def test_inside_radius_is_negative(self, obstacle):
        assert obstacle.distanceTo(Vector2D(1050, 0), t=0) == pytest.approx(-50.0)

    def test_exactly_on_boundary(self, obstacle):
        assert obstacle.distanceTo(Vector2D(1100, 0), t=0) == pytest.approx(0.0)

    def test_uses_time_offset_plus_t(self, robot_state):
        traj = _FakeTrajectory(Vector2D(0, 0), Vector2D(100, 0))
        obs = AllyRobotObstacle(robot_state, traj, time_offset=5.0, radius=10)
        obs.distanceTo(Vector2D(700, 0), t=2.0)
        # position should be sampled at time_offset + t = 7.0 -> (700, 0)
        assert traj.calls[-1] == pytest.approx(7.0)


class TestIsCollidingAt:
    def test_inside_collides(self, obstacle):
        assert obstacle.isCollidingAt(Vector2D(1050, 0), t=0) is True

    def test_outside_does_not_collide(self, obstacle):
        assert obstacle.isCollidingAt(Vector2D(1300, 0), t=0) is False

    def test_boundary_does_not_collide(self, obstacle):
        # distance == 0 is not < 0
        assert obstacle.isCollidingAt(Vector2D(1100, 0), t=0) is False


class TestBatchCollides:
    def test_detects_collision(self, obstacle):
        positions = np.array([[1050.0, 0.0], [5000.0, 5000.0]])
        times = np.array([0.0, 0.0])
        assert obstacle.batch_collides(positions, times) is True

    def test_no_collision(self, obstacle):
        positions = np.array([[5000.0, 5000.0], [-5000.0, -5000.0]])
        times = np.array([0.0, 0.0])
        assert obstacle.batch_collides(positions, times) is False

    def test_moving_trajectory_uses_time_offset(self, robot_state):
        traj = _FakeTrajectory(Vector2D(0, 0), Vector2D(100, 0))
        obs = AllyRobotObstacle(robot_state, traj, time_offset=1.0, radius=10)
        # At time_offset + t = 1 + 2 = 3 -> obstacle center is at (300, 0)
        positions = np.array([[300.0, 0.0]])
        times = np.array([2.0])
        assert obs.batch_collides(positions, times) is True

    def test_matches_isCollidingAt_for_single_point(self, obstacle):
        positions = np.array([[1050.0, 0.0]])
        times = np.array([0.0])
        assert obstacle.batch_collides(positions, times) == obstacle.isCollidingAt(
            Vector2D(1050.0, 0.0), 0.0
        )

    def test_empty_positions(self, obstacle):
        positions = np.zeros((0, 2))
        times = np.zeros((0,))
        assert obstacle.batch_collides(positions, times) is False


class TestAdaptDestination:
    def test_returns_same_point_when_not_colliding(self, obstacle):
        target = Vector2D(1300, 0)
        assert obstacle.adaptDestination(target, t=0) == target

    def test_projects_colliding_point_to_boundary(self, obstacle):
        result = obstacle.adaptDestination(Vector2D(1050, 0), t=0)
        assert result.x == pytest.approx(1100.0)
        assert result.y == pytest.approx(0.0)

    def test_target_at_center_uses_arbitrary_direction(self, obstacle):
        result = obstacle.adaptDestination(Vector2D(1000, 0), t=0)
        # arbitrary direction is (1, 0)
        assert result.x == pytest.approx(1100.0)
        assert result.y == pytest.approx(0.0)

    def test_uses_trajectory_position_at_given_time(self, robot_state):
        traj = _FakeTrajectory(Vector2D(0, 0), Vector2D(100, 0))
        obs = AllyRobotObstacle(robot_state, traj, radius=50)
        # At t=2, trajectory center is (200, 0); target inside radius.
        result = obs.adaptDestination(Vector2D(210, 0), t=2.0)
        assert result.x == pytest.approx(250.0)
        assert result.y == pytest.approx(0.0)


class TestUpdaters:
    def test_update_state(self, obstacle):
        new_state = MotionState(Vector2D(5, 5), Vector2D(1, 1))
        obstacle.updateState(new_state)
        assert obstacle.robotState is new_state

    def test_update_trajectory(self, obstacle):
        new_traj = _FakeTrajectory(Vector2D(9, 9))
        obstacle.updateTrajectory(new_traj)
        assert obstacle.trajectory is new_traj


class TestVelocity:
    def test_velocity_returns_robot_state_velocity(self):
        state = MotionState(Vector2D(0, 0), Vector2D(42, -7))
        obs = AllyRobotObstacle(state, _FakeTrajectory(Vector2D(0, 0)))
        assert obs.velocity() == Vector2D(42, -7)


class TestSweptSegments:
    def test_an_ally_crossing_mid_step_is_caught(self):
        """
        Both bodies move, so the pair can be far apart at both ends of the step and
        still pass through each other in between.
        """
        ally_start = Vector2D(0.0, -400.0)
        ally_velocity = Vector2D(0.0, 800.0)

        class _CrossingTrajectory:
            def get_position(self, t):
                return ally_start.add(ally_velocity.multiplyByScalar(t))

        obstacle = AllyRobotObstacle(
            MotionState(ally_start, ally_velocity),
            _CrossingTrajectory(),
            time_offset=0.0,
            radius=100.0,
        )

        # Our robot crosses the same point going the other way, over the same second.
        starts = np.array([[-400.0, 0.0]])
        ends = np.array([[400.0, 0.0]])

        assert not obstacle.batch_collides(np.vstack([starts, ends]), np.array([0.0, 1.0]))
        assert obstacle.batch_collides_segments(
            starts, ends, np.array([0.0]), np.array([1.0])
        )


class TestBounds:
    def test_the_box_holds_the_whole_trajectory(self):
        import random

        from movement.entities.trajectory.trajectory import Trajectory
        from movement.entities.trajectory.trajectory_sampler import TrajectorySampler
        from movement.local_planner.trajectory_generator import TrajectoryGenerator

        generator = TrajectoryGenerator()
        rng = random.Random(13)
        for _ in range(60):
            start = MotionState(
                Vector2D(rng.uniform(-3000, 3000), rng.uniform(-2000, 2000)),
                Vector2D(rng.uniform(-2000, 2000), rng.uniform(-2000, 2000)),
            )
            goal = MotionState(
                Vector2D(rng.uniform(-3000, 3000), rng.uniform(-2000, 2000)),
                Vector2D(0.0, 0.0),
            )
            trajectory = Trajectory(generator.generate(start, goal))
            obstacle = AllyRobotObstacle(start, trajectory, radius=190)

            min_x, min_y, max_x, max_y = obstacle.bounds()
            duration = TrajectorySampler(trajectory.root).duration
            for t in np.linspace(0.0, max(duration, 1e-9), 200):
                point = trajectory.get_position(float(t))
                assert min_x <= point.x - obstacle.radius + 1e-6
                assert max_x >= point.x + obstacle.radius - 1e-6
                assert min_y <= point.y - obstacle.radius + 1e-6
                assert max_y >= point.y + obstacle.radius - 1e-6

    def test_anything_but_a_real_trajectory_declines_to_bound_itself(self):
        """Anything that only offers get_position keeps being tested the long way."""

        class DuckTyped:
            def get_position(self, t):
                return Vector2D(0.0, 0.0)

        state = MotionState(Vector2D(0.0, 0.0), Vector2D(0.0, 0.0))
        assert AllyRobotObstacle(state, DuckTyped()).bounds() is None
