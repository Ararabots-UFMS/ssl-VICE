import pytest
from unittest.mock import MagicMock, patch

from new_movement.movement_path_driver import MovementPathDriver
from new_movement.entities.motion import MotionState
from new_movement.entities.obstacle.static_obstacle import StaticObstacle

from utils.math_util import Vector2D


class FakeTime:
    """Minimal stand-in for an rclpy.time.Time supporting subtraction and
    .nanoseconds, enough for the arithmetic done in MovementPathDriver."""

    def __init__(self, seconds: float):
        self.nanoseconds = int(seconds * 1e9)

    def __sub__(self, other):
        result = FakeTime(0)
        result.nanoseconds = self.nanoseconds - other.nanoseconds
        return result


def _clock(seconds: float) -> MagicMock:
    """A stand-in for Node.get_clock, so get_clock().now() reaches the FakeTime."""
    return MagicMock(return_value=MagicMock(now=MagicMock(return_value=FakeTime(seconds))))


@pytest.fixture
def driver_node():
    with patch('rclpy.init'), \
         patch('rclpy.node.Node.__init__', return_value=None), \
         patch('new_movement.movement_path_driver.sleep', return_value=None), \
         patch.object(MovementPathDriver, 'create_subscription', return_value=MagicMock()), \
         patch.object(MovementPathDriver, 'create_publisher', return_value=MagicMock()), \
         patch.object(MovementPathDriver, 'create_timer', return_value=MagicMock()), \
         patch.object(MovementPathDriver, 'create_service', return_value=MagicMock()), \
         patch.object(MovementPathDriver, 'get_logger', return_value=MagicMock()), \
         patch.object(MovementPathDriver, 'get_clock', _clock(0.0)):
        node = MovementPathDriver()

    # Re-bind as plain instance attributes: several methods call
    # get_logger()/get_clock() again well after __init__/the `with` block
    # above (whose class-level patches are undone on exit).
    node.get_logger = MagicMock()
    node.get_clock = _clock(0.0)

    # Isolate node-level tests from the real planner/obstacle-factory
    # implementations unless a test explicitly wants the real thing.
    node.planner = MagicMock()
    node.obstacle_factory = MagicMock()

    return node


def _make_robot(id, pos=(0.0, 0.0), vel=(0.0, 0.0)):
    robot = MagicMock()
    robot.id = id
    robot.position_x, robot.position_y = pos
    robot.velocity_x, robot.velocity_y = vel
    return robot


class TestGameStateCallback:
    def test_stores_indexed_state(self, driver_node):
        ally1 = _make_robot(1)
        ally2 = _make_robot(2)
        enemy1 = _make_robot(9)
        msg = MagicMock()
        msg.ally_robots = [ally1, ally2]
        msg.enemy_robots = [enemy1]
        msg.balls = [MagicMock()]
        msg.geometry = MagicMock()

        driver_node.game_state_callback(msg)

        assert driver_node.ally_robots == {1: ally1, 2: ally2}
        assert driver_node.enemy_robots == {9: enemy1}
        assert driver_node.balls == list(msg.balls)
        assert driver_node.geometry is msg.geometry


class TestDriverInit:
    def test_new_robot_without_last_command_plans_to_self(self, driver_node):
        robot = _make_robot(1, pos=(100.0, 200.0), vel=(1.0, 2.0))
        driver_node.ally_robots = {1: robot}
        driver_node.robot_data = {}
        driver_node.last_command = {}

        trajectory = MagicMock()
        driver_node.planner.find.return_value = trajectory

        driver_node.driver_init()

        assert driver_node.robot_data[1]["trajectory"] is trajectory
        assert driver_node.robot_data[1]["time_offset"] == 0.0
        (start, goal, obstacles), _ = driver_node.planner.find.call_args
        assert start.position == Vector2D(100.0, 200.0)
        assert goal.position == Vector2D(100.0, 200.0)  # plans to itself
        assert obstacles == []

    def test_new_robot_with_last_command_plans_to_command(self, driver_node):
        robot = _make_robot(1, pos=(0.0, 0.0), vel=(0.0, 0.0))
        driver_node.ally_robots = {1: robot}
        driver_node.robot_data = {}
        last_cmd = MagicMock()
        last_cmd.position_x, last_cmd.position_y = 500.0, 500.0
        last_cmd.velocity_x, last_cmd.velocity_y = 10.0, 10.0
        driver_node.last_command = {1: last_cmd}

        trajectory = MagicMock()
        driver_node.planner.find.return_value = trajectory

        driver_node.driver_init()

        (start, goal, obstacles), _ = driver_node.planner.find.call_args
        assert goal.position == Vector2D(500.0, 500.0)
        assert goal.velocity == Vector2D(10.0, 10.0)

    def test_existing_robot_is_not_replanned(self, driver_node):
        robot = _make_robot(1)
        driver_node.ally_robots = {1: robot}
        existing_entry = {"trajectory": "existing", "time_offset": 3.0, "obstacles": [], "last_obs_request": None}
        driver_node.robot_data = {1: existing_entry}
        driver_node.last_command = {}

        driver_node.driver_init()

        driver_node.planner.find.assert_not_called()
        assert driver_node.robot_data[1] is existing_entry

    def test_planner_exception_leaves_robot_with_no_trajectory(self, driver_node):
        robot = _make_robot(1)
        driver_node.ally_robots = {1: robot}
        driver_node.robot_data = {}
        driver_node.last_command = {}
        driver_node.planner.find.side_effect = RuntimeError("solver blew up")

        driver_node.driver_init()

        assert driver_node.robot_data[1]["trajectory"] is None
        driver_node.get_logger().warn.assert_called()

    def test_removes_robots_no_longer_present(self, driver_node):
        driver_node.ally_robots = {}
        driver_node.robot_data = {
            99: {"trajectory": None, "time_offset": 0.0, "obstacles": [], "last_obs_request": None}
        }
        driver_node.last_command = {}

        driver_node.driver_init()

        assert 99 not in driver_node.robot_data

    def test_refreshes_obstacles_when_last_obs_request_present(self, driver_node):
        robot = _make_robot(1)
        driver_node.ally_robots = {1: robot}
        request = MagicMock()
        driver_node.robot_data = {
            1: {"trajectory": "traj", "time_offset": 0.0, "obstacles": [], "last_obs_request": request}
        }
        driver_node.last_command = {}
        driver_node.obstacle_factory.create_obstacles.return_value = ["obs1", "obs2"]

        driver_node.driver_init()

        driver_node.obstacle_factory.create_obstacles.assert_called_once_with(
            request, driver_node.robot_data, driver_node.geometry,
            driver_node.balls, driver_node.enemy_robots, driver_node.ally_robots,
        )
        assert driver_node.robot_data[1]["obstacles"] == ["obs1", "obs2"]


class TestPublishControl:
    def test_builds_command_list_scaled_to_meters(self, driver_node):
        driver_node.driver_init = MagicMock()
        trajectory = MagicMock()
        trajectory.get_state.return_value = MotionState(Vector2D(1000.0, 2000.0), Vector2D(500.0, -500.0))
        trajectory.get_total_duration.return_value = 10.0
        driver_node.robot_data = {1: {"trajectory": trajectory, "time_offset": 0.0}}
        driver_node.get_clock = _clock(1.0)
        driver_node.last_time = FakeTime(0.0)

        driver_node.publish_control()

        driver_node.publisher.publish.assert_called_once()
        published = driver_node.publisher.publish.call_args[0][0]
        assert len(published.command) == 1
        cmd = published.command[0]
        assert cmd.id == 1
        assert cmd.position_x == pytest.approx(1.0)
        assert cmd.position_y == pytest.approx(2.0)
        assert cmd.velocity_x == pytest.approx(0.5)
        assert cmd.velocity_y == pytest.approx(-0.5)
        assert driver_node.robot_data[1]["time_offset"] == pytest.approx(1.0)

    def test_skips_robots_without_trajectory(self, driver_node):
        driver_node.driver_init = MagicMock()
        driver_node.robot_data = {1: {"trajectory": None, "time_offset": 0.0}}
        driver_node.get_clock = _clock(1.0)
        driver_node.last_time = FakeTime(0.0)

        driver_node.publish_control()

        published = driver_node.publisher.publish.call_args[0][0]
        assert published.command == []

    def test_skips_robots_with_no_state_at_offset(self, driver_node):
        driver_node.driver_init = MagicMock()
        trajectory = MagicMock()
        trajectory.get_state.return_value = None
        trajectory.get_total_duration.return_value = 10.0
        driver_node.robot_data = {1: {"trajectory": trajectory, "time_offset": 0.0}}
        driver_node.get_clock = _clock(1.0)
        driver_node.last_time = FakeTime(0.0)

        driver_node.publish_control()

        published = driver_node.publisher.publish.call_args[0][0]
        assert published.command == []

    def test_time_offset_clamped_once_past_duration(self, driver_node):
        driver_node.driver_init = MagicMock()
        trajectory = MagicMock()
        trajectory.get_state.return_value = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        trajectory.get_total_duration.return_value = 1.0
        driver_node.robot_data = {1: {"trajectory": trajectory, "time_offset": 5.0}}
        driver_node.get_clock = _clock(1.0)
        driver_node.last_time = FakeTime(0.0)

        driver_node.publish_control()

        assert driver_node.robot_data[1]["time_offset"] == 1.0

    def test_exception_is_caught_and_logged(self, driver_node):
        driver_node.driver_init = MagicMock()
        bad_trajectory = MagicMock()
        bad_trajectory.get_state.side_effect = RuntimeError("boom")
        driver_node.robot_data = {1: {"trajectory": bad_trajectory, "time_offset": 0.0}}
        driver_node.get_clock = _clock(1.0)
        driver_node.last_time = FakeTime(0.0)

        driver_node.publish_control()  # must not raise

        driver_node.get_logger().error.assert_called()
        driver_node.publisher.publish.assert_called_once()


class TestPublishTrajectories:
    def test_builds_sampled_points_scaled_to_meters(self, driver_node):
        driver_node.driver_init = MagicMock()
        trajectory = MagicMock()
        trajectory.get_total_duration.return_value = 2.0
        trajectory.get_state.return_value = MotionState(Vector2D(1000.0, 0.0), Vector2D(2000.0, 0.0))
        driver_node.robot_data = {1: {"trajectory": trajectory, "time_offset": 0.0}}

        driver_node.publish_trajectories()

        driver_node.trajectory_publisher.publish.assert_called_once()
        published = driver_node.trajectory_publisher.publish.call_args[0][0]
        assert len(published.trajectories) == 1
        robot_traj = published.trajectories[0]
        assert robot_traj.robot_id == 1
        assert robot_traj.total_duration == pytest.approx(2.0)
        assert len(robot_traj.points) > 0
        first_point = robot_traj.points[0]
        assert first_point.x == pytest.approx(1.0)
        assert first_point.velocity_x == pytest.approx(2.0)

    def test_skips_robots_without_trajectory(self, driver_node):
        driver_node.driver_init = MagicMock()
        driver_node.robot_data = {1: {"trajectory": None, "time_offset": 0.0}}

        driver_node.publish_trajectories()

        published = driver_node.trajectory_publisher.publish.call_args[0][0]
        assert published.trajectories == []

    def test_skips_robots_with_zero_or_none_duration(self, driver_node):
        driver_node.driver_init = MagicMock()
        traj_zero = MagicMock()
        traj_zero.get_total_duration.return_value = 0.0
        traj_none = MagicMock()
        traj_none.get_total_duration.return_value = None
        driver_node.robot_data = {
            1: {"trajectory": traj_zero, "time_offset": 0.0},
            2: {"trajectory": traj_none, "time_offset": 0.0},
        }

        driver_node.publish_trajectories()

        published = driver_node.trajectory_publisher.publish.call_args[0][0]
        assert published.trajectories == []

    def test_per_robot_exception_is_logged_and_others_continue(self, driver_node):
        driver_node.driver_init = MagicMock()
        bad = MagicMock()
        bad.get_total_duration.side_effect = RuntimeError("boom")
        good = MagicMock()
        good.get_total_duration.return_value = 1.0
        good.get_state.return_value = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        driver_node.robot_data = {
            1: {"trajectory": bad, "time_offset": 0.0},
            2: {"trajectory": good, "time_offset": 0.0},
        }

        driver_node.publish_trajectories()

        driver_node.get_logger().warn.assert_called()
        published = driver_node.trajectory_publisher.publish.call_args[0][0]
        assert len(published.trajectories) == 1
        assert published.trajectories[0].robot_id == 2


class TestPublishTrajectoriesSafe:
    def test_swallows_exceptions_from_publish_trajectories(self, driver_node):
        driver_node.publish_trajectories = MagicMock(side_effect=RuntimeError("boom"))

        driver_node.publish_trajectories_safe()  # must not raise

        driver_node.get_logger().warn.assert_called()


class TestReplan:
    def test_unknown_robot_returns_false(self, driver_node):
        driver_node.robot_data = {}

        result = driver_node.replan(42, MotionState(Vector2D(0, 0), Vector2D(0, 0)))

        assert result is False

    def test_successful_replan_updates_state(self, driver_node):
        current_trajectory = MagicMock()
        current_trajectory.get_state.return_value = MotionState(Vector2D(10, 10), Vector2D(0, 0))
        driver_node.robot_data = {
            1: {"trajectory": current_trajectory, "time_offset": 2.0, "obstacles": ["obs"]}
        }

        new_trajectory = MagicMock()
        new_trajectory.root = MagicMock()
        new_trajectory.get_state.return_value = MotionState(Vector2D(10, 10), Vector2D(0, 0))
        driver_node.planner.find.return_value = new_trajectory
        driver_node.get_clock = _clock(5.0)

        new_destination = MotionState(Vector2D(1000, 1000), Vector2D(0, 0))
        result = driver_node.replan(1, new_destination)

        assert result is True
        assert driver_node.robot_data[1]["trajectory"] is new_trajectory
        assert driver_node.robot_data[1]["time_offset"] == 0.0
        driver_node.planner.find.assert_called_once_with(
            current_trajectory.get_state.return_value, new_destination, ["obs"]
        )

    def test_failed_replan_when_root_is_none(self, driver_node):
        current_trajectory = MagicMock()
        current_trajectory.get_state.return_value = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        driver_node.robot_data = {1: {"trajectory": current_trajectory, "time_offset": 0.0, "obstacles": []}}

        failed_trajectory = MagicMock()
        failed_trajectory.root = None
        driver_node.planner.find.return_value = failed_trajectory

        result = driver_node.replan(1, MotionState(Vector2D(1000, 0), Vector2D(0, 0)))

        assert result is False
        assert driver_node.robot_data[1]["trajectory"] is current_trajectory

    def test_failed_replan_when_no_state_at_zero(self, driver_node):
        current_trajectory = MagicMock()
        current_trajectory.get_state.return_value = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        driver_node.robot_data = {1: {"trajectory": current_trajectory, "time_offset": 0.0, "obstacles": []}}

        failed_trajectory = MagicMock()
        failed_trajectory.root = MagicMock()
        failed_trajectory.get_state.return_value = None
        driver_node.planner.find.return_value = failed_trajectory

        result = driver_node.replan(1, MotionState(Vector2D(1000, 0), Vector2D(0, 0)))

        assert result is False


class TestCheckCollision:
    def test_no_collision_does_not_replan(self, driver_node):
        obs = MagicMock(spec=StaticObstacle)
        obs.isCollidingAt.return_value = False
        trajectory = MagicMock()
        trajectory.get_state.return_value = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        driver_node.robot_data = {1: {"trajectory": trajectory, "time_offset": 0.0, "obstacles": [obs]}}
        driver_node.replan = MagicMock()

        driver_node.check_collision()

        driver_node.replan.assert_not_called()

    def test_static_collision_triggers_replan(self, driver_node):
        obs = MagicMock(spec=StaticObstacle)
        obs.isCollidingAt.return_value = True
        trajectory = MagicMock()
        trajectory.get_state.return_value = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        trajectory.get_destination.return_value = MotionState(Vector2D(1000, 0), Vector2D(0, 0))
        driver_node.robot_data = {1: {"trajectory": trajectory, "time_offset": 0.0, "obstacles": [obs]}}
        driver_node.replan = MagicMock()

        driver_node.check_collision()

        driver_node.replan.assert_called_once_with(1, trajectory.get_destination.return_value)

    def test_dynamic_collision_triggers_replan(self, driver_node):
        # Not a StaticObstacle -> the (position, time) branch is used.
        obs = MagicMock()
        obs.isCollidingAt.return_value = True
        trajectory = MagicMock()
        trajectory.get_state.return_value = MotionState(Vector2D(0, 0), Vector2D(0, 0))
        trajectory.get_destination.return_value = MotionState(Vector2D(1000, 0), Vector2D(0, 0))
        driver_node.robot_data = {1: {"trajectory": trajectory, "time_offset": 0.0, "obstacles": [obs]}}
        driver_node.replan = MagicMock()

        driver_node.check_collision()

        driver_node.replan.assert_called_once()
        assert obs.isCollidingAt.call_args[0][1] == pytest.approx(0.0)


class TestUpdateObstacles:
    def test_unknown_robot_returns_failure(self, driver_node):
        driver_node.driver_init = MagicMock()
        driver_node.robot_data = {}
        request = MagicMock()
        request.id = 99
        response = MagicMock()

        result = driver_node.update_obstacles(request, response)

        assert result.success is False
        driver_node.get_logger().debug.assert_called()

    def test_known_robot_updates_obstacles(self, driver_node):
        driver_node.driver_init = MagicMock()
        driver_node.robot_data = {1: {"trajectory": None, "time_offset": 0.0, "obstacles": [], "last_obs_request": None}}
        driver_node.obstacle_factory.create_obstacles.return_value = ["obs1"]
        request = MagicMock()
        request.id = 1
        response = MagicMock()

        result = driver_node.update_obstacles(request, response)

        assert result.success is True
        assert driver_node.robot_data[1]["obstacles"] == ["obs1"]
        assert driver_node.robot_data[1]["last_obs_request"] is request


class TestUpdateObstaclesTimerCallback:
    def test_refreshes_only_robots_with_last_request(self, driver_node):
        req = MagicMock()
        driver_node.robot_data = {
            1: {"obstacles": [], "last_obs_request": req},
            2: {"obstacles": ["stale"], "last_obs_request": None},
        }
        driver_node.obstacle_factory.create_obstacles.return_value = ["fresh"]

        driver_node.update_obstacles_timer_callback()

        assert driver_node.robot_data[1]["obstacles"] == ["fresh"]
        assert driver_node.robot_data[2]["obstacles"] == ["stale"]


class TestUpdateTarget:
    def test_unknown_robot_returns_failure(self, driver_node):
        driver_node.driver_init = MagicMock()
        driver_node.robot_data = {}
        request = MagicMock()
        request.id = 99
        response = MagicMock()

        result = driver_node.update_target(request, response)

        assert result.success is False

    def test_known_robot_stores_command_and_replans(self, driver_node):
        driver_node.driver_init = MagicMock()
        driver_node.robot_data = {1: {"trajectory": MagicMock(), "time_offset": 0.0, "obstacles": []}}
        request = MagicMock()
        request.id = 1
        request.position_x, request.position_y = 100.0, 200.0
        request.velocity_x, request.velocity_y = 1.0, 2.0
        response = MagicMock()
        driver_node.replan = MagicMock(return_value=True)

        result = driver_node.update_target(request, response)

        assert result.success is True
        assert driver_node.last_command[1] is request
        replan_args = driver_node.replan.call_args[0]
        assert replan_args[0] == 1
        assert replan_args[1].position == Vector2D(100.0, 200.0)
        assert replan_args[1].velocity == Vector2D(1.0, 2.0)

    def test_replan_failure_propagates(self, driver_node):
        driver_node.driver_init = MagicMock()
        driver_node.robot_data = {1: {"trajectory": MagicMock(), "time_offset": 0.0, "obstacles": []}}
        request = MagicMock()
        request.id = 1
        request.position_x, request.position_y = 0.0, 0.0
        request.velocity_x, request.velocity_y = 0.0, 0.0
        response = MagicMock()
        driver_node.replan = MagicMock(return_value=False)

        result = driver_node.update_target(request, response)

        assert result.success is False
