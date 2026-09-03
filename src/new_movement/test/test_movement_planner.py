import pytest
from unittest.mock import MagicMock, patch

from new_movement.movement_planner import MovementPlanner
from new_movement.entities.motion import MotionState

from utils.math_util import Vector2D


class FakeTime:
    """Minimal stand-in for an rclpy.time.Time, enough for `.nanoseconds`."""

    def __init__(self, seconds: float):
        self.nanoseconds = int(seconds * 1e9)


DEFAULT_PARAMS = {
    "planner_freq": 50.0,
    "max_threads": 8,
    "overhead_max_age": 0.5,
    "accept_radius": 10,
}


@pytest.fixture
def planner_node():
    params = dict(DEFAULT_PARAMS)

    def fake_get_parameter(name):
        return MagicMock(value=params[name])

    with patch('rclpy.init'), \
         patch('rclpy.node.Node.__init__', return_value=None), \
         patch.object(MovementPlanner, 'create_subscription', return_value=MagicMock()), \
         patch.object(MovementPlanner, 'create_publisher', return_value=MagicMock()), \
         patch.object(MovementPlanner, 'create_timer', return_value=MagicMock()), \
         patch.object(MovementPlanner, 'declare_parameter', return_value=MagicMock()), \
         patch.object(MovementPlanner, 'get_parameter', side_effect=fake_get_parameter), \
         patch.object(MovementPlanner, 'get_logger', return_value=MagicMock()), \
         patch.object(MovementPlanner, 'get_clock', return_value=MagicMock(now=MagicMock(return_value=FakeTime(0.0)))):
        node = MovementPlanner()

    # Re-bind as plain instance attributes: the node's own methods
    # (plan_for_robot in particular) call get_parameter/get_logger/get_clock
    # again at runtime, well after __init__ and after the `with` block
    # above (and its class-level patches) has already exited.
    node.get_parameter = MagicMock(side_effect=fake_get_parameter)
    node.get_logger = MagicMock()
    node.get_clock = MagicMock(now=MagicMock(return_value=FakeTime(0.0)))
    node._test_params = params

    # Isolate node-level tests from the real planner/factory implementations
    # unless a test explicitly wants the real thing.
    node.planner = MagicMock()
    node.factory = MagicMock()
    node.par_executor = MagicMock()

    return node


def _make_target(robot_id=1, initial_pos=(0, 0), initial_vel=(0, 0),
                  target_pos=(1000, 0), target_vel=(0, 0)):
    target = MagicMock()
    target.robot_id = robot_id
    target.initial_pos = MagicMock(x=initial_pos[0], y=initial_pos[1])
    target.initial_vel = MagicMock(x=initial_vel[0], y=initial_vel[1])
    target.target_pos = MagicMock(x=target_pos[0], y=target_pos[1])
    target.target_vel = MagicMock(x=target_vel[0], y=target_vel[1])
    return target


class TestCallbacks:
    def test_target_callback_stores_msg(self, planner_node):
        msg = MagicMock()
        planner_node.target_callback(msg)
        assert planner_node.cur_targets is msg

    def test_overhead_callback_stores_by_robot_id(self, planner_node):
        msg = MagicMock()
        msg.robot_id = 7
        planner_node.overhead_callback(msg)
        assert planner_node.cur_overhead_points[7] is msg

    def test_game_state_callback_stores_msg(self, planner_node):
        msg = MagicMock()
        planner_node.game_state_callback(msg)
        assert planner_node.game_state is msg


class TestPlanningLoop:
    def test_noop_when_no_targets(self, planner_node):
        planner_node.cur_targets = None
        planner_node.game_state = MagicMock()

        planner_node.planning_loop()

        planner_node.par_executor.submit.assert_not_called()

    def test_noop_when_no_game_state(self, planner_node):
        targets = MagicMock()
        targets.targets = [_make_target(robot_id=1)]
        planner_node.cur_targets = targets
        planner_node.game_state = None

        planner_node.planning_loop()

        planner_node.par_executor.submit.assert_not_called()

    def test_submits_task_per_target(self, planner_node):
        targets = MagicMock()
        targets.targets = [_make_target(robot_id=1), _make_target(robot_id=2)]
        planner_node.cur_targets = targets
        planner_node.game_state = MagicMock()

        future = MagicMock()
        planner_node.par_executor.submit.return_value = future

        planner_node.planning_loop()

        assert planner_node.par_executor.submit.call_count == 2
        assert set(planner_node.active_futures.keys()) == {1, 2}
        assert future.add_done_callback.call_count == 2

    def test_skips_robot_with_active_unfinished_future(self, planner_node):
        targets = MagicMock()
        targets.targets = [_make_target(robot_id=1)]
        planner_node.cur_targets = targets
        planner_node.game_state = MagicMock()

        pending_future = MagicMock()
        pending_future.done.return_value = False
        planner_node.active_futures = {1: pending_future}

        planner_node.planning_loop()

        planner_node.par_executor.submit.assert_not_called()

    def test_resubmits_robot_whose_future_is_done(self, planner_node):
        targets = MagicMock()
        targets.targets = [_make_target(robot_id=1)]
        planner_node.cur_targets = targets
        planner_node.game_state = MagicMock()

        finished_future = MagicMock()
        finished_future.done.return_value = True
        planner_node.active_futures = {1: finished_future}

        planner_node.planning_loop()

        planner_node.par_executor.submit.assert_called_once()


class TestMakePublishCallback:
    def test_discards_result_for_robot_no_longer_targeted(self, planner_node):
        targets = MagicMock()
        other_target = _make_target(robot_id=2)
        targets.targets = [other_target]
        planner_node.cur_targets = targets

        future = MagicMock()
        callback = planner_node.make_publish_callback(1)
        callback(future)

        future.result.assert_not_called()
        planner_node.trajectory_pub.publish.assert_not_called()

    def test_publishes_trajectory_on_success(self, planner_node):
        planner_node.cur_targets = None  # discard-check only runs if cur_targets is truthy

        trajectory = MagicMock()
        msg = MagicMock()
        trajectory.to_msg.return_value = msg
        future = MagicMock()
        future.result.return_value = (1, trajectory, 2.5)

        callback = planner_node.make_publish_callback(1)
        callback(future)

        trajectory.to_msg.assert_called_once_with(1)
        assert msg.handoff_stamp == 2.5
        planner_node.trajectory_pub.publish.assert_called_once_with(msg)

    def test_no_publish_when_result_is_none(self, planner_node):
        planner_node.cur_targets = None
        future = MagicMock()
        future.result.return_value = None

        callback = planner_node.make_publish_callback(1)
        callback(future)

        planner_node.trajectory_pub.publish.assert_not_called()

    def test_exception_in_result_is_logged_not_raised(self, planner_node):
        planner_node.cur_targets = None
        future = MagicMock()
        future.result.side_effect = RuntimeError("boom")

        callback = planner_node.make_publish_callback(1)
        callback(future)  # must not raise

        planner_node.get_logger().error.assert_called()
        planner_node.trajectory_pub.publish.assert_not_called()


class TestPlanForRobot:
    def test_no_overhead_point_plans_from_vision_state(self, planner_node):
        target = _make_target(robot_id=1, initial_pos=(0, 0), initial_vel=(0, 0),
                               target_pos=(1000, 0), target_vel=(0, 0))
        planner_node.cur_overhead_points = {}
        planner_node.game_state = MagicMock()
        planner_node.factory.create_obstacles.return_value = []

        trajectory = MagicMock()
        trajectory.root = MagicMock()
        planner_node.planner.find.return_value = trajectory

        result = planner_node.plan_for_robot(target)

        assert result == (1, trajectory, 0.0)
        args, _ = planner_node.planner.find.call_args
        start_state, goal_state, obstacles = args
        assert isinstance(start_state, MotionState)
        assert start_state.position == Vector2D(0, 0)
        assert goal_state.position == Vector2D(1000, 0)
        assert obstacles == []

    def test_fresh_overhead_point_used_as_start_state(self, planner_node):
        target = _make_target(robot_id=1, target_pos=(1000, 0))
        overhead = MagicMock()
        overhead.pos = MagicMock(x=500, y=50)
        overhead.vel = MagicMock(x=10, y=20)
        overhead.wall_stamp = 100.0
        planner_node.cur_overhead_points = {1: overhead}
        planner_node.game_state = MagicMock()
        planner_node.factory.create_obstacles.return_value = []
        # now_sec == wall_stamp -> age == 0, well within overhead_max_age
        planner_node.get_clock = MagicMock(now=MagicMock(return_value=FakeTime(100.0)))

        trajectory = MagicMock()
        trajectory.root = MagicMock()
        planner_node.planner.find.return_value = trajectory

        result = planner_node.plan_for_robot(target)

        assert result == (1, trajectory, 100.0)
        args, _ = planner_node.planner.find.call_args
        start_state = args[0]
        assert start_state.position == Vector2D(500, 50)
        assert start_state.velocity == Vector2D(10, 20)

    def test_stale_overhead_point_near_goal_returns_none(self, planner_node):
        target = _make_target(robot_id=1, initial_pos=(995, 0), target_pos=(1000, 0))
        overhead = MagicMock()
        overhead.pos = MagicMock(x=500, y=0)
        overhead.vel = MagicMock(x=0, y=0)
        overhead.wall_stamp = 0.0
        planner_node.cur_overhead_points = {1: overhead}
        planner_node.game_state = MagicMock()
        # now_sec - wall_stamp = 100.0 >> overhead_max_age(0.5) -> stale
        planner_node.get_clock = MagicMock(now=MagicMock(return_value=FakeTime(100.0)))

        result = planner_node.plan_for_robot(target)

        assert result is None
        planner_node.planner.find.assert_not_called()

    def test_stale_overhead_point_far_from_goal_replans_from_vision(self, planner_node):
        target = _make_target(robot_id=1, initial_pos=(0, 0), initial_vel=(1, 2), target_pos=(1000, 0))
        overhead = MagicMock()
        overhead.pos = MagicMock(x=500, y=0)
        overhead.vel = MagicMock(x=0, y=0)
        overhead.wall_stamp = 0.0
        planner_node.cur_overhead_points = {1: overhead}
        planner_node.game_state = MagicMock()
        planner_node.factory.create_obstacles.return_value = []
        planner_node.get_clock = MagicMock(now=MagicMock(return_value=FakeTime(100.0)))

        trajectory = MagicMock()
        trajectory.root = MagicMock()
        planner_node.planner.find.return_value = trajectory

        result = planner_node.plan_for_robot(target)

        assert result == (1, trajectory, 0.0)
        args, _ = planner_node.planner.find.call_args
        start_state = args[0]
        # Falls back to the target's vision-reported initial pos/vel.
        assert start_state.position == Vector2D(0, 0)
        assert start_state.velocity == Vector2D(1, 2)

    def test_solver_exception_is_logged_and_returns_none(self, planner_node):
        target = _make_target(robot_id=1)
        planner_node.cur_overhead_points = {}
        planner_node.game_state = MagicMock()
        planner_node.factory.create_obstacles.return_value = []
        planner_node.planner.find.side_effect = RuntimeError("solver exploded")

        result = planner_node.plan_for_robot(target)

        assert result is None
        planner_node.get_logger().warn.assert_called()

    def test_trajectory_without_root_returns_none(self, planner_node):
        target = _make_target(robot_id=1)
        planner_node.cur_overhead_points = {}
        planner_node.game_state = MagicMock()
        planner_node.factory.create_obstacles.return_value = []

        trajectory = MagicMock()
        trajectory.root = None
        planner_node.planner.find.return_value = trajectory

        result = planner_node.plan_for_robot(target)

        assert result is None

    def test_obstacles_built_from_game_state_when_present(self, planner_node):
        target = _make_target(robot_id=1)
        planner_node.cur_overhead_points = {}
        game_state = MagicMock()
        planner_node.game_state = game_state
        planner_node.factory.create_obstacles.return_value = ["obstacle"]

        trajectory = MagicMock()
        trajectory.root = MagicMock()
        planner_node.planner.find.return_value = trajectory

        planner_node.plan_for_robot(target)

        planner_node.factory.create_obstacles.assert_called_once_with(
            robot_id=1,
            config=target,
            geometry=game_state.geometry,
            balls=game_state.balls,
            enemy_robots=game_state.enemy_robots,
            ally_robots=game_state.ally_robots,
            ally_info=planner_node.cur_overhead_points,
        )
