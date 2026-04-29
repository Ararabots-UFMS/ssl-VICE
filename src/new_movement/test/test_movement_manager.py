import pytest
from unittest.mock import MagicMock
from movement_interfaces.msg import Vector2D as Vector2DMsg

from new_movement.movement_manager import MovementManager


def _make_vector(x: float, y: float):
    vec = Vector2DMsg()
    vec.x = float(x)
    vec.y = float(y)
    return vec

class FakeFuture:
    def __init__(self, response):
        self._response = response

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self._response

@pytest.fixture
def test_robot():
    robot = MagicMock()
    robot.id = 1
    robot.position_x = 200.0
    robot.position_y = 100.0
    robot.velocity_x = 3.0
    robot.velocity_y = 3.0

    return robot

@pytest.fixture
def test_command():
    cmd = MagicMock()
    cmd.robot_id = 1
    cmd.target_pos = _make_vector(300.0, 200.0)

    return cmd

@pytest.fixture
def movement_manager():
    mgr = MovementManager.__new__(MovementManager)

    mgr.get_logger = MagicMock()
    mgr.create_subscription = MagicMock()
    mgr.create_client = MagicMock()
    mgr.create_publisher = MagicMock()
    mgr.create_timer = MagicMock()

    mgr._target_array_pub = MagicMock()

    mgr.StaticObstacles_srv = MagicMock()
    mgr.GoalKeeper_srv = MagicMock()
    mgr.TeamColor_srv = MagicMock()

    mgr._MovementManager__game_config_inflight = False
    mgr._static_obstacles_inflight = False
    mgr._goal_keeper_inflight = False

    mgr._movement_commands = None
    mgr.yellow_robots = None
    mgr.blue_robots = None
    mgr._team_color_yellow = None
    mgr._static_obstacles = None
    mgr._goal_keeper_id = None

    return mgr


@pytest.fixture
def manager(movement_manager):
    return movement_manager

@pytest.fixture
def game_config_response():
    resp = MagicMock()
    resp.is_team_color_yellow = True
    return resp

@pytest.fixture
def static_obstacles_response():
    resp = MagicMock()
    resp.border_area = True
    resp.center_circle = False
    return resp

@pytest.fixture
def goal_keeper_response():
    resp = MagicMock()
    resp.robot_id = 1
    return resp

@pytest.fixture
def configured_service_future(manager, game_config_response, static_obstacles_response, goal_keeper_response):
    manager.TeamColor_srv.service_is_ready.return_value = True
    manager.StaticObstacles_srv.service_is_ready.return_value = True
    manager.GoalKeeper_srv.service_is_ready.return_value = True

    manager.TeamColor_srv.call_async.return_value = FakeFuture(game_config_response)
    manager.StaticObstacles_srv.call_async.return_value = FakeFuture(static_obstacles_response)
    manager.GoalKeeper_srv.call_async.return_value = FakeFuture(goal_keeper_response)

    return manager

def test_poll_game_config(configured_service_future):
    mgr = configured_service_future
    mgr._poll_game_config()
    assert mgr._team_color_yellow is True

def test_poll_static_obstacles(configured_service_future):
    mgr = configured_service_future
    mgr._poll_static_obstacles()
    assert mgr._static_obstacles == {'border_area': True, 'center_circle': False}

def test_poll_goal_keeper(configured_service_future):
    mgr = configured_service_future
    mgr._poll_goal_keeper()
    assert mgr._goal_keeper_id == 1

def test_poll_game_config_skip(manager):
    manager.TeamColor_srv.service_is_ready.return_value = False
    manager._poll_game_config()
    manager.TeamColor_srv.call_async.assert_not_called()

def test_poll_game_config_inflight(manager):
    manager.TeamColor_srv.service_is_ready.return_value = True
    manager._MovementManager__game_config_inflight = True
    manager._poll_game_config()
    manager.TeamColor_srv.call_async.assert_not_called()

def test_poll_static_obstacles_skip(manager):
    manager.StaticObstacles_srv.service_is_ready.return_value = False
    manager._poll_static_obstacles()
    manager.StaticObstacles_srv.call_async.assert_not_called()

def test_poll_static_obstacles_inflight(manager):
    manager.StaticObstacles_srv.service_is_ready.return_value = True
    manager._static_obstacles_inflight = True
    manager._poll_static_obstacles()
    manager.StaticObstacles_srv.call_async.assert_not_called()

def test_poll_goal_keeper_skip(manager):
    manager.GoalKeeper_srv.service_is_ready.return_value = False
    manager._poll_goal_keeper()
    manager.GoalKeeper_srv.call_async.assert_not_called()

def test_poll_goal_keeper_inflight(manager):
    manager.GoalKeeper_srv.service_is_ready.return_value = True
    manager._goal_keeper_inflight = True
    manager._poll_goal_keeper()
    manager.GoalKeeper_srv.call_async.assert_not_called()

def test_movement_commands_callback_trigger(manager, test_command):
    manager.try_publish_targets = MagicMock()
    msg = MagicMock()
    msg.commands = [test_command]

    manager.movement_command_callback(msg)
    assert manager._movement_commands == [test_command]
    manager.try_publish_targets.assert_called_once()

def test_vision_message_callback_trigger(manager, test_robot):
    manager.try_publish_targets = MagicMock()
    msg = MagicMock()
    msg.yellow_robots = [test_robot]
    msg.blue_robots = []

    manager.vision_message_callback(msg)
    assert manager.yellow_robots == [test_robot]
    assert manager.blue_robots == []
    manager.try_publish_targets.assert_called_once()

def test_try_publish_targets_no_commands(manager):
    manager._team_color_yellow = True
    manager.yellow_robots = []
    manager._static_obstacles = {'border_area': True, 'center_circle': False}
    manager._goal_keeper_id = 1

    manager.try_publish_targets()
    manager._target_array_pub.publish.assert_not_called()

def test_try_publish_targets_no_team_color(manager):
    manager._movement_commands = []
    manager.yellow_robots = []
    manager._static_obstacles = {'border_area': True, 'center_circle': False}
    manager._goal_keeper_id = 1

    manager.try_publish_targets()
    manager._target_array_pub.publish.assert_not_called()

def test_try_publish_targets_no_robots_team(manager):
    manager._movement_commands = None
    manager._team_color_yellow = True
    manager.yellow_robots = []
    manager._static_obstacles = {'border_area': True, 'center_circle': False}
    manager._goal_keeper_id = 1

    manager.try_publish_targets()
    manager._target_array_pub.publish.assert_not_called()

def test_try_publish_targets_no_static_obstacles(manager):
    manager._movement_commands = []
    manager._team_color_yellow = True
    manager.yellow_robots = []
    manager._goal_keeper_id = 1

    manager.try_publish_targets()
    manager._target_array_pub.publish.assert_not_called()

def test_try_publish_targets_no_goal_keeper(manager):
    manager._movement_commands = []
    manager._team_color_yellow = True
    manager.yellow_robots = []
    manager._static_obstacles = {'border_area': True, 'center_circle': False}

    manager.try_publish_targets()
    manager._target_array_pub.publish.assert_not_called()

def test_try_publish_targets_success(manager, test_command, test_robot):
    manager._movement_commands = [test_command]
    manager._team_color_yellow = True
    manager.yellow_robots = [test_robot]
    manager._static_obstacles = {'border_area': True, 'center_circle': False}
    manager._goal_keeper_id = 1

    manager.try_publish_targets()
    manager._target_array_pub.publish.assert_called_once()

@pytest.fixture
def test_robot():
    r = MagicMock()
    r.id = 1
    r.position_x = 10.0
    r.position_y = 20.0
    r.velocity_x = 1.0
    r.velocity_y = 2.0
    return r

@pytest.fixture
def test_robot2():
    r = MagicMock()
    r.id = 2
    r.position_x = 30.0
    r.position_y = 40.0
    r.velocity_x = 3.0
    r.velocity_y = 4.0
    return r

@pytest.fixture
def test_command():
    tp = _make_vector(100.0, 200.0)
    cmd = MagicMock()
    cmd.robot_id = 1
    cmd.target_pos = tp
    return cmd

@pytest.fixture
def test_command2():
    tp = _make_vector(300.0, 400.0)
    cmd = MagicMock()
    cmd.robot_id = 2
    cmd.target_pos = tp
    return cmd

def test_target_array_maps_fields(manager, test_robot, test_robot2, test_command, test_command2):
    manager._movement_commands = [test_command, test_command2]
    manager._team_color_yellow = True
    manager.yellow_robots = [test_robot, test_robot2]
    manager._static_obstacles = {'border_area': True, 'center_circle': False}
    manager._goal_keeper_id = 1

    target_array_msg = manager.Target_Array()

    assert len(target_array_msg.targets) == 2

    target1 = target_array_msg.targets[0]
    assert target1.robot_id == 2
    assert target1.initial_pos.x == test_robot2.position_x
    assert target1.initial_pos.y == test_robot2.position_y
    assert target1.initial_vel.x == test_robot2.velocity_x
    assert target1.initial_vel.y == test_robot2.velocity_y
    assert target1.target_pos.x == test_command2.target_pos.x
    assert target1.target_pos.y == test_command2.target_pos.y
    assert target1.planning_options.avoid_penalty_area is True
    assert target1.planning_options.avoid_center_area is False

    target2 = target_array_msg.targets[1]
    assert target2.robot_id == 1
    assert target2.initial_pos.x == test_robot.position_x
    assert target2.initial_pos.y == test_robot.position_y
    assert target2.initial_vel.x == test_robot.velocity_x
    assert target2.initial_vel.y == test_robot.velocity_y
    assert target2.target_pos.x == test_command.target_pos.x
    assert target2.target_pos.y == test_command.target_pos.y
    assert target2.planning_options.avoid_penalty_area is True
    assert target2.planning_options.avoid_center_area is False

def test_target_array_goal_keeper_last(manager, test_robot, test_robot2, test_command, test_command2):
    manager._movement_commands = [test_command, test_command2]
    manager._team_color_yellow = True
    manager.yellow_robots = [test_robot, test_robot2]
    manager._static_obstacles = {'border_area': True, 'center_circle': False}
    manager._goal_keeper_id = 2

    target_array_msg = manager.Target_Array()

    assert len(target_array_msg.targets) == 2
    assert target_array_msg.targets[-1].robot_id == 2