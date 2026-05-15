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
    def __init__(self, response, exception=None):
        self._response = response
        self._exception = exception

    def done(self):
        return True

    def exception(self):
        return self._exception

    def result(self):
        if self._exception:
            raise self._exception
        return self._response

    def add_done_callback(self, callback):
        callback(self)


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
    mgr.create_service = MagicMock()
    mgr.create_publisher = MagicMock()
    mgr.create_timer = MagicMock()

    mgr._target_array_pub = MagicMock()
    mgr._team_color_cli = MagicMock()

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


def test_poll_game_config(manager, game_config_response):
    manager._team_color_cli.wait_for_service.return_value = True
    manager._team_color_cli.call_async.return_value = FakeFuture(game_config_response)

    manager._poll_game_config()

    assert manager._team_color_yellow is True


def test_poll_game_config_skip(manager):
    manager._team_color_cli.wait_for_service.return_value = False
    manager._poll_game_config()
    manager._team_color_cli.call_async.assert_not_called()


def test_set_static_obstacles(manager):
    request = MagicMock()
    request.border_area = True
    request.center_circle = False
    response = MagicMock()

    manager._set_static_obstacles(request, response)

    assert manager._static_obstacles == {'border_area': True, 'center_circle': False}


def test_set_goal_keeper(manager):
    request = MagicMock()
    request.robot_id = 3
    response = MagicMock()

    manager._set_goal_keeper(request, response)

    assert manager._goal_keeper_id == 3


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
    manager._static_obstacles = {'center_circle': False}
    manager._goal_keeper_id = 1

    manager.try_publish_targets()
    manager._target_array_pub.publish.assert_not_called()


def test_try_publish_targets_no_team_color(manager, test_command):
    manager._movement_commands = [test_command]
    manager.yellow_robots = []
    manager._static_obstacles = {'center_circle': False}
    manager._goal_keeper_id = 1

    manager.try_publish_targets()
    manager._target_array_pub.publish.assert_not_called()


def test_try_publish_targets_no_robots(manager, test_command):
    manager._movement_commands = [test_command]
    manager._team_color_yellow = True
    manager.yellow_robots = None
    manager._static_obstacles = {'center_circle': False}
    manager._goal_keeper_id = 1

    manager.try_publish_targets()
    manager._target_array_pub.publish.assert_not_called()


def test_try_publish_targets_no_static_obstacles(manager, test_command, test_robot):
    manager._movement_commands = [test_command]
    manager._team_color_yellow = True
    manager.yellow_robots = [test_robot]
    manager._goal_keeper_id = 1

    manager.try_publish_targets()
    manager._target_array_pub.publish.assert_not_called()


def test_try_publish_targets_no_goal_keeper(manager, test_command, test_robot):
    manager._movement_commands = [test_command]
    manager._team_color_yellow = True
    manager.yellow_robots = [test_robot]
    manager._static_obstacles = {'center_circle': False}

    manager.try_publish_targets()
    manager._target_array_pub.publish.assert_not_called()


def test_try_publish_targets_success(manager, test_command, test_robot):
    manager._movement_commands = [test_command]
    manager._team_color_yellow = True
    manager.yellow_robots = [test_robot]
    manager._static_obstacles = {'center_circle': False, 'border_area': True}
    manager._goal_keeper_id = 1

    manager.try_publish_targets()
    manager._target_array_pub.publish.assert_called_once()


@pytest.fixture
def test_robot1():
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
def test_command1():
    cmd = MagicMock()
    cmd.robot_id = 1
    cmd.target_pos = _make_vector(100.0, 200.0)
    return cmd


@pytest.fixture
def test_command2():
    cmd = MagicMock()
    cmd.robot_id = 2
    cmd.target_pos = _make_vector(300.0, 400.0)
    return cmd


def test_build_target_array_maps_fields(manager, test_robot1, test_robot2, test_command1, test_command2):
    manager._movement_commands = [test_command1, test_command2]
    manager._team_color_yellow = True
    manager.yellow_robots = [test_robot1, test_robot2]
    manager._static_obstacles = {'center_circle': False, 'border_area': True}
    manager._goal_keeper_id = 1

    result = manager._build_target_array()

    assert len(result.targets) == 2

    target1 = result.targets[0]
    assert target1.robot_id == 1
    assert target1.initial_pos.x == test_robot1.position_x
    assert target1.initial_pos.y == test_robot1.position_y
    assert target1.initial_vel.x == test_robot1.velocity_x
    assert target1.initial_vel.y == test_robot1.velocity_y
    assert target1.target_pos.x == test_command1.target_pos.x
    assert target1.target_pos.y == test_command1.target_pos.y
    assert target1.planning_options.avoid_penalty_area is False  # goalkeeper
    assert target1.planning_options.avoid_center_area is False

    target2 = result.targets[1]
    assert target2.robot_id == 2
    assert target2.planning_options.avoid_penalty_area is True  # não é goalkeeper


def test_build_target_array_goalkeeper_last(manager, test_robot1, test_robot2, test_command1, test_command2):
    manager._movement_commands = [test_command1, test_command2]
    manager._team_color_yellow = True
    manager.yellow_robots = [test_robot1, test_robot2]
    manager._static_obstacles = {'center_circle': False, 'border_area': True}
    manager._goal_keeper_id = 2

    result = manager._build_target_array()

    assert len(result.targets) == 2
    assert result.targets[-1].robot_id == 2