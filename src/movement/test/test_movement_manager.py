import pytest
from unittest.mock import MagicMock
from movement_interfaces.msg import MovementCommand

from movement.movement_manager import MovementManager


def _make_command(robot_id, target_pos, target_vel=(0.0, 0.0), **options):
    """A real MovementCommand: Target copies its fields straight across, so a mock
    would only be rejected by the message's own type checking."""
    cmd = MovementCommand()
    cmd.robot_id = robot_id
    cmd.target_pos.x, cmd.target_pos.y = float(target_pos[0]), float(target_pos[1])
    cmd.target_vel.x, cmd.target_vel.y = float(target_vel[0]), float(target_vel[1])
    cmd.planning_options.avoid_penalty_area = options.get('avoid_penalty_area', True)
    cmd.planning_options.avoid_center_area = options.get('avoid_center_area', False)
    cmd.planning_options.avoid_ball = options.get('avoid_ball', False)
    return cmd


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
    return _make_command(1, (300.0, 200.0))


@pytest.fixture
def movement_manager():
    mgr = MovementManager.__new__(MovementManager)

    mgr.get_logger = MagicMock()
    mgr.create_subscription = MagicMock()
    mgr.create_service = MagicMock()
    mgr.create_publisher = MagicMock()

    mgr._target_array_pub = MagicMock()

    mgr._movement_commands = None
    mgr._robots = None
    mgr._vision_stamp = 0.0
    mgr._static_obstacles = None
    mgr._goal_keeper_id = None

    return mgr


@pytest.fixture
def manager(movement_manager):
    return movement_manager


def test_set_static_obstacles(manager):
    request = MagicMock()
    request.border_area = True
    request.center_area = False
    response = MagicMock()

    manager._set_static_obstacles(request, response)

    assert manager._static_obstacles == {'border_area': True, 'center_area': False}


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


def test_game_state_callback_trigger(manager, test_robot):
    manager.try_publish_targets = MagicMock()
    msg = MagicMock()
    msg.ally_robots = [test_robot]

    manager.game_state_callback(msg)

    assert manager._robots == [test_robot]
    manager.try_publish_targets.assert_called_once()


def test_game_state_callback_captures_the_vision_stamp(manager, test_robot):
    manager.try_publish_targets = MagicMock()
    msg = MagicMock()
    msg.ally_robots = [test_robot]
    msg.vision_wall_stamp = 1234.5

    manager.game_state_callback(msg)

    assert manager._vision_stamp == 1234.5


def test_try_publish_targets_no_commands(manager, test_robot):
    manager._robots = [test_robot]
    manager._static_obstacles = {'center_area': False}
    manager._goal_keeper_id = 1

    manager.try_publish_targets()
    manager._target_array_pub.publish.assert_not_called()


def test_try_publish_targets_no_robots(manager, test_command):
    manager._movement_commands = [test_command]
    manager._static_obstacles = {'center_area': False}
    manager._goal_keeper_id = 1

    manager.try_publish_targets()
    manager._target_array_pub.publish.assert_not_called()


def test_try_publish_targets_without_either_override(manager, test_command, test_robot):
    """
    SetStaticObstacles and SetGoalKeeper are optional overrides. Requiring them meant
    nothing was ever published unless somebody called both, and nobody did.
    """
    manager._movement_commands = [test_command]
    manager._robots = [test_robot]

    manager.try_publish_targets()
    manager._target_array_pub.publish.assert_called_once()


def test_try_publish_targets_success(manager, test_command, test_robot):
    manager._movement_commands = [test_command]
    manager._robots = [test_robot]
    manager._static_obstacles = {'center_area': False, 'border_area': True}
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
    return _make_command(1, (100.0, 200.0), target_vel=(5.0, 6.0))


@pytest.fixture
def test_command2():
    return _make_command(2, (300.0, 400.0))


def test_build_target_array_maps_fields(manager, test_robot1, test_robot2, test_command1, test_command2):
    manager._movement_commands = [test_command1, test_command2]
    manager._robots = [test_robot1, test_robot2]
    manager._static_obstacles = {'center_area': False, 'border_area': True}
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
    assert target1.target_vel.x == test_command1.target_vel.x
    assert target1.target_vel.y == test_command1.target_vel.y
    # The goalkeeper is the one robot allowed inside its own penalty area.
    assert target1.planning_options.avoid_penalty_area is False
    assert target1.planning_options.avoid_center_area is False
    assert target1.vision_stamp == manager._vision_stamp

    target2 = result.targets[1]
    assert target2.robot_id == 2
    assert target2.planning_options.avoid_penalty_area is True


def test_build_target_array_goalkeeper_last(manager, test_robot1, test_robot2, test_command1, test_command2):
    manager._movement_commands = [test_command1, test_command2]
    manager._robots = [test_robot1, test_robot2]
    manager._static_obstacles = {'center_area': False, 'border_area': True}
    manager._goal_keeper_id = 2

    result = manager._build_target_array()

    assert len(result.targets) == 2
    assert result.targets[-1].robot_id == 2

def test_the_command_carries_its_own_planning_options(manager, test_robot1):
    """Per-robot options replace the driver's UpdateObstacle service."""
    manager._movement_commands = [
        _make_command(1, (100.0, 200.0), avoid_penalty_area=True, avoid_ball=True)
    ]
    manager._robots = [test_robot1]

    options = manager._build_target_array().targets[0].planning_options

    assert options.avoid_penalty_area is True
    assert options.avoid_ball is True


def test_the_static_obstacle_override_wins_over_the_command(manager, test_robot1):
    manager._movement_commands = [_make_command(1, (100.0, 200.0), avoid_center_area=False)]
    manager._robots = [test_robot1]
    manager._static_obstacles = {'center_area': True, 'border_area': True}

    options = manager._build_target_array().targets[0].planning_options

    assert options.avoid_center_area is True
