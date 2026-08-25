import pytest
from unittest.mock import MagicMock, patch
from time import time

from new_movement.entities.States import State, Vector2D
from new_movement.entities.Trajectory import Trajectory, TrajectorySegment
from new_movement.utilities.trajectory_generator.TrajGenerator import TrajectoryGenerator

from movement_interfaces.msg import (
    Trajectory as TrajectoryMsg,
    TrajectorySegment as TrajectorySegmentMsg,
    TrajectoryPoint as TrajectoryPointMsg,
)

from movement_interfaces.msg import TargetArray
from system_interfaces.msg import GameState

from new_movement.optimizer_node import OptimizerNode

@pytest.fixture
def optimizer_node():
    with patch('rclpy.init'), \
         patch('rclpy.node.Node.__init__', return_value=None), \
         patch.object(OptimizerNode, 'create_subscription', return_value=MagicMock()), \
         patch.object(OptimizerNode, 'create_publisher', return_value=MagicMock()), \
         patch.object(OptimizerNode, 'create_timer', return_value=MagicMock()), \
         patch.object(OptimizerNode, 'declare_parameter', return_value=MagicMock()), \
         patch.object(OptimizerNode, 'get_parameter', return_value=MagicMock(value=50.0)), \
         patch.object(OptimizerNode, 'get_logger', return_value=MagicMock()):
        node = OptimizerNode()
        return node

@pytest.fixture
def trajectory_msg():
    generator = TrajectoryGenerator()
    start = State(Vector2D(0, 0), Vector2D(0, 0))
    goal = State(Vector2D(1000, 0), Vector2D(0, 0))
    segment = generator.generate(start, goal)
    trajectory = Trajectory(segment)
    return trajectory.to_msg(robot_id=0)

class TestGetRemainingTime:
    def test_no_active_trajectory(self, optimizer_node):
        assert optimizer_node.get_remaining_time(0) == 0.0

    def test_with_active_trajectory(self, optimizer_node):
        optimizer_node.active_durations[0] = 5.0
        optimizer_node.active_start_times[0] = time() - 2.0
        remaining = optimizer_node.get_remaining_time(0)
        assert 2.5 < remaining < 3.5

    def test_expired_trajectory_returns_zero(self, optimizer_node):
        optimizer_node.active_durations[0] = 1.0
        optimizer_node.active_start_times[0] = time() - 10.0
        assert optimizer_node.get_remaining_time(0) == 0.0

class TestOptimizeForRobot:
    def test_keeps_current_if_faster(self, optimizer_node, trajectory_msg):
        optimizer_node.active_durations[0] = 100.0
        optimizer_node.active_start_times[0] = time()
        optimizer_node.cur_targets = None

        result = optimizer_node.optimize_for_robot(0, trajectory_msg)
        assert result is None

    def test_returns_none_if_no_targets(self, optimizer_node, trajectory_msg):
        optimizer_node.active_durations[0] = 100.0
        optimizer_node.active_start_times[0] = time()
        optimizer_node.cur_targets = None

        result = optimizer_node.optimize_for_robot(0, trajectory_msg)
        assert result is None

    def test_returns_none_if_robot_not_in_targets(self, optimizer_node, trajectory_msg):
        optimizer_node.active_durations[0] = 100.0
        optimizer_node.active_start_times[0] = time()

        targets = MagicMock()
        targets.targets = []
        optimizer_node.cur_targets = targets

        result = optimizer_node.optimize_for_robot(0, trajectory_msg)
        assert result is None

    def test_optimizes_if_new_is_faster(self, optimizer_node, trajectory_msg):
        optimizer_node.active_durations[0] = 100.0
        optimizer_node.active_start_times[0] = time()

        target = MagicMock()
        target.robot_id = 0
        targets = MagicMock()
        targets.targets = [target]
        optimizer_node.cur_targets = targets
        optimizer_node.game_state = None

        optimizer_node.factory.create_obstacles = MagicMock(return_value=[])

        result = optimizer_node.optimize_for_robot(0, trajectory_msg)
        assert result is not None
        robot_id, optimized = result
        assert robot_id == 0
        assert optimized.root is not None

    def test_optimized_duration_is_shorter(self, optimizer_node, trajectory_msg):
        optimizer_node.active_durations[0] = 100.0
        optimizer_node.active_start_times[0] = time()

        target = MagicMock()
        target.robot_id = 0
        targets = MagicMock()
        targets.targets = [target]
        optimizer_node.cur_targets = targets
        optimizer_node.game_state = None

        optimizer_node.factory.create_obstacles = MagicMock(return_value=[])

        original_duration = Trajectory.from_msg(trajectory_msg).get_total_duration()
        result = optimizer_node.optimize_for_robot(0, trajectory_msg)

        assert result is not None
        _, optimized = result
        assert optimized.get_total_duration() <= original_duration + 1e-6

class TestCallbacks:
    def test_trajectory_callback_stores_msg(self, optimizer_node, trajectory_msg):
        optimizer_node.trajectory_callback(trajectory_msg)
        assert 0 in optimizer_node.pending_trajectories
        assert optimizer_node.pending_trajectories[0] == trajectory_msg

    def test_overhead_callback_stores_msg(self, optimizer_node):
        msg = MagicMock()
        msg.robot_id = 0
        optimizer_node.overhead_callback(msg)
        assert optimizer_node.cur_overhead_points[0] == msg

    def test_game_state_callback_stores_msg(self, optimizer_node):
        msg = MagicMock()
        optimizer_node.game_state_callback(msg)
        assert optimizer_node.game_state == msg

    def test_target_callback_stores_msg(self, optimizer_node):
        msg = MagicMock()
        optimizer_node.target_callback(msg)
        assert optimizer_node.cur_targets == msg