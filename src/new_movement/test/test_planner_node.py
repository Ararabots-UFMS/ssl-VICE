import unittest
import rclpy
from rclpy.node import Node
import launch
import launch_ros.actions
import launch_testing.actions
import pytest
from movement_interfaces.msg import TargetArray, Target, Trajectory
from system_interfaces.msg import GameState, Robot, Ball
from movement_interfaces.msg import Vector2D as Vector2DMsg
from movement_interfaces.msg import TrajectoryPoint as TrajectoryPointMsg

@pytest.mark.launch_test
def generate_test_description():
    planner_node = launch_ros.actions.Node(
        package='new_movement',
        executable='planner_node', # Ensure this matches your setup.py entry point
        name='planner_node',
    )

    return launch.LaunchDescription([
        planner_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'planner_node': planner_node}

class TestPlannerNodeLink(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_node')
        self.target_pub = self.node.create_publisher(TargetArray, 'movement_manager/targets', 10)
        self.game_state_pub = self.node.create_publisher(GameState, 'game_state', 10)
        self.received_msgs = []
        self.sub = self.node.create_subscription(
            Trajectory, 'planner/trajectories', lambda msg: self.received_msgs.append(msg), 10)

    def tearDown(self):
        self.node.destroy_node()

    def test_node_publishes_trajectory(self):
        # 1. Create a dummy GameState
        gs = GameState()
        ball = Ball()
        ball.position_x, ball.position_y = 0.0, 0.0
        gs.balls = [ball]
        self.game_state_pub.publish(gs)

        # 2. Create a dummy Target
        target_array = TargetArray()
        t = Target()
        t.robot_id = 1
        t.target_pos = Vector2DMsg(x=1000.0, y=0.0)
        t.target_vel = Vector2DMsg(x=0.0, y=0.0)
        t.initial_pos = Vector2DMsg(x=0.0, y=0.0)
        t.initial_vel = Vector2DMsg(x=0.0, y=0.0)
        # Mock planning options (adjust based on your actual message definition)
        t.planning_options.avoid_ball = True
        target_array.targets = [t]

        # 3. Spin and Publish until we get a response
        end_time = self.node.get_clock().now().nanoseconds + 5e9 # 5 seconds timeout
        while rclpy.ok() and self.node.get_clock().now().nanoseconds < end_time:
            self.game_state_pub.publish(gs)
            self.target_pub.publish(target_array)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.received_msgs:
                break

        # 4. Assertions
        self.assertTrue(len(self.received_msgs) > 0, "No trajectory was published by the planner_node")
        self.assertEqual(self.received_msgs[0].robot_id, 1)
        self.assertTrue(len(self.received_msgs[0].segments) > 0)
