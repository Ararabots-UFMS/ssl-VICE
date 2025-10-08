from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    return LaunchDescription([
        
        ExecuteProcess(
            cmd=['bash', '-c', os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts', 'run_controller_with_forwarder.sh'))],
            output='screen'
        ),
        Node(
            package='vision',
            executable='visionNode',
        ),
        Node(
            package='control_unit',
            executable='gameWatcher',
        ),
        Node(
            package='control',
            executable='controller',
        ),
        Node(
            package='new_movement',
            executable='driver',
        ),
        Node(
            package='grsim_messenger',
            executable='grsim_publisher_node',
        ),
        Node(
            package='referee',
            executable='referee_node',
            parameters=[{"forward_port": 10003, "verbose": False}],
        ),
        Node(
            package='strategy_command_gui',
            executable='strategy_gui',
        ),
    ])
