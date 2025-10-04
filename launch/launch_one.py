from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='vision',
            executable='visionNode',
        ),
        Node(
            package='control_unit',
            executable='mainNode',
        ),
        Node(
            package='grsim_messenger',
            executable='grsim_publisher_node',
        ),
        Node(
            package='gui_interpreter',
            executable='apiNode',
        ),
        Node(
            package='strategy_command_gui',
            executable='strategy_gui',
        ),

    ])        