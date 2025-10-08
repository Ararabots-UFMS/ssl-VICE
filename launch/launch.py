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
            remappings=[
                ('set_orientation', 'blue/set_orientation'),
                ('control_command', 'blue/control_command'),
                ('commandTopic', 'blue/commandTopic'),
                ('set_team_color', 'blue/set_team_color'),
                ('strategy_command', 'blue/strategy_command'),
                ('update_obstacles', 'blue/update_obstacles'),
                ('update_pid', 'blue/update_pid'),
            ]
        ),
        Node(
            package='grsim_messenger',
            executable='grsim_publisher_node',
            remappings=[
                ('commandTopic', 'blue/commandTopic'),
            ]
        ),
        Node(
            package='strategy_command_gui',
            executable='strategy_gui',
            remappings=[
                ('set_orientation', 'blue/set_orientation'),
                ('set_team_color', 'blue/set_team_color'),
                ('strategy_command', 'blue/strategy_command'),
                ('update_obstacles', 'blue/update_obstacles'),
                ('update_pid', 'blue/update_pid'),
            ]
        ),
        Node(
            package='vision',
            executable='visionNode',
        ),
        Node(
            package='control_unit',
            executable='mainNode',
            remappings=[
                ('set_orientation', 'yellow/set_orientation'),
                ('control_command', 'yellow/control_command'),
                ('commandTopic', 'yellow/commandTopic'),
                ('set_team_color', 'yellow/set_team_color'),
                ('strategy_command', 'yellow/strategy_command'),
                ('update_obstacles', 'yellow/update_obstacles'),
                ('update_pid', 'yellow/update_pid'),
            ]
        ),
        Node(
            package='grsim_messenger',
            executable='grsim_publisher_node',
            remappings=[
                ('commandTopic', 'yellow/commandTopic'),
            ]
        ),
        Node(
            package='strategy_command_gui',
            executable='strategy_gui',
            remappings=[
                ('set_orientation', 'yellow/set_orientation'),
                ('set_team_color', 'yellow/set_team_color'),
                ('strategy_command', 'yellow/strategy_command'),
                ('update_obstacles', 'yellow/update_obstacles'),
                ('update_pid', 'yellow/update_pid'),
            ]
        ),
    ])
