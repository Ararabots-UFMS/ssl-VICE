#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # PathDriver node
        Node(
            package='new_movement',
            executable='driver',
            name='path_driver',
            output='screen'
        ),
        
        # Strategy Command GUI
        Node(
            package='strategy_command_gui',
            executable='strategy_gui',
            name='strategy_command_gui',
            output='screen'
        ),
        
        Node(
            package='control_unit',
            executable='gameWatcher',
            name='game_watcher',
            output='screen'
        ),

        Node(
            package='vision',
            executable='visionNode',
            name='visionNode',
            output='screen'
        ),
    ])
