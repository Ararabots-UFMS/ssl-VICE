from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="vision",
                executable="visionNode",
                parameters=[{"ip": "224.5.23.2", "port": 10006, "verbose": False}],
            ),
            Node(
                package="control_unit",
                executable="gameWatcher",
            ),
            Node(
                package="control",
                executable="controller",
            ),
            Node(
                package="new_movement",
                executable="driver",
            ),
            # Node(
            #     package="strategy",
            #     executable="strategyNode",
            # ),
            # Node(
            #     package="gui_interpreter",
            #     executable="apiNode",
            # ),
            Node(
                package="referee",
                executable="referee_node",
                parameters=[{"forward_port": 10003, "verbose": False}],
            ),
            Node(
                package="strategy_command_gui",
                executable="strategy_gui"
            ),
            Node(
                package="hardware_messenger",
                executable="hardware"
            )
        ]
    )
