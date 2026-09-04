from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
               package="vision",
               executable="visionNode",
               parameters=[{"ip": "224.5.23.2", "port": 10020}],
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
                package="movement",
                executable="planner",
            ),
            Node(
                package="movement",
                executable="manager",
            ),
            Node(
                package="movement",
                executable="tracker",
            ),
            Node(
                package="grsim_messenger",
                executable="grsim_publisher_node",
            ),
            #Node(
            #   package="strategy",
            #   executable="strategyNode",
            #),
            Node(
                package="gui_interpreter",
                executable="apiNode",
            ),
            Node(
                 package="referee",
                 executable="referee_node",
             )
        ]
    )
