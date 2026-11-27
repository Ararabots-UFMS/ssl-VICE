"""
Robot identification in grSim. Starts only vision, gameWatcher and the grSim messenger:
the controller, planner and tracker would also publish on commandTopic. Then run

    ros2 run control identify_robot --ros-args -p confirm:=true -p robot_id:=0 -p axis:=x
    ros2 run control identify_robot --fit identify_robot.csv

and put the numbers in ROBOT in sim_one.py.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="vision",
                executable="visionNode",
                # local_only: another grSim on the network multicasts to this group too.
                # max_time_undetected: drop a lost robot fast, so identify_robot aborts.
                parameters=[{"ip": "224.5.23.2", "port": 10020, "local_only": True,
                             "max_time_undetected": 0.1}],
            ),
            Node(
                package="control_unit",
                executable="gameWatcher",
            ),
            Node(
                package="grsim_messenger",
                executable="grsim_publisher_node",
            ),
        ]
    )
