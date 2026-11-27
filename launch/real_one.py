from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Real robot: placeholders until measured with identify_robot (README, "Modelo do robô").
# s, s, m/s². They deliberately err high: too high costs a little speed, too low
# brings the overshoot back.
ROBOT = {"actuation_delay": 0.04, "tau": 0.08, "max_accel": 3.0}

CONTROLLER = {
    # `ros2 launch launch/<this file> control_mode:=legacy` rolls back to the old path.
    "control_mode": LaunchConfiguration("control_mode"),
    "actuation_delay": ROBOT["actuation_delay"],
    "velocity_time_constant": ROBOT["tau"],
    "robot_max_acceleration": ROBOT["max_accel"],
    "slew_limit": ROBOT["max_accel"],
}
# Planner acceleration ~ 0.7 x min(robot max accel, slew_limit), in mm/s²: the margin
# that absorbs a late start of braking.
PLANNER = {"max_acceleration": round(0.7 * ROBOT["max_accel"] * 1000.0, 1)}


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("control_mode", default_value="predictive",
                                  description="predictive | legacy"),
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
                parameters=[CONTROLLER],
            ),
            Node(
                package="movement",
                executable="planner",
                parameters=[PLANNER],
            ),
            Node(
                package="movement",
                executable="manager",
            ),
            Node(
                package="movement",
                executable="tracker",
            ),
            # Node(
            #     package="strategy",
            #     executable="strategyNode",
            # ),
            Node(
                package="gui_interpreter",
                executable="apiNode",
            ),
            Node(
                package="referee",
                executable="referee_node",
                parameters=[{"forward_port": 10003, "verbose": False}],
            ),
            Node(
                package="hardware_messenger",
                executable="hardware"
            ),
        ]
    )
