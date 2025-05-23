#!/bin/bash
source /opt/ros/humble/setup.bash
source /root/ssl-VICE/install/setup.bash
colcon build
ros2 run gui_interpreter apiNode
