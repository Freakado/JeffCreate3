#!/bin/bash

# Source the main ROS2 setup file
source /opt/ros/jazzy/setup.bash

# Source your personal workspace setup file
source ~/ros2_ws/install/setup.bash

# Execute your master launch file from the robot_bringup package
ros2 launch robot_bringup robot.launch.py
