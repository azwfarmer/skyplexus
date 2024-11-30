#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 run tf2_ros tf2_echo $1 $2
