#!/bin/bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/isaac_ros_common/src/install/setup.bash
export BAG=$1
export OFFSET=$2
export CUDA_LAUNCH_BLOCKING=1
ros2 launch radar_drone_launch mapgen.launch.py
