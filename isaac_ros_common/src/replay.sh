#!/bin/bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
source /workspaces/isaac_ros-dev/isaac_ros_common/src/install/setup.bash
export BAG=$1
ros2 launch radar_drone_launch replay.launch.py
