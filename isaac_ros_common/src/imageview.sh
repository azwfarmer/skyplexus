#!/bin/bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
source /workspaces/isaac_ros-dev/isaac_ros_common/src/install/setup.bash
ros2 run image_view image_view image:=/overlay
