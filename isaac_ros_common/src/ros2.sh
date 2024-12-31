#!/bin/bash

# Check if any arguments are passed
if [ "$#" -eq 0 ]; then
  echo "Usage: $0 <command>"
  exit 1
fi

. /opt/ros/humble/setup.bash
. /workspaces/isaac_ros-dev/isaac_ros_common/src/install/setup.bash

# Execute the command passed as arguments
"ros2 $@"
