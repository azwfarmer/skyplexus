#!/bin/bash
#gnome-terminal -- bash -c "ros2 launch altosradar rviz_launch.py;read"
# sleep 3s
# gnome-terminal -- bash -c "ros2 run altosradar altosRadarParse"./sc
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
sudo chmod 666 /dev/ttyTHS1
source /workspaces/isaac_ros-dev/isaac_ros_common/src/install/setup.bash 

#./run_preptp.sh
#./run_ptp.sh

# ros2 launch radar_drone_launch apm_altos.launch.py
