#!/bin/bash
. /opt/ros/humble/setup.bash
ros2 bag play $1 --start-offset $2
