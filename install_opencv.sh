#!/bin/bash
cd isaac_ros_common/opencv/build
sudo make install
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib

