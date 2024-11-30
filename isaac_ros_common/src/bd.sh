source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/isaac_ros_common/src/install/setup.bash
export PACKAGE=$1
export CUDA_LAUNCH_BLOCKING=1
colcon build --packages-select $PACKAGE --cmake-args -DTORCH_USE_CUDA_DSA=ON

