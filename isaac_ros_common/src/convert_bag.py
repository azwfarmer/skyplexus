from rosbags import converter

converter.convert(srcs='/workspaces/isaac_ros-dev/isaac_ros_common/src/data/rosbag2_2024_08_24-17_29_54', 
                        dst='/workspaces/isaac_ros-dev/isaac_ros_common/src/data/rosbag2_2024_08_24-17_29_54.bag',
                        include_topics='/ouster/points /left/image_raw /left/camera_info /altos/altosRadar /ouster/imu /mavros/imu/data')