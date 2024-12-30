import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import numpy as np
from scipy.spatial.transform import Rotation as R

from launch.actions import TimerAction

def generate_launch_description():

    bag_replay = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', os.environ['BAG'],"--start-offset", "0"],
        output='screen'
    )

    #"--start-offset", "0",
    # "--qos-profile-overrides-path", "/workspaces/isaac_ros-dev/isaac_ros_common/src/qos_override.yaml"
        
    # imu_integrator = Node(
    #     package='perception',
    #     executable='imu_integrator',
    #     name='imu_integrator',
    #     output='screen'
    # )

    # base_link2payload = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='os_base_static_tf_pub',
    #     arguments=[
    #         '0', '0', '0',
    #         '0', '0', '0', '1',
    #         'base_link','payload']
    # )

    # payload2altosRadar = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='os_base_static_tf_pub',
    #     arguments=[
    #         '0.115', '0', '0.11',
    #         '0', '0', '0', '1',
    #         'payload','altosRadar']
    # )

    # payload2os_sensor = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='os_base_static_tf_pub',
    #     arguments=[
    #         '0', '0', '0',
    #         '1', '0', '0', '0',
    #         'payload','os_sensor']
    # )

    # payload2camera_link = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='os_base_static_tf_pub',
    #     arguments=[
    #         '0.115', '0.05', '0.2',
    #         '0', '0', '0', '1',
    #         'payload','camera_link']
    # )

    # camera_color_frame2camera_color_optical_frame = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='os_base_static_tf_pub',
    #     arguments=[
    #         '0', '0', '0',
    #         '-0.5', '0.5', '-0.5', '0.5',
    #         'camera_color_frame','camera_color_optical_frame']
    # )

    # camera_color_frame2camera_accel_frame = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='os_base_static_tf_pub',
    #     arguments=[
    #         '-0.0024994454383850096', '-0.018502710342407226', '0.0002345915138721466',
    #         '-0.005852920934557915', '0.0007018941687420011', '0.0002480990660842508', '0.9999825954437256',
    #         'camera_color_frame','camera_accel_frame']
    # )

    # camera_accel_frame2camera_accel_optical_frame = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='os_base_static_tf_pub',
    #     arguments=[
    #         '0', '0', '0',
    #         '-0.5', '0.5', '-0.5', '0.5',
    #         'camera_accel_frame','camera_accel_optical_frame']
    # )

    # camera_color_frame2camera_gyro_frame = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='os_base_static_tf_pub',
    #     arguments=[
    #         '-0.0024994454383850096', '-0.018502710342407226', '0.0002345915138721466',
    #         '-0.005852920934557915', '0.0007018941687420011', '0.0002480990660842508', '0.9999825954437256',
    #         'camera_color_frame','camera_gyro_frame']
    # )

    # camera_gyro_frame2camera_gyro_optical_frame = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='os_base_static_tf_pub',
    #     arguments=[
    #         '0', '0', '0',
    #         '-0.5', '0.5', '-0.5', '0.5',
    #         'camera_gyro_frame','camera_gyro_optical_frame']
    # )

    # camera_link2camera_color_frame = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='os_base_static_tf_pub',
    #     arguments=[
    #         '0', '0', '0',
    #         '0', '0', '0', '1',
    #         'camera_link','camera_color_frame']
    # )

    # os_sensor2os_lidar = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='os_base_static_tf_pub',
    #     arguments=[
    #         '0', '0', '0.03618',
    #         '0', '0', '1', '0',
    #         'os_sensor','os_lidar']
    # )

    # os_sensor2os_imu = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='os_base_static_tf_pub',
    #         arguments=[
    #         '0.006253', '-0.011775', '0.007645',
    #         '0', '0', '0', '1',
    #         'os_sensor','os_imu']
    # )

    # perception = Node(
    #     package='perception',
    #     executable='perception',
    #     name='perception',
    #     output='screen'
    # )

    rviz_old = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', '/workspaces/isaac_ros-dev/isaac_ros_common/src/rviz_camlidrad.rviz']
    )

    return LaunchDescription([
        rviz_old,
        bag_replay,
        # base_link2payload,
        # payload2altosRadar,
        # payload2os_sensor,
        # payload2camera_link,
        # camera_color_frame2camera_color_optical_frame,
        # camera_color_frame2camera_accel_frame,
        # camera_accel_frame2camera_accel_optical_frame,
        # camera_color_frame2camera_gyro_frame,
        # camera_gyro_frame2camera_gyro_optical_frame,
        # camera_link2camera_color_frame,
        # os_sensor2os_lidar,
        # os_sensor2os_imu,
        # imu_integrator,
        # perception
    ])
