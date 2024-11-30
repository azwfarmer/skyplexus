import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import launch
import numpy as np
import launch_ros.actions

def generate_launch_description():

    map2home = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map2home',
        arguments=['--x', '0.3009932041168213', '--y', '-8.091190338134766', '--z', '39.90847396850586',
            '--roll', '0.0', '--pitch', '0.0','--yaw', '0.0',
            '--frame-id', 'map', '--child-frame-id', 'home']
    )

    payload2os_sensor = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='payload2os_sensor',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', str(np.pi), '--pitch', '0.0','--yaw', '0.0',
            '--frame-id', 'payload', '--child-frame-id', 'os_sensor']
    )

    payload2radar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='payload2radar',
        arguments=['--x', '0.11', '--y', '0.008', '--z', '0.116',
            '--roll', '0.0', '--pitch', '0.0','--yaw', '0.0',
            '--frame-id', 'payload', '--child-frame-id', 'altosRadar']
    )

    payload2camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='payload2camera_link',
        arguments=["0.127","0.03","0.197",
            "0","0","0",
            'payload','camera_link']
    )

    camera_color_frame2camera_color_optical_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_color_frame2camera_color_optical_frame',
        arguments=[
            '0', '0', '0',
            '-0.5', '0.5', '-0.5', '0.5',
            'camera_color_frame','camera_color_optical_frame']
    )

    camera_color_frame2camera_accel_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_color_frame2camera_accel_frame',
        arguments=[
            '-0.0024994454383850096', '-0.018502710342407226', '0.0002345915138721466',
            '-0.005852920934557915', '0.0007018941687420011', '0.0002480990660842508', '0.9999825954437256',
            'camera_color_frame','camera_accel_frame']
    )

    camera_accel_frame2camera_accel_optical_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_accel_frame2camera_accel_optical_frame',
        arguments=[
            '0', '0', '0',
            '-0.5', '0.5', '-0.5', '0.5',
            'camera_accel_frame','camera_accel_optical_frame']
    )

    camera_color_frame2camera_gyro_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_color_frame2camera_gyro_frame',
        arguments=[
            '-0.0024994454383850096', '-0.018502710342407226', '0.0002345915138721466',
            '-0.005852920934557915', '0.0007018941687420011', '0.0002480990660842508', '0.9999825954437256',
            'camera_color_frame','camera_gyro_frame']
    )

    camera_gyro_frame2camera_gyro_optical_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_gyro_frame2camera_gyro_optical_frame',
        arguments=[
            '0', '0', '0',
            '-0.5', '0.5', '-0.5', '0.5',
            'camera_gyro_frame','camera_gyro_optical_frame']
    )

    camera_link2camera_color_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link2camera_color_frame',
        arguments=[
            '0', '0', '0',
            '0', '0', '0', '1',
            'camera_link','camera_color_frame']
    )

    os_sensor2os_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='os_sensor2os_lidar',
        arguments=[
            '0', '0', '0.03618',
            '0', '0', '1', '0',
            'os_sensor','os_lidar']
    )

    os_sensor2os_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='os_sensor2os_imu',
            arguments=[
            '0.006253', '-0.011775', '0.007645',
            '0', '0', '0', '1',
            'os_sensor','os_imu']
    )

    base_link2payload = Node(
        package='perception',
        executable='imu_orientation',
        name='base_link2payload'
    )

    mapgen = Node(
        package='py_perception',
        executable='map_gen',
        name='map_gen',
    )

    bag_replay = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', os.environ['BAG'],"--start-offset", os.environ['OFFSET']],
        output='log'
    )
    return LaunchDescription([
        map2home,
        base_link2payload,
        payload2radar,
        payload2os_sensor,
        payload2camera_link,
        camera_color_frame2camera_color_optical_frame,
        camera_color_frame2camera_accel_frame,
        camera_accel_frame2camera_accel_optical_frame,
        camera_color_frame2camera_gyro_frame,
        camera_gyro_frame2camera_gyro_optical_frame,
        camera_link2camera_color_frame,
        os_sensor2os_lidar,
        os_sensor2os_imu,
        mapgen,
        bag_replay
    ])
